#pragma once

#include "adapters/storage/recording_writer.h"
#include "adapters/storage/recovery.h"
#include "application/ports/recording_storage.h"

#include <functional>
#include <memory>
#include <string>
#include <sys/statvfs.h>
#include <utility>

namespace dlidar
{
    // RecordingStorage as numbered files in one directory (CSV, LAZ, JSON,
    // packet log, packet state). `profile` describes the sensor that records
    // (its packetstate header: none means no packetstate files); `profiles`
    // lists every known sensor, so leftovers of any of them are finished as
    // the sensor that recorded them (it must contain `profile`).
    class FileRecordingStorage : public RecordingStorage
    {
    public:
        FileRecordingStorage(std::string dir, const RecordingProfile &profile, std::vector<RecordingProfile> profiles)
            : dir_(std::move(dir)), profile_(profile), profiles_(std::move(profiles)) {}

        // Finishing a recording needs about half its points temp file again
        // (the LAZ was 0.38x in a 300 s Mid-360S replay), plus this margin; a
        // recording is not started with less than the margin free.
        static const uint64_t kMarginBytes = 500ull * 1024 * 1024;

        // Free bytes on the disk of `dir` (replaceable for tests).
        std::function<uint64_t(const std::string &dir)> free_bytes = [](const std::string &dir) -> uint64_t {
            struct statvfs st;
            if (statvfs(dir.c_str(), &st) != 0)
                return UINT64_MAX; // unknown: do not stop a recording for it
            return (uint64_t)st.f_bavail * st.f_frsize;
        };

        bool Open() override
        {
            uint64_t free = free_bytes(dir_);
            if (free < kMarginBytes)
            {
                std::cerr << "Error: only " << free / (1024 * 1024) << " MB free in " << dir_ << "; a recording needs at least "
                          << kMarginBytes / (1024 * 1024) << " MB." << std::endl;
                return false;
            }
            writer_ = std::make_unique<ContinuousDataWriter>(profile_, profiles_);
            return writer_->Open(dir_);
        }
        void Append(const SensorChunk &chunk) override { writer_->AppendChunk(chunk); }
        bool HasRoom(std::string &why) override
        {
            uint64_t need = writer_->PointsTempBytes() / 2 + kMarginBytes, free = free_bytes(dir_);
            if (free >= need)
                return true;
            why = "the disk is nearly full (" + std::to_string(free / (1024 * 1024)) + " MB free, " +
                  std::to_string(need / (1024 * 1024)) + " MB needed to save the recording)";
            return false;
        }
        bool Close() override
        {
            bool ok = writer_->CloseAndPackageLAZ();
            writer_.reset();
            return ok;
        }
        bool RecoverLeftovers(bool move_aside) override { return RecoverInterruptedRecording(dir_, move_aside, profiles_); }
        std::string Location() const override { return dir_; }

    private:
        std::string dir_;
        RecordingProfile profile_;
        std::vector<RecordingProfile> profiles_;
        std::unique_ptr<ContinuousDataWriter> writer_;
    };
}
