#pragma once

#include "adapters/storage/recording_writer.h"
#include "adapters/storage/recovery.h"
#include "application/ports/recording_storage.h"

#include <memory>
#include <string>
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

        bool Open() override
        {
            writer_ = std::make_unique<ContinuousDataWriter>(profile_, profiles_);
            return writer_->Open(dir_);
        }
        void Append(const SensorChunk &chunk) override { writer_->AppendChunk(chunk); }
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
