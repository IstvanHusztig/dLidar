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
    // packet log, packet state). `packet_state_header` is the sensor's
    // diagnostics header; empty means no packetstate files.
    class FileRecordingStorage : public RecordingStorage
    {
    public:
        FileRecordingStorage(std::string dir, std::string packet_state_header)
            : dir_(std::move(dir)), header_(std::move(packet_state_header)) {}

        bool Open() override
        {
            writer_ = std::make_unique<ContinuousDataWriter>(header_);
            return writer_->Open(dir_);
        }
        void Append(const SensorChunk &chunk) override { writer_->AppendChunk(chunk); }
        bool Close() override
        {
            bool ok = writer_->CloseAndPackageLAZ();
            writer_.reset();
            return ok;
        }
        bool RecoverLeftovers(bool move_aside) override { return RecoverInterruptedRecording(dir_, move_aside, header_); }
        std::string Location() const override { return dir_; }

    private:
        std::string dir_;
        std::string header_;
        std::unique_ptr<ContinuousDataWriter> writer_;
    };
}
