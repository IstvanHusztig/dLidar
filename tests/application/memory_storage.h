#pragma once

// RecordingStorage fake: keeps every chunk in memory.
#include "application/ports/recording_storage.h"

#include <mutex>
#include <stdexcept>
#include <vector>

namespace dlidar::test
{
    class MemoryStorage : public RecordingStorage
    {
    public:
        bool fail_open = false;
        int throw_at_append = -1; // Append number that throws (0 = the first)

        bool Open() override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            opens++;
            return !fail_open;
        }
        void Append(const SensorChunk &chunk) override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (appends++ == throw_at_append)
                throw std::runtime_error("disk broke");
            chunks.push_back(chunk);
        }
        bool Close() override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closes++;
            return true;
        }
        bool RecoverLeftovers(bool) override { return true; }
        std::string Location() const override { return "memory"; }

        // Read after the recording finished.
        int opens = 0, closes = 0, appends = 0;
        std::vector<SensorChunk> chunks;

    private:
        std::mutex mutex_;
    };
}
