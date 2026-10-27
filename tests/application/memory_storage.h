#pragma once

// RecordingStorage fake: keeps every chunk in memory.
#include "application/ports/recording_storage.h"

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace dlidar::test
{
    class MemoryStorage : public RecordingStorage
    {
    public:
        bool fail_open = false;
        int throw_at_append = -1; // Append number that throws (0 = the first)
        int room_for = -1;        // HasRoom false once this many chunks were appended (-1: always room)
        int append_delay_ms = 0;  // a slow disk

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
            if (append_delay_ms)
                std::this_thread::sleep_for(std::chrono::milliseconds(append_delay_ms));
        }
        bool HasRoom(std::string &why) override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            room_checks++;
            if (room_for < 0 || appends < room_for)
                return true;
            why = "the memory is full";
            return false;
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
        int opens = 0, closes = 0, appends = 0, room_checks = 0;
        std::vector<SensorChunk> chunks;

    private:
        std::mutex mutex_;
    };
}
