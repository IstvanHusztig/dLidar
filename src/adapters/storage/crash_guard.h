#pragma once

#include <atomic>
#include <cstdint>

namespace dlidar
{
    // The IMU CSV that the fatal-signal handler (src/app/signals) truncates to
    // its last complete row before the process dies, and that size. Kept up
    // to date by ContinuousDataWriter; -1 when no CSV is open.
    inline std::atomic<int> &CrashTruncateFd()
    {
        static std::atomic<int> fd{-1};
        return fd;
    }

    inline std::atomic<int64_t> &CrashTruncateSize()
    {
        static std::atomic<int64_t> size{0};
        return size;
    }
}
