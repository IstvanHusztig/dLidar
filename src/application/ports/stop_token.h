#pragma once

#include <atomic>

namespace dlidar
{
    // Tells long-running work that a stop was requested. Reads up to two flags,
    // e.g. the process-wide signal flag and a per-recording stop command, and
    // never owns them.
    class StopToken
    {
    public:
        StopToken() = default;
        explicit StopToken(const std::atomic<bool> *a, const std::atomic<bool> *b = nullptr) : a_(a), b_(b) {}

        bool StopRequested() const { return (a_ && a_->load()) || (b_ && b_->load()); }

    private:
        const std::atomic<bool> *a_ = nullptr;
        const std::atomic<bool> *b_ = nullptr;
    };
}
