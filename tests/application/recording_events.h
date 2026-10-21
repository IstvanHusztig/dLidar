#pragma once

// DeviceEvents fakes: one that records what it gets, one that throws, one
// that blocks until released.
#include "application/ports/device_events.h"

#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace dlidar::test
{
    class RecordingEvents : public DeviceEvents
    {
    public:
        void OnState(const DeviceStatus &s) override { Add(std::string("state:") + StateName(s.state)); }
        void OnWarning(const DeviceWarning &w) override { Add("warning:" + w.text); }
        void OnProgress(const DeviceStatus &s) override { Add("progress:" + std::to_string(s.chunks)); }
        void OnSummary(const RecordingSummary &) override { Add("summary"); }

        std::vector<std::string> Events() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return events_;
        }
        // The state events only, in order.
        std::vector<std::string> States() const
        {
            std::vector<std::string> out;
            for (const std::string &e : Events())
                if (e.compare(0, 6, "state:") == 0)
                    out.push_back(e.substr(6));
            return out;
        }

    private:
        mutable std::mutex mutex_;
        std::vector<std::string> events_;
        void Add(const std::string &e)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            events_.push_back(e);
        }
    };

    class ThrowingEvents : public DeviceEvents
    {
    public:
        void OnState(const DeviceStatus &) override { throw std::runtime_error("channel broke"); }
        void OnProgress(const DeviceStatus &) override { throw std::runtime_error("channel broke"); }
    };

    class BlockingEvents : public DeviceEvents
    {
    public:
        void OnState(const DeviceStatus &) override
        {
            std::unique_lock<std::mutex> lock(mutex_);
            calls_++;
            cv_.wait(lock, [&] { return released_; });
        }
        void Release()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                released_ = true;
            }
            cv_.notify_all();
        }
        int Calls()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return calls_;
        }

    private:
        std::mutex mutex_;
        std::condition_variable cv_;
        bool released_ = false;
        int calls_ = 0;
    };
}
