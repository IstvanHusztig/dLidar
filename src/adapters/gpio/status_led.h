#pragma once

#include "adapters/gpio/gpio_lines.h"
#include "application/ports/device_events.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

namespace dlidar::gpio
{
    // The status LED as an output channel (register it with
    // EventHub::AddAsync). OnState only records the state; its own thread
    // drives the line every 50 ms from LedOn(), so blinking never waits for
    // events and a failing line never blocks anything. The LED is off when
    // the channel ends.
    class StatusLed : public DeviceEvents
    {
    public:
        explicit StatusLed(std::unique_ptr<LedLine> line);
        ~StatusLed() override;

        void OnState(const DeviceStatus &status) override;

        uint64_t WriteFailures() const { return failures_; }

    private:
        std::unique_ptr<LedLine> line_;
        std::mutex mutex_;
        DeviceState state_ = DeviceState::Idle;
        std::chrono::steady_clock::time_point since_ = std::chrono::steady_clock::now();
        std::atomic<bool> stop_{false};
        std::atomic<uint64_t> failures_{0};
        std::thread thread_;

        void Loop();
        void Write(bool on);
    };
}
