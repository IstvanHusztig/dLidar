#include "adapters/gpio/status_led.h"

#include "adapters/gpio/led_pattern.h"

#include <iostream>

namespace dlidar::gpio
{
    StatusLed::StatusLed(std::unique_ptr<LedLine> line) : line_(std::move(line)), thread_([this] { Loop(); }) {}

    StatusLed::~StatusLed()
    {
        stop_ = true;
        if (thread_.joinable())
            thread_.join();
        Write(false);
    }

    void StatusLed::OnState(const DeviceStatus &status)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (status.state != state_)
        {
            state_ = status.state;
            since_ = std::chrono::steady_clock::now();
        }
    }

    void StatusLed::Write(bool on)
    {
        if (!line_->Set(on) && failures_++ == 0)
            std::cerr << "WARNING: setting the status LED failed; it may not show the state." << std::endl;
    }

    void StatusLed::Loop()
    {
        int last = -1; // -1: not written yet
        while (!stop_)
        {
            DeviceState state;
            std::chrono::steady_clock::time_point since;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state = state_;
                since = since_;
            }
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - since).count();
            int on = LedOn(state, ms) ? 1 : 0;
            if (on != last)
            {
                Write(on);
                last = on;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}
