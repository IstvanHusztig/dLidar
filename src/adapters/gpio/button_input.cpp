#include "adapters/gpio/button_input.h"

#include <iostream>
#include <time.h>
#include <vector>

namespace dlidar::gpio
{
    namespace
    {
        int64_t MonotonicNs()
        {
            timespec t;
            clock_gettime(CLOCK_MONOTONIC, &t);
            return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
        }
    }

    ButtonInput::ButtonInput(std::unique_ptr<ButtonLine> line, std::function<void()> on_short, std::function<void()> on_long)
        : line_(std::move(line)), on_short_(std::move(on_short)), on_long_(std::move(on_long)), thread_([this] { Loop(); })
    {
    }

    ButtonInput::~ButtonInput()
    {
        stop_ = true;
        if (thread_.joinable())
            thread_.join();
    }

    void ButtonInput::Loop()
    {
        std::vector<ButtonChange> changes;
        while (!stop_)
        {
            changes.clear();
            if (!line_->Wait(100, changes))
            {
                std::cerr << "WARNING: reading the button failed; the button no longer works until dLidar restarts."
                          << std::endl;
                running_ = false;
                return;
            }
            std::vector<Press> presses;
            for (const ButtonChange &c : changes)
                presses.push_back(detector_.OnLevel(c.t_ns, c.pressed));
            presses.push_back(detector_.Tick(MonotonicNs()));
            for (Press p : presses)
            {
                if (stop_)
                    return;
                if (p == Press::Short && on_short_)
                    on_short_();
                else if (p == Press::Long && on_long_)
                    on_long_();
            }
        }
    }
}
