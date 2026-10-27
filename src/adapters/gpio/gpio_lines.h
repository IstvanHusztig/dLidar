#pragma once

#include <stdint.h>
#include <vector>

namespace dlidar::gpio
{
    // A change of the button, timed on CLOCK_MONOTONIC.
    struct ButtonChange
    {
        int64_t t_ns;
        bool pressed;
    };

    // The push button's line, as ButtonInput reads it (GpioLine, or a fake in tests).
    class ButtonLine
    {
    public:
        virtual ~ButtonLine() = default;
        // Waits up to `timeout_ms` for changes and appends them; false if
        // the line failed.
        virtual bool Wait(int timeout_ms, std::vector<ButtonChange> &out) = 0;
    };

    // The LED's line, as StatusLed drives it.
    class LedLine
    {
    public:
        virtual ~LedLine() = default;
        virtual bool Set(bool on) = 0; // false if writing failed
    };
}
