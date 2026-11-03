#pragma once

#include "application/ports/device_events.h"

#include <stdint.h>

namespace dlidar::gpio
{
    // What the status LED shows (docs/manual.html, "Wiring and pinout"):
    // on while a recording starts and runs, blinking 2 Hz while it is saved,
    // three fast flashes then off after a failure, off when idle. `ms` is the
    // time since the state began.
    inline bool LedOn(DeviceState state, int64_t ms)
    {
        switch (state)
        {
        case DeviceState::Preparing:
        case DeviceState::Recording:
            return true;
        case DeviceState::Finalizing:
            return ms % 500 < 250;
        case DeviceState::Error:
            return ms < 600 && ms % 200 < 100;
        default:
            return false;
        }
    }
}
