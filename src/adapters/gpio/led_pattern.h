#pragma once

#include "application/ports/device_events.h"

#include <stdint.h>

namespace dlidar::gpio
{
    // What the status LED shows (docs/manual.html, "Wiring and pinout"):
    // on while a recording starts and runs, blinking 2 Hz while it is saved,
    // three fast flashes then off after a failure, off when idle. While the
    // Bluetooth pairing window is open (`pairing`) and nothing else shows,
    // two 100 ms flashes every second. `ms` is the time since the state
    // began, `pairing_ms` the time since the window opened.
    inline bool LedOn(DeviceState state, int64_t ms, bool pairing = false, int64_t pairing_ms = 0)
    {
        bool pairing_on = pairing && (pairing_ms % 1000 < 100 || (pairing_ms % 1000 >= 200 && pairing_ms % 1000 < 300));
        switch (state)
        {
        case DeviceState::Preparing:
        case DeviceState::Recording:
            return true;
        case DeviceState::Finalizing:
            return ms % 500 < 250;
        case DeviceState::Error:
            return ms < 600 ? ms % 200 < 100 : pairing_on;
        default:
            return pairing_on;
        }
    }
}
