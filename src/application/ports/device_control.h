#pragma once

#include "application/ports/device_events.h"

namespace dlidar
{
    // The commands an input channel gives the device (the button, signals,
    // the phone): ScannerController implements it. Thread-safe.
    class DeviceControl
    {
    public:
        virtual ~DeviceControl() = default;

        virtual DeviceStatus Status() const = 0;

        // Starts a recording. False (and nothing changes) unless Idle or Error;
        // `state_out` gets the state the command saw.
        virtual bool StartRecording(DeviceState *state_out = nullptr) = 0;

        // Ends the running recording normally. No-op when idle.
        virtual void StopRecording() = 0;
    };
}
