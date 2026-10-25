#pragma once

#include "domain/timing/time_alignment.h"

#include <stdint.h>
#include <string>

namespace dlidar
{
    // What the scanning device is doing (docs/architecture.md, device-control spec).
    enum class DeviceState
    {
        Idle,
        Preparing,  // sensor start-up before a recording
        Recording,
        Finalizing, // sensor stopped, files being finished
        Error
    };

    inline const char *StateName(DeviceState s)
    {
        switch (s)
        {
        case DeviceState::Idle: return "idle";
        case DeviceState::Preparing: return "preparing";
        case DeviceState::Recording: return "recording";
        case DeviceState::Finalizing: return "finalizing";
        case DeviceState::Error: return "error";
        }
        return "?";
    }

    struct DeviceStatus
    {
        DeviceState state = DeviceState::Idle;
        int chunks = 0;        // of the current / last recording
        uint64_t points = 0;
        std::string location;  // where the recordings go
        bool last_ok = true;   // the last recording was written completely
    };

    enum class WarningKind
    {
        LensDirty,
        StorageFailed,
        StorageSlow,     // chunks pile up in memory
        SensorFailed,
        SensorSilent,    // no points during the recording; it was stopped
        RecordingFailed  // an internal error ended the recording
    };

    struct DeviceWarning
    {
        WarningKind kind;
        std::string text; // complete, human-readable message
    };

    // End of a recording: how its IMU times were made.
    struct RecordingSummary
    {
        ImuTimeline::Mode mode = ImuTimeline::Mode::Auto;
        std::string decision;
        ImuTimeline::Stats stats;
        ImuTimeline::Clocks clocks;
    };

    // Output channel of the device (console, later LEDs, a phone over
    // Bluetooth). Called from the recording's threads (capture and writer);
    // see EventHub for isolation.
    class DeviceEvents
    {
    public:
        virtual ~DeviceEvents() = default;
        virtual void OnState(const DeviceStatus &status) = 0;
        virtual void OnWarning(const DeviceWarning &) {}
        virtual void OnProgress(const DeviceStatus &) {} // once per written chunk
        virtual void OnSummary(const RecordingSummary &) {}
    };
}
