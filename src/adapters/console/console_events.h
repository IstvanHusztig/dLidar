#pragma once

#include "application/ports/device_events.h"

namespace dlidar
{
    // The device's console output: the texts bin/dLidar has always printed
    // (start banner, dirty-lens warning, IMU timing summary, completion).
    // Register it with EventHub::AddSync so it keeps its place among the
    // other console lines.
    class ConsoleEvents : public DeviceEvents
    {
    public:
        void OnState(const DeviceStatus &status) override;
        void OnWarning(const DeviceWarning &warning) override;
        void OnSummary(const RecordingSummary &summary) override;

    private:
        DeviceState last_ = DeviceState::Idle;
    };
}
