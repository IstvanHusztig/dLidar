#pragma once

#include "application/ports/device_events.h"

#include <string>
#include <utility>

namespace dlidar
{
    // The device's console output: the texts bin/dLidar has always printed
    // (start banner, dirty-lens warning, IMU timing summary, completion).
    // Register it with EventHub::AddSync so it keeps its place among the
    // other console lines.
    class ConsoleEvents : public DeviceEvents
    {
    public:
        // `sensor`: the sensor's short name in the clock summary ("L2").
        explicit ConsoleEvents(std::string sensor = "L2") : sensor_(std::move(sensor)) {}

        void OnState(const DeviceStatus &status) override;
        void OnWarning(const DeviceWarning &warning) override;
        void OnSummary(const RecordingSummary &summary) override;

    private:
        std::string sensor_;
        DeviceState last_ = DeviceState::Idle;
    };
}
