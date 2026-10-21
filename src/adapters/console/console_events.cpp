#include "adapters/console/console_events.h"

#include <iostream>

namespace dlidar
{
    void ConsoleEvents::OnState(const DeviceStatus &status)
    {
        switch (status.state)
        {
        case DeviceState::Recording:
            std::cout << ">>> STARTING CONTINUOUS DATA CAPTURE <<< (Ctrl-C to stop)" << std::endl;
            break;
        case DeviceState::Finalizing:
            std::cout << "LiDAR stopped. Flushing final data to continuous streams..." << std::endl;
            break;
        case DeviceState::Idle:
            if (last_ == DeviceState::Finalizing)
                std::cout << "Capture complete. Recording is in " << status.location << std::endl;
            break;
        case DeviceState::Error:
            std::cerr << "Recording failed. Whatever was captured is in " << status.location << std::endl;
            break;
        case DeviceState::Preparing:
            break;
        }
        last_ = status.state;
    }

    void ConsoleEvents::OnWarning(const DeviceWarning &warning)
    {
        std::cerr << warning.text << std::endl;
    }

    void ConsoleEvents::OnSummary(const RecordingSummary &r)
    {
        const ImuTimeline::Stats &st = r.stats;
        const ImuTimeline::Clocks &c = r.clocks;
        std::cout << "IMU time source: " << ImuTimeline::ModeName(r.mode) << " (" << r.decision << ")\n"
                  << "IMU: " << st.imu_out << " of " << st.imu_in << " samples written, dropped " << st.non_increasing_dropped
                  << " non-increasing, " << st.duplicates_dropped << " duplicate, " << st.negative_dropped << " before t=0, "
                  << st.unmapped_dropped << " unmapped; lost " << st.lost_samples << ", stamp jumps " << st.stamp_jumps
                  << ", nudged " << st.nudged << std::endl;
        if (c.valid)
        {
            std::cout << "Clocks over " << c.span_s << " s: IMU period " << c.imu_period_ns / 1e6 << " ms on the L2 clock ("
                      << 1e9 / c.imu_period_ns << " Hz), L2 clock vs host " << c.lidar_rate_ppm << " ppm, IMU stamps vs host "
                      << c.imu_rate_ppm << " ppm; IMU - point min latency " << c.min_latency_diff_start_ms << " -> "
                      << c.min_latency_diff_end_ms << " ms; mean extra latency IMU " << c.imu_extra_latency_ms << " ms, points "
                      << c.lidar_extra_latency_ms << " ms; the old host-stamped IMU would have lagged by "
                      << c.old_lag_start_ms << " -> " << c.old_lag_end_ms << " ms" << std::endl;
        }
    }
}
