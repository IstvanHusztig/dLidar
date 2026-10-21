#pragma once

#include "adapters/unitree_l2/l2_packet_source.h"
#include "adapters/unitree_l2/vendor/unitree_lidar_protocol.h"
#include "domain/recording/packet_timing.h"

#include <vector>

namespace dlidar
{
    // Replays the packet sequence of a packetsNNNN.bin timing log as L2
    // packets. The log holds only timing, so packet contents are synthetic and
    // deterministic: rays of the L2 sensor model (same formula as the parser)
    // hit a box room, the IMU is at rest. Host times come from the log, so two
    // builds fed the same log write the same recording (tools/golden_replay.sh).
    class SyntheticL2Source : public L2PacketSource
    {
    public:
        // Replays `log` up to `seconds` of host time after its first packet.
        SyntheticL2Source(std::vector<PacketTiming> log, double seconds);

        int Next(HostTime &arrival) override;
        const unitree_lidar_sdk::LidarPointDataPacket &PointPacket() const override { return points_; }
        const unitree_lidar_sdk::LidarImuData &ImuPacket() const override { return imu_; }
        RecordingInfo Versions() override;

    private:
        std::vector<PacketTiming> log_;
        double seconds_;
        size_t next_ = 0;
        int64_t t0_ = 0;
        unitree_lidar_sdk::LidarPointDataPacket points_{};
        unitree_lidar_sdk::LidarImuData imu_{};
    };
}
