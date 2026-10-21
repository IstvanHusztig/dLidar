#pragma once

#include "adapters/unitree_l2/l2_packet_source.h"

#include <memory>
#include <stdint.h>
#include <string>

namespace unitree_lidar_sdk
{
    class UnitreeLidarReader;
}

namespace dlidar
{
    struct LiveL2Config
    {
        std::string lidar_ip;
        unsigned short lidar_port = 0;
        std::string local_ip;
        unsigned short local_port = 0;
        uint16_t cloud_scan_num = 18; // SDK default
        // false: the SDK leaves every packet with the L2's own stamp. With true
        // it would overwrite only the IMU stamp with the host arrival time, a
        // different clock from the points' (the old 50-80 ms IMU lag).
        bool use_system_timestamp = false;
        uint32_t work_mode = 0; // sent with setLidarWorkMode, recorded in timingNNNN.json
    };

    // The L2 over UDP through the vendor SDK reader.
    class LiveL2Source : public L2PacketSource
    {
    public:
        // Initializes the SDK reader and sets the work mode; nullptr (after
        // printing why) if the UDP setup fails.
        static std::unique_ptr<LiveL2Source> Open(const LiveL2Config &config);

        int Next(HostTime &arrival) override;
        const unitree_lidar_sdk::LidarPointDataPacket &PointPacket() const override;
        const unitree_lidar_sdk::LidarImuData &ImuPacket() const override;

        // Restarts the rotation, prints the dirtiness and time delay, queries
        // the versions, then reads out the SDK's packet backlog for 2 s.
        bool Prepare(const StopToken &stop) override;
        RecordingInfo Versions() override { return info_; }
        std::optional<float> LensDirtiness() override;
        void Stop() override; // stops the rotation

    private:
        explicit LiveL2Source(unitree_lidar_sdk::UnitreeLidarReader *reader, uint32_t work_mode);

        unitree_lidar_sdk::UnitreeLidarReader *reader_;
        uint32_t work_mode_;
        RecordingInfo info_;
    };
}
