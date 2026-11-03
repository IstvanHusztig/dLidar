#pragma once

// Test helper: hand L2 SDK packets one at a time to the real L2 adapter
// (UnitreeL2Sensor) and the core's CaptureTiming, as a live capture does.
#include "adapters/unitree_l2/l2_conversion.h"
#include "adapters/unitree_l2/unitree_l2_sensor.h"
#include "application/capture/capture_timing.h"

#include <memory>
#include <vector>

namespace dlidar::test
{
    // Returns the packet it was given last, once.
    class QueuedL2Source : public L2PacketSource
    {
    public:
        void SetPoints(const unitree_lidar_sdk::LidarPointDataPacket &p, HostTime h)
        {
            points_ = p;
            host_ = h;
            type_ = LIDAR_POINT_DATA_PACKET_TYPE;
        }
        void SetImu(const unitree_lidar_sdk::LidarImuData &d, HostTime h)
        {
            imu_ = d;
            host_ = h;
            type_ = LIDAR_IMU_DATA_PACKET_TYPE;
        }

        int Next(HostTime &arrival) override
        {
            int t = type_;
            type_ = 0;
            arrival = host_;
            return t;
        }
        const unitree_lidar_sdk::LidarPointDataPacket &PointPacket() const override { return points_; }
        const unitree_lidar_sdk::LidarImuData &ImuPacket() const override { return imu_; }
        RecordingInfo Versions() override { return RecordingInfo(); }

    private:
        int type_ = 0;
        HostTime host_{};
        unitree_lidar_sdk::LidarPointDataPacket points_{};
        unitree_lidar_sdk::LidarImuData imu_{};
    };

    class L2Feed
    {
    public:
        explicit L2Feed(const ImuTimeline::Config &config = ImuTimeline::Config(), const AxisMap &mount = AxisMap(),
                        const PointFilter &filter = l2::kDefaultFilter)
            : source_(new QueuedL2Source), sensor_(std::unique_ptr<L2PacketSource>(source_)),
              timing_(sensor_, config, mount, filter)
        {
        }

        void OnImu(const unitree_lidar_sdk::LidarImuData &d, HostTime now, std::vector<ImuSample> &imu_out)
        {
            source_->SetImu(d, now);
            std::vector<PointRecord> unused;
            timing_.OnPacket(sensor_.Poll(), unused, imu_out);
        }

        void OnPoints(const unitree_lidar_sdk::LidarPointDataPacket &p, HostTime now, std::vector<PointRecord> &points_out,
                      std::vector<ImuSample> &imu_out)
        {
            source_->SetPoints(p, now);
            timing_.OnPacket(sensor_.Poll(), points_out, imu_out);
        }

        void Flush(std::vector<ImuSample> &imu_out) { timing_.Flush(imu_out); }
        void TakePacketLog(std::vector<PacketTiming> &out) { timing_.TakePacketLog(out); }
        const ImuTimeline &Timeline() const { return timing_.Timeline(); }
        int64_t ZeroNs() const { return timing_.ZeroNs(); }
        UnitreeL2Sensor &Sensor() { return sensor_; }

    private:
        QueuedL2Source *source_; // owned by sensor_
        UnitreeL2Sensor sensor_;
        CaptureTiming timing_;
    };
}
