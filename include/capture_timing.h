#pragma once

#include "unitree_lidar_utilities.h"
#include "time_alignment.h"
#include "packet_state.h"

#include <time.h>
#include <vector>

using namespace unitree_lidar_sdk;

// Reject points inside the housing blind zone (spec: 0.05 m) to filter out
// self-reflection off the sensor housing. Kept a bit past the blind zone
// since accuracy degrades near it.
const float kMinPointRangeM = 0.1f;

// Reject low-reflectivity returns (0-255 scale) that are usually noise/
// multipath rather than a real surface.
const uint8_t kMinPointIntensity = 5;

// Host clocks when a packet was returned by runParse().
struct HostTime
{
    int64_t mono_ns; // CLOCK_MONOTONIC: arrival timing (never stepped)
    int64_t unix_ns; // CLOCK_REALTIME: Unix anchor
};

inline HostTime HostNow()
{
    timespec m, r;
    clock_gettime(CLOCK_MONOTONIC, &m);
    clock_gettime(CLOCK_REALTIME, &r);
    return {(int64_t)m.tv_sec * 1000000000LL + m.tv_nsec, (int64_t)r.tv_sec * 1000000000LL + r.tv_nsec};
}

// Turns parsed SDK packets into written points and IMU samples, all timed on
// the L2 clock (see ImuTimeline), and keeps the per-packet timing log.
class CaptureTiming
{
public:
    explicit CaptureTiming(const ImuTimeline::Config &config = ImuTimeline::Config()) : timeline_(config) {}

    // IMU packet just parsed. Acceleration in g, angular velocity in rad/s.
    void OnImu(const LidarImuData &imu, HostTime now, std::vector<OutputImuData> &imu_out)
    {
        ParsedImuData parsed;
        parseFromImuPacket(parsed, imu);

        ImuInput in{};
        in.data.AccelerationX = parsed.accel[0];
        in.data.AccelerationY = parsed.accel[1];
        in.data.AccelerationZ = parsed.accel[2];
        in.data.GyroX = parsed.gyro[0];
        in.data.GyroY = parsed.gyro[1];
        in.data.GyroZ = parsed.gyro[2];
        in.data.ImuId = 0;
        in.sensor_ns = parsed.sensor_stamp_ns;
        in.seq = parsed.seq;
        in.host_mono_ns = now.mono_ns;
        in.host_unix_ns = now.unix_ns;
        timeline_.PushImu(in, imu_out);

        log_.push_back({now.mono_ns, now.unix_ns, parsed.sensor_stamp_ns, parsed.seq, kPacketTypeImu, 0.0f, 0});
    }

    // Point packet just parsed. The first one sets the zero of all times.
    void OnPoints(const LidarPointDataPacket &packet, HostTime now, std::vector<PointDLidar> &points_out,
                  std::vector<OutputImuData> &imu_out)
    {
        const LidarPointData &data = packet.data;
        int64_t stamp = StampToNs(data.info.stamp);
        log_.push_back({now.mono_ns, now.unix_ns, stamp, data.info.seq, kPacketTypePoints, data.time_increment, data.point_num});
        packet_states_.push_back(MakePointPacketState(data));
        if (data.point_num == 0)
            return;

        parseFromPacketToPointCloud(cloud_, packet, kMinPointRangeM, 100, kMinPointIntensity);
        points_out.insert(points_out.end(), cloud_.points.begin(), cloud_.points.end());

        timeline_.SetZero(GetGlobalTimeOffsetNs());
        timeline_.PushPointPacket(stamp, PacketDurationNs(data.point_num, data.time_increment), now.mono_ns, now.unix_ns,
                                  imu_out);
    }

    void Flush(std::vector<OutputImuData> &imu_out) { timeline_.Flush(imu_out); }

    // Moves the packet records collected since the last call into `out`.
    void TakePacketLog(std::vector<PacketTiming> &out)
    {
        out.insert(out.end(), log_.begin(), log_.end());
        log_.clear();
    }

    // Moves the point packet states collected since the last call into `out`.
    void TakePacketStates(std::vector<PointPacketState> &out)
    {
        out.insert(out.end(), packet_states_.begin(), packet_states_.end());
        packet_states_.clear();
    }

    const ImuTimeline &Timeline() const { return timeline_; }

private:
    ImuTimeline timeline_;
    PointCloudDLidar cloud_;
    std::vector<PacketTiming> log_;
    std::vector<PointPacketState> packet_states_;
};
