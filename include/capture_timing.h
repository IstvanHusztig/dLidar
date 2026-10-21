#pragma once

#include "unitree_lidar_utilities.h"
#include "time_alignment.h"
#include "packet_state.h"
#include "domain/geometry/axis_map.h"
#include "domain/geometry/point_filter.h"

#include <time.h>
#include <vector>

using namespace unitree_lidar_sdk;

// The L2's default point filter: rejects points inside the housing blind zone
// (spec: 0.05 m, kept a bit past it since accuracy degrades near it: 0.1 m),
// beyond 100 m, and low-reflectivity returns (intensity below 5 of 255), which
// are usually noise or multipath rather than a real surface.
const dlidar::PointFilter kL2DefaultFilter{0.1f, 100.0f, 5.0f};

// How the sensor's data becomes recorded data: the mount and the point filter.
struct CaptureSetup
{
    dlidar::AxisMap mount;                 // default: sensor axes = output axes
    dlidar::PointFilter filter = kL2DefaultFilter;
};

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
// the L2 clock (see ImuTimeline), and keeps the per-packet timing log. Owns
// the recording's time zero: the L2 stamp of the first point packet that has
// points. Applies the mount, the conversion to g and the point filter.
class CaptureTiming
{
public:
    explicit CaptureTiming(const ImuTimeline::Config &config = ImuTimeline::Config(), const CaptureSetup &setup = CaptureSetup())
        : timeline_(config), setup_(setup) {}

    // IMU packet just parsed. Acceleration in g, angular velocity in rad/s.
    void OnImu(const LidarImuData &imu, HostTime now, std::vector<OutputImuData> &imu_out)
    {
        ParsedImuData parsed;
        parseFromImuPacket(parsed, imu);

        // m/s^2 -> g, then the mount (select and negate only, so the order
        // does not change a bit).
        float accel_g[3], accel[3], gyro[3];
        for (int i = 0; i < 3; i++)
            accel_g[i] = parsed.accel[i] / 9.80665f;
        setup_.mount.Apply(accel_g, accel);
        setup_.mount.Apply(parsed.gyro, gyro);

        ImuInput in{};
        in.data.AccelerationX = accel[0];
        in.data.AccelerationY = accel[1];
        in.data.AccelerationZ = accel[2];
        in.data.GyroX = gyro[0];
        in.data.GyroY = gyro[1];
        in.data.GyroZ = gyro[2];
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

        if (zero_ns_ == 0) // set before converting this packet, so its first point is at 0
            zero_ns_ = stamp;
        parseFromPacketToPointCloud(cloud_, packet, zero_ns_, setup_.filter);
        for (PointDLidar p : cloud_.points)
        {
            if (!setup_.filter.Accepts(p.range, p.intensity))
                continue;
            setup_.mount.ApplyInPlace(p.x, p.y, p.z);
            points_out.push_back(p);
        }

        timeline_.SetZero(zero_ns_);
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

    // Appends the packetstate rows of the point packets since the last call to
    // `out`, with stamp_ns relative to `zero_ns`.
    void TakePacketStateRows(int64_t zero_ns, std::string &out)
    {
        out.reserve(out.size() + packet_states_.size() * 320);
        for (const PointPacketState &st : packet_states_)
            AppendPacketStateRow(out, st, zero_ns);
        packet_states_.clear();
    }

    const ImuTimeline &Timeline() const { return timeline_; }

    // The L2 stamp all recorded times are relative to; 0 until the first point
    // packet with points.
    int64_t ZeroNs() const { return zero_ns_; }

private:
    ImuTimeline timeline_;
    CaptureSetup setup_;
    int64_t zero_ns_ = 0;
    PointCloudDLidar cloud_;
    std::vector<PacketTiming> log_;
    std::vector<PointPacketState> packet_states_;
};
