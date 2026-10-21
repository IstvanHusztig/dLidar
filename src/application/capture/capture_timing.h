#pragma once

#include "application/ports/lidar_sensor.h"
#include "domain/geometry/axis_map.h"
#include "domain/geometry/point_filter.h"
#include "domain/recording/imu_sample.h"
#include "domain/recording/packet_timing.h"
#include "domain/recording/point_record.h"
#include "domain/timing/time_alignment.h"

#include <optional>
#include <string>
#include <vector>

namespace dlidar
{
    // How the sensor's data becomes recorded data. The mount belongs to the
    // device; the filter defaults to the sensor's own (SensorInfo).
    struct CaptureSetup
    {
        AxisMap mount; // default: sensor axes = output axes
        std::optional<PointFilter> filter;
    };

    // Turns the packets of a LidarSensor into recorded points and IMU samples,
    // all timed on the sensor clock (see ImuTimeline), and keeps the
    // per-packet timing log. Owns the recording's time zero: the sensor stamp
    // of the first point packet that has points (set before that packet is
    // converted, even if the filter removes all its points). Applies the
    // mount, the conversion to g and the point filter.
    class CaptureTiming
    {
    public:
        CaptureTiming(LidarSensor &sensor, const ImuTimeline::Config &config, const AxisMap &mount,
                      const PointFilter &filter)
            : sensor_(sensor), timeline_(config), mount_(mount), filter_(filter) {}

        // A packet Poll() just returned (kind Imu or Points).
        void OnPacket(const Polled &p, std::vector<PointRecord> &points_out, std::vector<ImuSample> &imu_out)
        {
            if (p.kind == PacketKind::Imu)
                OnImu(p.record, imu_out);
            else if (p.kind == PacketKind::Points)
                OnPoints(p.record, points_out, imu_out);
            else
                return;
            log_.push_back(p.record);
        }

        void Flush(std::vector<ImuSample> &imu_out) { timeline_.Flush(imu_out); }

        // Moves the packet records collected since the last call into `out`.
        void TakePacketLog(std::vector<PacketTiming> &out)
        {
            out.insert(out.end(), log_.begin(), log_.end());
            log_.clear();
        }

        // Appends the sensor's diagnostics rows since the last call, stamps
        // relative to `zero_ns`.
        void TakePacketStateRows(int64_t zero_ns, std::string &out)
        {
            if (PacketDiagnostics *d = sensor_.Diagnostics())
                d->TakeRows(zero_ns, out);
        }

        const ImuTimeline &Timeline() const { return timeline_; }

        // The sensor stamp all recorded times are relative to; 0 until the
        // first point packet with points.
        int64_t ZeroNs() const { return zero_ns_; }

    private:
        LidarSensor &sensor_;
        ImuTimeline timeline_;
        AxisMap mount_;
        PointFilter filter_;
        int64_t zero_ns_ = 0;
        std::vector<ImuReading> readings_;
        std::vector<PointRecord> cloud_;
        std::vector<PacketTiming> log_;

        void OnImu(const PacketTiming &record, std::vector<ImuSample> &imu_out)
        {
            readings_.clear();
            sensor_.TakeImu(readings_);
            for (const ImuReading &r : readings_)
            {
                // m/s^2 -> g, then the mount (select and negate only, so the
                // order does not change a bit).
                float accel_g[3], accel[3], gyro[3];
                for (int i = 0; i < 3; i++)
                    accel_g[i] = r.accel_mps2[i] / 9.80665f;
                mount_.Apply(accel_g, accel);
                mount_.Apply(r.gyro_rads, gyro);

                ImuInput in{};
                in.data.AccelerationX = accel[0];
                in.data.AccelerationY = accel[1];
                in.data.AccelerationZ = accel[2];
                in.data.GyroX = gyro[0];
                in.data.GyroY = gyro[1];
                in.data.GyroZ = gyro[2];
                in.data.ImuId = 0;
                in.sensor_ns = r.stamp_ns;
                in.seq = r.seq;
                in.host_mono_ns = record.host_mono_ns;
                in.host_unix_ns = record.host_unix_ns;
                timeline_.PushImu(in, imu_out);
            }
        }

        void OnPoints(const PacketTiming &record, std::vector<PointRecord> &points_out, std::vector<ImuSample> &imu_out)
        {
            const PointPacketInfo &info = sensor_.Points();
            if (info.count == 0)
                return;

            if (zero_ns_ == 0) // set before converting this packet, so its first point is at 0
                zero_ns_ = info.first_ns;
            cloud_.clear();
            sensor_.ConvertPoints(zero_ns_, filter_, cloud_);
            for (PointRecord p : cloud_)
            {
                if (!filter_.Accepts(p.range, p.intensity))
                    continue;
                mount_.ApplyInPlace(p.x, p.y, p.z);
                points_out.push_back(p);
            }

            timeline_.SetZero(zero_ns_);
            timeline_.PushPointPacket(info.first_ns, info.duration_ns, record.host_mono_ns, record.host_unix_ns, imu_out);
        }
    };
}
