#pragma once

#include "domain/recording/imu_sample.h"
#include "domain/recording/packet_timing.h"
#include "domain/timing/imu_timestamper.h"
#include "domain/timing/time_alignment.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace dlidar
{
// Replays the packet timing log of a recording (packetsNNNN.bin) through the
// IMU timestamping: ImuTimeline in auto, sensor and host_aligned mode, and the
// old method (ImuTimestamper on the host Unix arrival time, as before the
// fix). Prints the auto-mode report and, per 60 s segment, the mean
// difference of each method from the L2 IMU stamps (the sensor mode).
// Only the timing is logged, so IMU values are zero.


struct ReplaySegment
{
    double start_s = 0;
    uint64_t n = 0;
    double host_minus_sensor_ms = 0; // mean, host_aligned - sensor
    double old_minus_sensor_ms = 0;  // mean, old method - sensor
    uint64_t n_host = 0, n_old = 0;
};

struct ReplayResult
{
    ImuTimeline::Mode auto_mode = ImuTimeline::Mode::Auto;
    std::string report; // auto mode
    std::vector<ReplaySegment> segments;
};

inline ReplayResult ReplayPackets(const std::vector<PacketTiming> &packets, int64_t imu_offset_ns = 0)
{
    ImuTimeline::Config cfg;
    cfg.imu_offset_ns = imu_offset_ns;
    cfg.log = false;
    cfg.host.log = false;
    ImuTimeline::Config sensor_cfg = cfg, host_cfg = cfg;
    sensor_cfg.mode = ImuTimeline::Mode::Sensor;
    host_cfg.mode = ImuTimeline::Mode::HostAligned;
    ImuTimeline automatic(cfg), sensor(sensor_cfg), host(host_cfg);
    ImuTimestamper old_method(cfg.host);

    // Written times by packet seq (the replay has no other sample identity).
    std::map<uint32_t, int64_t> t_sensor, t_host, t_old;
    std::vector<ImuSample> out;
    auto collect = [&](std::map<uint32_t, int64_t> &into) {
        for (const ImuSample &s : out)
            into[(uint32_t)s.ImuId] = s.LidarTimestamp;
        out.clear();
    };

    bool zero_set = false;
    int64_t zero = 0;
    for (const PacketTiming &p : packets)
    {
        if (p.type == kPacketTypePoints)
        {
            if (p.point_num == 0)
                continue;
            if (!zero_set)
            {
                zero = p.sensor_ns;
                zero_set = true;
                automatic.SetZero(zero);
                sensor.SetZero(zero);
                host.SetZero(zero);
            }
            int64_t d = PacketDurationNs(p.point_num, p.time_increment);
            automatic.PushPointPacket(p.sensor_ns, d, p.host_mono_ns, p.host_unix_ns, out);
            out.clear();
            sensor.PushPointPacket(p.sensor_ns, d, p.host_mono_ns, p.host_unix_ns, out);
            collect(t_sensor);
            host.PushPointPacket(p.sensor_ns, d, p.host_mono_ns, p.host_unix_ns, out);
            collect(t_host);
        }
        else if (p.type == kPacketTypeImu)
        {
            ImuInput in{};
            in.data.ImuId = (int)p.seq; // carried through as the sample's identity
            in.sensor_ns = p.sensor_ns;
            in.seq = p.seq;
            in.host_mono_ns = p.host_mono_ns;
            in.host_unix_ns = p.host_unix_ns;
            automatic.PushImu(in, out);
            out.clear();
            sensor.PushImu(in, out);
            collect(t_sensor);
            host.PushImu(in, out);
            collect(t_host);
            if (zero_set)
            {
                old_method.Push(in.data, p.host_unix_ns - zero, p.seq, p.host_unix_ns, out);
                collect(t_old);
            }
        }
    }
    automatic.Flush(out);
    out.clear();
    sensor.Flush(out);
    collect(t_sensor);
    host.Flush(out);
    collect(t_host);
    old_method.Flush(out);
    collect(t_old);

    ReplayResult r;
    r.auto_mode = automatic.ActiveMode();
    r.report = automatic.Report();
    for (const auto &kv : t_sensor)
    {
        size_t seg = (size_t)std::max<int64_t>(0, kv.second / 60000000000LL);
        if (r.segments.size() <= seg)
            r.segments.resize(seg + 1);
        ReplaySegment &s = r.segments[seg];
        s.start_s = 60.0 * (double)seg;
        s.n++;
        auto h = t_host.find(kv.first);
        if (h != t_host.end())
        {
            s.host_minus_sensor_ms += (double)(h->second - kv.second) / 1e6;
            s.n_host++;
        }
        auto o = t_old.find(kv.first);
        if (o != t_old.end())
        {
            s.old_minus_sensor_ms += (double)(o->second - kv.second) / 1e6;
            s.n_old++;
        }
    }
    for (ReplaySegment &s : r.segments)
    {
        if (s.n_host)
            s.host_minus_sensor_ms /= (double)s.n_host;
        if (s.n_old)
            s.old_minus_sensor_ms /= (double)s.n_old;
    }
    return r;
}

inline void PrintReplay(const ReplayResult &r, std::ostream &os)
{
    os << r.report;
    os << "\nPer 60 s segment, mean IMU time difference from the L2 IMU stamps (sensor mode):\n"
       << "  start_s  samples  host_aligned-sensor_ms  old_method-sensor_ms\n";
    for (const ReplaySegment &s : r.segments)
    {
        char line[160];
        snprintf(line, sizeof line, "  %7.0f  %7llu  %22.3f  %20.3f\n", s.start_s, (unsigned long long)s.n,
                 s.n_host ? s.host_minus_sensor_ms : NAN, s.n_old ? s.old_minus_sensor_ms : NAN);
        os << line;
    }
}

} // namespace dlidar
