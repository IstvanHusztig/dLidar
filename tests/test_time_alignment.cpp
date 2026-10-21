// IMU / LiDAR time alignment, driven through CaptureTiming (the same code as
// the capture loop) with synthetic L2 packets.
//
// A simulated L2 has its own clock: L2(tau) = l2_0 + tau * (1 + kL2RatePpm),
// the host has CLOCK_MONOTONIC = mono_0 + tau and CLOCK_REALTIME = unix_0 + tau,
// with the L2 clock kL2UnixOffsetNs behind Unix time (like recording 44). The
// IMU samples every kImuPeriodS (true time) and is sent in batches; point
// packets of kPointsPerPacket points are sent when complete. Both arrive with
// a minimum latency plus jitter, host stalls and lost packets.
//
// For every written IMU sample and point the residual is
//   written time - (L2 clock at its true measurement time - zero),
// i.e. both streams are compared with one common clock. Pass: |residual|
// below 1 ms everywhere over 10 minutes, no drift, strictly increasing IMU
// times, timestampUnix - timestamp constant. The old host-stamped method is
// replayed on the same data to show the lag it produced.
#include "capture_timing.h"
#include "packet_replay.h"

#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace
{
int g_failures = 0;
#define CHECK(cond, ...)                                  \
    do                                                    \
    {                                                     \
        if (!(cond))                                      \
        {                                                 \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
            g_failures++;                                 \
        }                                                 \
    } while (0)

const double kImuPeriodS = 1.0 / 889.7;
const double kL2RatePpm = 98.0;               // L2 clock runs fast vs the host (rec 44: 0.098 ms/s)
const int64_t kL2UnixOffsetNs = 78000000;     // L2 clock behind Unix time
const int64_t kL2Zero = 1790000000000000000;  // L2 clock at tau = 0
const int64_t kMono0 = 5000000000000;
const int kPointsPerPacket = 120;
const double kPointDtS = 1.0 / 64000;         // true time between points
const double kMinLatencyS = 0.0010;           // both packet types

enum class ImuStamps
{
    Good,
    Frozen,     // every IMU packet carries the same stamp
    OtherClock, // IMU stamps 5 s off the point stamps
    BatchSend,  // IMU stamps are the batch send time, not the sample time
};

struct Sim
{
    double duration_s = 600;
    ImuStamps stamps = ImuStamps::Good;
    double batch_s = 0.008;           // IMU batch interval
    double stall_every_s = 7.0;       // host stalls
    double stall_s = 0.040;
    double imu_drop_every_s = 3.0;    // lose 5 IMU packets
    int duplicate_every = 0;          // resend an IMU packet (same seq, same stamp)
    uint64_t seed = 1;
};

int64_t L2At(double tau) { return kL2Zero + (int64_t)std::llround(tau * 1e9 * (1.0 + kL2RatePpm * 1e-6)); }

TimeStamp ToStamp(int64_t ns) { return TimeStamp{(uint32_t)(ns / 1000000000LL), (uint32_t)(ns % 1000000000LL)}; }

struct Event
{
    double arrival; // true time
    double sent;
    bool imu;
    int64_t index;  // IMU sample k or point packet p
};

std::vector<Event> Deliver(const Sim &sim)
{
    std::mt19937_64 rng(sim.seed);
    std::exponential_distribution<double> jitter(1.0 / 0.0003);
    std::vector<Event> ev;

    // IMU: sample k at k*T, sent with its batch.
    int64_t n_imu = (int64_t)(sim.duration_s / kImuPeriodS);
    int64_t per_drop = (int64_t)(sim.imu_drop_every_s / kImuPeriodS);
    double batch_jitter = 0;
    int64_t batch = -1;
    int in_batch = 0;
    for (int64_t k = 0; k < n_imu; k++)
    {
        double tau = k * kImuPeriodS;
        int64_t b = (int64_t)std::ceil(tau / sim.batch_s);
        if (b != batch)
        {
            batch = b;
            in_batch = 0;
            batch_jitter = jitter(rng);
        }
        if (per_drop > 0 && k % per_drop >= per_drop - 5)
            continue;
        double sent = b * sim.batch_s;
        ev.push_back({sent + kMinLatencyS + batch_jitter + 15e-6 * in_batch, sent + 1e-9 * in_batch, true, k});
        in_batch++;
        if (sim.duplicate_every && k % sim.duplicate_every == 0)
            ev.push_back({ev.back().arrival + 5e-6, ev.back().sent + 1e-10, true, k});
    }

    // Points: packet p covers [p*D, p*D + (n-1)*dt], sent after its last point.
    double packet_s = kPointsPerPacket * kPointDtS;
    int64_t n_packets = (int64_t)(sim.duration_s / packet_s);
    for (int64_t p = 0; p < n_packets; p++)
    {
        double sent = p * packet_s + (kPointsPerPacket - 1) * kPointDtS;
        ev.push_back({sent + kMinLatencyS + jitter(rng), sent, false, p});
    }

    // One link: packets arrive in the order they were sent.
    std::stable_sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) { return a.sent < b.sent; });
    for (size_t i = 1; i < ev.size(); i++)
        ev[i].arrival = std::max(ev[i].arrival, ev[i - 1].arrival + 2e-6);

    // Host stalls: nothing is read for stall_s, then the backlog at once.
    for (Event &e : ev)
    {
        double phase = std::fmod(e.arrival, sim.stall_every_s);
        if (e.arrival > sim.stall_every_s && phase < sim.stall_s)
            e.arrival += sim.stall_s - phase + 1e-6;
    }
    std::stable_sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) { return a.arrival < b.arrival; });
    return ev;
}

LidarImuData ImuPacket(int64_t k, const Sim &sim)
{
    LidarImuData d{};
    d.info.seq = 1000 + (uint32_t)k;
    int64_t stamp = L2At(k * kImuPeriodS);
    if (sim.stamps == ImuStamps::Frozen)
        stamp = kL2Zero;
    else if (sim.stamps == ImuStamps::OtherClock)
        stamp += 5000000000LL;
    else if (sim.stamps == ImuStamps::BatchSend)
        stamp = L2At(std::ceil(k * kImuPeriodS / sim.batch_s) * sim.batch_s) + 1000 * (k % 16);
    d.info.stamp = ToStamp(stamp);
    d.angular_velocity[0] = (float)k; // identifies the sample in the output
    d.linear_acceleration[2] = 9.80665f;
    return d;
}

LidarPointDataPacket PointPacket(int64_t p)
{
    LidarPointDataPacket pkt{};
    LidarPointData &d = pkt.data;
    d.info.seq = (uint32_t)p;
    d.info.stamp = ToStamp(L2At(p * kPointsPerPacket * kPointDtS));
    d.param.range_scale = 0.001f;
    d.range_min = 0;
    d.range_max = 100;
    d.point_num = kPointsPerPacket;
    d.time_increment = (float)(kPointDtS * (1.0 + kL2RatePpm * 1e-6)); // in L2 seconds
    d.scan_period = d.time_increment * kPointsPerPacket;
    for (int j = 0; j < kPointsPerPacket; j++)
    {
        d.ranges[j] = 2000;
        d.intensities[j] = 100;
    }
    return pkt;
}

struct Residuals
{
    double max_abs_ms = 0;
    double first_min_ms = 0; // mean over the first and last 60 s
    double last_min_ms = 0;
    double sum_first = 0, sum_last = 0;
    uint64_t n_first = 0, n_last = 0, n = 0;
    void Add(double tau, double r_ns, double duration_s)
    {
        double ms = r_ns / 1e6;
        max_abs_ms = std::max(max_abs_ms, std::fabs(ms));
        n++;
        if (tau < 60)
            sum_first += ms, n_first++;
        if (tau > duration_s - 60)
            sum_last += ms, n_last++;
    }
    double First() const { return n_first ? sum_first / n_first : NAN; }
    double Last() const { return n_last ? sum_last / n_last : NAN; }
};

struct Result
{
    Residuals imu, points, old_imu;
    ImuTimeline::Mode mode;
    ImuTimeline::Stats stats;
    ImuTimeline::Clocks clocks;
    std::string report;
    bool increasing = true, epoch_constant = true;
};

Result Run(const Sim &sim, ImuTimeline::Mode mode, std::vector<PacketTiming> *log = nullptr)
{
    ImuTimeline::Config cfg;
    cfg.mode = mode;
    cfg.log = false;
    cfg.host.log = false;
    CaptureTiming timing(cfg);
    ImuTimestamper old_method(cfg.host); // IMU stamped with host Unix arrival, as before
    std::vector<OutputImuData> imu_out, old_out;
    std::vector<PointDLidar> points_out;
    Result res;
    int64_t zero = 0;
    bool have_last = false;
    int64_t last_t = 0, epoch_diff = 0;

    auto check_imu = [&](std::vector<OutputImuData> &out) {
        for (const OutputImuData &s : out)
        {
            double tau = s.GyroX * kImuPeriodS;
            res.imu.Add(tau, (double)s.LidarTimestamp - (double)(L2At(tau) - zero), sim.duration_s);
            if (have_last && s.LidarTimestamp <= last_t)
                res.increasing = false;
            if (have_last && s.EpochTimestamp - s.LidarTimestamp != epoch_diff)
                res.epoch_constant = false;
            have_last = true;
            last_t = s.LidarTimestamp;
            epoch_diff = s.EpochTimestamp - s.LidarTimestamp;
        }
        out.clear();
    };
    auto check_old = [&](std::vector<OutputImuData> &out) {
        for (const OutputImuData &s : out)
        {
            double tau = s.GyroX * kImuPeriodS;
            res.old_imu.Add(tau, (double)s.LidarTimestamp - (double)(L2At(tau) - zero), sim.duration_s);
        }
        out.clear();
    };

    for (const Event &e : Deliver(sim))
    {
        HostTime now{kMono0 + (int64_t)std::llround(e.arrival * 1e9),
                     kL2Zero + kL2UnixOffsetNs + (int64_t)std::llround(e.arrival * 1e9)};
        if (e.imu)
        {
            LidarImuData d = ImuPacket(e.index, sim);
            timing.OnImu(d, now, imu_out);
            if (zero != 0)
            {
                OutputImuData v{};
                v.GyroX = (float)e.index;
                old_method.Push(v, now.unix_ns - zero, d.info.seq, now.unix_ns, old_out);
            }
        }
        else
        {
            size_t before = points_out.size();
            timing.OnPoints(PointPacket(e.index), now, points_out, imu_out);
            zero = timing.ZeroNs();
            double start = e.index * kPointsPerPacket * kPointDtS;
            for (size_t i = before; i < points_out.size(); i++)
            {
                double tau = start + (double)(i - before) * kPointDtS;
                res.points.Add(tau, points_out[i].time * 1e9 - (double)(L2At(tau) - zero), sim.duration_s);
            }
            points_out.clear();
        }
        check_imu(imu_out);
        check_old(old_out);
        if (log)
            timing.TakePacketLog(*log);
    }
    timing.Flush(imu_out);
    old_method.Flush(old_out);
    check_imu(imu_out);
    check_old(old_out);
    res.mode = timing.Timeline().ActiveMode();
    res.stats = timing.Timeline().GetStats();
    res.clocks = timing.Timeline().MeasureClocks();
    res.report = timing.Timeline().Report();
    return res;
}

void Report(const char *name, const Result &r)
{
    printf("  %-34s mode %-12s IMU residual max %.4f ms (first 60 s %+.4f, last 60 s %+.4f), points max %.4f ms;"
           " old method %+.2f -> %+.2f ms\n",
           name, ImuTimeline::ModeName(r.mode), r.imu.max_abs_ms, r.imu.First(), r.imu.Last(), r.points.max_abs_ms,
           r.old_imu.First(), r.old_imu.Last());
}

void ExpectAligned(const char *name, const Result &r, ImuTimeline::Mode mode, double max_ms, double max_drift_ms)
{
    Report(name, r);
    CHECK(r.mode == mode, "%s: mode %s", name, ImuTimeline::ModeName(r.mode));
    CHECK(r.imu.n > 100000, "%s: only %llu IMU samples written", name, (unsigned long long)r.imu.n);
    CHECK(r.imu.max_abs_ms < max_ms, "%s: IMU residual %.4f ms", name, r.imu.max_abs_ms);
    CHECK(r.points.max_abs_ms < 0.001, "%s: point residual %.4f ms", name, r.points.max_abs_ms);
    CHECK(std::fabs(r.imu.Last() - r.imu.First()) < max_drift_ms, "%s: drift %.4f ms", name, r.imu.Last() - r.imu.First());
    CHECK(r.increasing, "%s: IMU timestamps not strictly increasing", name);
    CHECK(r.epoch_constant, "%s: timestampUnix - timestamp not constant", name);
}

// One synthetic packet of each type through the conversion: exact times.
void TestPacketConversion()
{
    printf("packet conversion\n");
    ImuTimeline::Config cfg;
    cfg.mode = ImuTimeline::Mode::Sensor;
    cfg.log = false;
    CaptureTiming timing(cfg);
    std::vector<OutputImuData> imu;
    std::vector<PointDLidar> pts;

    LidarPointDataPacket p = PointPacket(0);
    p.data.info.stamp = TimeStamp{1790000000, 123456789};
    p.data.point_num = 3;
    p.data.time_increment = 1e-5f;
    timing.OnPoints(p, {1, 1}, pts, imu);
    CHECK(timing.ZeroNs() == 1790000000123456789LL, "zero %" PRId64, timing.ZeroNs());
    CHECK(pts.size() == 3, "%zu points", pts.size());
    for (size_t j = 0; j < pts.size(); j++)
        CHECK(std::fabs(pts[j].time - j * (double)1e-5f) < 1e-12, "point %zu time %.12f", j, pts[j].time);

    LidarImuData d{};
    d.info.seq = 7;
    d.info.stamp = TimeStamp{1790000000, 124456789}; // 1 ms after the first point
    d.linear_acceleration[2] = 9.80665f;
    d.angular_velocity[0] = 0.5f;
    timing.OnImu(d, {2, 2}, imu);
    CHECK(imu.size() == 1, "%zu IMU samples", imu.size());
    if (!imu.empty())
    {
        CHECK(imu[0].LidarTimestamp == 1000000, "IMU timestamp %" PRId64, imu[0].LidarTimestamp);
        CHECK(std::fabs(imu[0].AccelerationZ - 1.0f) < 1e-6 && imu[0].GyroX == 0.5f, "IMU values");
    }
    std::vector<PacketTiming> log;
    timing.TakePacketLog(log);
    CHECK(log.size() == 2 && log[0].type == kPacketTypePoints && log[1].type == kPacketTypeImu &&
              log[1].sensor_ns == 1790000000124456789LL && log[0].point_num == 3,
          "packet log");
}
} // namespace

int main()
{
    TestPacketConversion();

    printf("10 min simulated, L2 clock %+.0f ppm vs host, %.0f ms behind Unix time\n", kL2RatePpm, kL2UnixOffsetNs / 1e6);
    Sim good;
    Result r = Run(good, ImuTimeline::Mode::Auto);
    ExpectAligned("good stamps (auto)", r, ImuTimeline::Mode::Sensor, 0.001, 0.001);
    CHECK(r.stats.lost_samples > 0 && r.stats.non_increasing_dropped == 0, "lost %llu, non-increasing %llu",
          (unsigned long long)r.stats.lost_samples, (unsigned long long)r.stats.non_increasing_dropped);
    // The old method reproduces the bug: tens of ms of lag, shrinking.
    CHECK(r.old_imu.First() > 50 && r.old_imu.First() - r.old_imu.Last() > 30, "old method %.2f -> %.2f ms",
          r.old_imu.First(), r.old_imu.Last());
    // The recording's own diagnostics recover the simulated clocks.
    const ImuTimeline::Clocks &c = r.clocks;
    printf("  diagnostics: L2 clock %+.2f ppm, IMU stamps %+.2f ppm, IMU period %.5f ms, IMU - point min latency "
           "%+.3f -> %+.3f ms, old method lag %.2f -> %.2f ms\n",
           c.lidar_rate_ppm, c.imu_rate_ppm, c.imu_period_ns / 1e6, c.min_latency_diff_start_ms, c.min_latency_diff_end_ms,
           c.old_lag_start_ms, c.old_lag_end_ms);
    CHECK(c.valid && std::fabs(c.lidar_rate_ppm - kL2RatePpm) < 2 && std::fabs(c.imu_rate_ppm - kL2RatePpm) < 2,
          "clock rates %.2f / %.2f ppm", c.lidar_rate_ppm, c.imu_rate_ppm);
    CHECK(std::fabs(c.imu_period_ns - kImuPeriodS * 1e9 * (1 + kL2RatePpm * 1e-6)) < 1, "IMU period %.3f ns", c.imu_period_ns);
    CHECK(std::fabs(c.min_latency_diff_start_ms) < 0.5 && std::fabs(c.min_latency_diff_end_ms) < 0.5, "min latency diff");
    double expect_start = (kL2UnixOffsetNs + kMinLatencyS * 1e9) / 1e6;
    CHECK(std::fabs(c.old_lag_start_ms - expect_start) < 1 &&
              std::fabs(c.old_lag_end_ms - (expect_start - kL2RatePpm * 1e-6 * c.span_s * 1e3)) < 1,
          "old lag %.2f -> %.2f ms", c.old_lag_start_ms, c.old_lag_end_ms);
    CHECK(r.report.find("\"imu_time_source\": \"sensor\"") != std::string::npos, "report:\n%s", r.report.c_str());
    CHECK(std::fabs(c.spacing_p1_ms - 1.124) < 0.002 && std::fabs(c.spacing_p99_ms - 1.124) < 0.002,
          "stamp spacing p1 %.4f p50 %.4f p99 %.4f ms", c.spacing_p1_ms, c.spacing_p50_ms, c.spacing_p99_ms);

    Sim dup = good;
    dup.duration_s = 120;
    dup.duplicate_every = 997;
    r = Run(dup, ImuTimeline::Mode::Auto);
    Report("duplicated IMU packets (auto)", r);
    CHECK(r.mode == ImuTimeline::Mode::Sensor && r.stats.duplicates_dropped > 0 && r.increasing && r.imu.max_abs_ms < 0.001,
          "duplicates: mode %s, dropped %llu", ImuTimeline::ModeName(r.mode), (unsigned long long)r.stats.duplicates_dropped);

    r = Run(good, ImuTimeline::Mode::HostAligned);
    ExpectAligned("good stamps, forced host_aligned", r, ImuTimeline::Mode::HostAligned, 1.0, 0.5);

    Sim frozen = good;
    frozen.stamps = ImuStamps::Frozen;
    r = Run(frozen, ImuTimeline::Mode::Auto);
    ExpectAligned("frozen IMU stamps (auto)", r, ImuTimeline::Mode::HostAligned, 1.0, 0.5);

    Sim other = good;
    other.stamps = ImuStamps::OtherClock;
    r = Run(other, ImuTimeline::Mode::Auto);
    ExpectAligned("IMU stamps on another clock (auto)", r, ImuTimeline::Mode::HostAligned, 1.0, 0.5);

    Sim batch = good;
    batch.stamps = ImuStamps::BatchSend;
    r = Run(batch, ImuTimeline::Mode::Auto);
    ExpectAligned("IMU stamps = batch send time (auto)", r, ImuTimeline::Mode::HostAligned, 1.0, 0.5);

    Sim smooth = good;
    smooth.batch_s = 0.0005; // nearly unbatched delivery, no stalls
    smooth.stall_every_s = 1e9;
    r = Run(smooth, ImuTimeline::Mode::Auto);
    ExpectAligned("smooth delivery (auto)", r, ImuTimeline::Mode::Sensor, 0.001, 0.001);

    // The packet log of a recording replays to the same numbers.
    {
        Sim replay = good;
        replay.duration_s = 180;
        std::vector<PacketTiming> log;
        Result live = Run(replay, ImuTimeline::Mode::Auto, &log);
        std::string path = "/tmp/dlidar_test_packets_" + std::to_string(getpid()) + ".bin";
        {
            std::ofstream f(path, std::ios::binary);
            f.write(kPacketLogMagic, sizeof kPacketLogMagic);
            f.write(reinterpret_cast<const char *>(log.data()), log.size() * sizeof(PacketTiming));
        }
        std::vector<PacketTiming> read;
        CHECK(ReadPacketLog(path, read) && read.size() == log.size(), "packet log read back %zu of %zu", read.size(), log.size());
        std::remove(path.c_str());
        ReplayResult rr = ReplayPackets(read);
        printf("  replay of a 180 s packet log:\n");
        for (const ReplaySegment &seg : rr.segments)
            printf("    %4.0f s: host_aligned - sensor %+.3f ms, old method - sensor %+.2f ms\n", seg.start_s,
                   seg.host_minus_sensor_ms, seg.old_minus_sensor_ms);
        CHECK(rr.auto_mode == ImuTimeline::Mode::Sensor && rr.segments.size() >= 3, "replay mode / segments");
        for (const ReplaySegment &seg : rr.segments)
        {
            CHECK(std::fabs(seg.host_minus_sensor_ms) < 1, "replay host_aligned - sensor %.3f ms", seg.host_minus_sensor_ms);
            CHECK(seg.old_minus_sensor_ms > 50, "replay old method - sensor %.3f ms", seg.old_minus_sensor_ms);
        }
        CHECK(rr.report == live.report, "replayed report differs from the live one");
    }

    printf(g_failures ? "time alignment: %d checks FAILED\n" : "time alignment: all checks passed\n", g_failures);
    return g_failures ? 1 : 0;
}
