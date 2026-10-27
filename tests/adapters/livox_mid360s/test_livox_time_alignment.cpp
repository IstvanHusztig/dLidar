// IMU / LiDAR time alignment for the Mid-360S, driven through its adapter
// (LivoxMid360sSensor) and CaptureTiming (the same code as the capture loop)
// with packets built byte for byte as the sensor sends them.
//
// A simulated Mid-360S has its own clock, counting from power-on:
// S(tau) = s_0 + tau * (1 + kRatePpm). Its IMU samples every 5 ms (200 Hz)
// and sends each sample in its own packet (udp_cnt counts them and wraps at
// 65536 within the 10 minutes); point packets of 96 points (200,000 points/s)
// are sent when complete. Both arrive with a minimum latency plus jitter,
// host stalls and lost packets of both kinds.
//
// For every written IMU sample and point the residual is
//   written time - (sensor clock at its true measurement time - zero).
// Pass: auto mode keeps the sensor stamps, |residual| below 1 us, no drift,
// strictly increasing IMU times, timestampUnix - timestamp constant, lost
// packets visible as seq gaps. IMU stamps on another clock fall back to
// host_aligned within 1 ms.
#include "adapters/livox_mid360s/livox_mid360s_sensor.h"
#include "adapters/livox_mid360s/livox_packet_builder.h"
#include "application/capture/capture_timing.h"
#include "scripted_livox_source.h"
#include "../../check.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

using namespace dlidar;
using namespace dlidar::livox;

namespace
{
const double kImuPeriodS = 0.005;
const double kRatePpm = -42.0;               // the sensor clock runs slow vs the host
const int64_t kS0 = 123456789000LL;          // sensor clock at tau = 0: ~2 min after power-on
const int64_t kMono0 = 5000000000000LL;
const int64_t kUnix0 = 1790000000000000000LL;
const int kPoints = 96;
const double kPointDtS = 1.0 / 200000;       // true time between points
const double kMinLatencyS = 0.0008;          // both packet types
const int kPacketsPerFrame = 208;            // 10 Hz frames: udp_cnt restarts

int64_t SensorAt(double tau) { return kS0 + (int64_t)std::llround(tau * 1e9 * (1.0 + kRatePpm * 1e-6)); }

struct Sim
{
    double duration_s = 600;
    bool imu_other_clock = false;  // IMU stamps 5 s off the point stamps
    double stall_every_s = 7.0;    // host stalls
    double stall_s = 0.030;
    double imu_drop_every_s = 3.0; // lose 3 IMU packets
    double point_drop_every_s = 5.0; // lose 2 point packets
    uint64_t seed = 1;
};

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
    std::exponential_distribution<double> jitter(1.0 / 0.0002);
    std::vector<Event> ev;
    int64_t n_imu = (int64_t)(sim.duration_s / kImuPeriodS);
    int64_t imu_drop = (int64_t)(sim.imu_drop_every_s / kImuPeriodS);
    for (int64_t k = 0; k < n_imu; k++)
    {
        if (k > 0 && k % imu_drop >= imu_drop - 3)
            continue;
        double sent = k * kImuPeriodS + 20e-6;
        ev.push_back({sent + kMinLatencyS + jitter(rng), sent, true, k});
    }
    double packet_s = kPoints * kPointDtS;
    int64_t n_packets = (int64_t)(sim.duration_s / packet_s);
    int64_t point_drop = (int64_t)(sim.point_drop_every_s / packet_s);
    for (int64_t p = 0; p < n_packets; p++)
    {
        if (p > 0 && p % point_drop >= point_drop - 2)
            continue;
        double sent = p * packet_s + (kPoints - 1) * kPointDtS;
        ev.push_back({sent + kMinLatencyS + jitter(rng), sent, false, p});
    }
    // One link: packets arrive in the order they were sent.
    std::stable_sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) { return a.sent < b.sent; });
    for (size_t i = 1; i < ev.size(); i++)
        ev[i].arrival = std::max(ev[i].arrival, ev[i - 1].arrival + 1e-6);
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

std::vector<uint8_t> ImuPacket(int64_t k, const Sim &sim)
{
    int64_t stamp = SensorAt(k * kImuPeriodS);
    if (sim.imu_other_clock)
        stamp += 5000000000LL;
    // gyro_x identifies the sample in the output.
    return MakeImuPacket(stamp, (uint16_t)k, LivoxLidarImuRawPoint{(float)k, 0, 0, 0, 0, 1.0f});
}

std::vector<uint8_t> PointPacket(int64_t p)
{
    static const std::vector<LivoxLidarCartesianHighRawPoint> pts(kPoints, LivoxLidarCartesianHighRawPoint{2000, 500, 300, 50, 0});
    // The sensor's own view of the packet: its time_interval on its clock.
    uint16_t interval = (uint16_t)std::lround((kPoints - 1) * kPointDtS * (1.0 + kRatePpm * 1e-6) * 1e7);
    return MakePointPacket(SensorAt(p * kPoints * kPointDtS), (uint8_t)(p / kPacketsPerFrame),
                           (uint16_t)(p % kPacketsPerFrame), interval, pts);
}

struct Residuals
{
    double max_abs_ms = 0;
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
    Residuals imu, points;
    ImuTimeline::Mode mode;
    ImuTimeline::Stats stats;
    ImuTimeline::Clocks clocks;
    std::string report;
    bool increasing = true, epoch_constant = true;
    uint64_t point_seq_gaps = 0; // lost point packets seen in the packet log
};

Result Run(const Sim &sim, ImuTimeline::Mode mode)
{
    ImuTimeline::Config cfg;
    cfg.mode = mode;
    cfg.log = false;
    cfg.host.log = false;
    auto owned = std::make_unique<test::ScriptedLivoxSource>();
    test::ScriptedLivoxSource &source = *owned;
    LivoxMid360sSensor sensor(std::move(owned));
    CaptureTiming timing(sensor, cfg, AxisMap(), kDefaultFilter);
    std::vector<ImuSample> imu_out;
    std::vector<PointRecord> points_out;
    std::vector<PacketTiming> log;
    Result res;
    int64_t zero = 0, last_t = 0, epoch_diff = 0;
    bool have_last = false;
    int64_t last_packet = -1;

    auto check_imu = [&]() {
        for (const ImuSample &s : imu_out)
        {
            double tau = s.GyroX * kImuPeriodS;
            res.imu.Add(tau, (double)s.LidarTimestamp - (double)(SensorAt(tau) - zero), sim.duration_s);
            if (have_last && s.LidarTimestamp <= last_t)
                res.increasing = false;
            if (have_last && s.EpochTimestamp - s.LidarTimestamp != epoch_diff)
                res.epoch_constant = false;
            have_last = true;
            last_t = s.LidarTimestamp;
            epoch_diff = s.EpochTimestamp - s.LidarTimestamp;
        }
        imu_out.clear();
    };

    for (const Event &e : Deliver(sim))
    {
        HostTime now{kMono0 + (int64_t)std::llround(e.arrival * 1e9), kUnix0 + (int64_t)std::llround(e.arrival * 1e9)};
        source.Add(e.imu ? ImuPacket(e.index, sim) : PointPacket(e.index), now);
        Polled p = sensor.Poll();
        timing.OnPacket(p, points_out, imu_out);
        if (!e.imu)
        {
            zero = timing.ZeroNs();
            double start = e.index * kPoints * kPointDtS;
            CHECK(points_out.size() == (size_t)kPoints, "packet %lld: %zu points", (long long)e.index, points_out.size());
            for (const PointRecord &pt : points_out)
            {
                double tau = start + pt.sample_index * kPointDtS;
                res.points.Add(tau, pt.time * 1e9 - (double)(SensorAt(tau) - zero), sim.duration_s);
            }
            points_out.clear();
        }
        check_imu();
        timing.TakePacketLog(log);
        for (const PacketTiming &t : log)
            if (t.type == kPacketTypePoints)
            {
                // The packet index from (frame_cnt, udp_cnt) as the log records it.
                int64_t index = (int64_t)(t.seq >> 16) * kPacketsPerFrame + (t.seq & 0xffff);
                if (last_packet >= 0 && (index - last_packet) % (256 * kPacketsPerFrame) > 1)
                    res.point_seq_gaps += (uint64_t)((index - last_packet) % (256 * kPacketsPerFrame) - 1);
                last_packet = index;
            }
        log.clear();
    }
    timing.Flush(imu_out);
    check_imu();
    res.mode = timing.Timeline().ActiveMode();
    res.stats = timing.Timeline().GetStats();
    res.clocks = timing.Timeline().MeasureClocks();
    res.report = timing.Timeline().Report();
    return res;
}

void Report(const char *name, const Result &r)
{
    printf("  %-38s mode %-12s IMU residual max %.4f ms (first 60 s %+.4f, last 60 s %+.4f), points max %.4f ms\n", name,
           ImuTimeline::ModeName(r.mode), r.imu.max_abs_ms, r.imu.First(), r.imu.Last(), r.points.max_abs_ms);
}

void ExpectAligned(const char *name, const Result &r, ImuTimeline::Mode mode, double max_ms, double max_drift_ms,
                   uint64_t min_imu)
{
    Report(name, r);
    CHECK(r.mode == mode, "%s: mode %s", name, ImuTimeline::ModeName(r.mode));
    CHECK(r.imu.n > min_imu, "%s: only %llu IMU samples written", name, (unsigned long long)r.imu.n);
    CHECK(r.imu.max_abs_ms < max_ms, "%s: IMU residual %.4f ms", name, r.imu.max_abs_ms);
    CHECK(r.points.max_abs_ms < 0.001, "%s: point residual %.4f ms", name, r.points.max_abs_ms);
    CHECK(std::fabs(r.imu.Last() - r.imu.First()) < max_drift_ms, "%s: drift %.4f ms", name, r.imu.Last() - r.imu.First());
    CHECK(r.increasing, "%s: IMU timestamps not strictly increasing", name);
    CHECK(r.epoch_constant, "%s: timestampUnix - timestamp not constant", name);
}
} // namespace

int main()
{
    printf("10 min simulated Mid-360S, clock %+.0f ppm vs host, 200 Hz IMU, 200,000 points/s\n", kRatePpm);
    Sim good;
    Result r = Run(good, ImuTimeline::Mode::Auto);
    ExpectAligned("good stamps (auto)", r, ImuTimeline::Mode::Sensor, 0.001, 0.001, 110000);
    const ImuTimeline::Clocks &c = r.clocks;
    printf("  diagnostics: sensor clock %+.2f ppm, IMU stamps %+.2f ppm, IMU period %.5f ms, lost IMU samples %llu,"
           " lost point packets %llu\n",
           c.lidar_rate_ppm, c.imu_rate_ppm, c.imu_period_ns / 1e6, (unsigned long long)r.stats.lost_samples,
           (unsigned long long)r.point_seq_gaps);
    CHECK(c.valid && std::fabs(c.lidar_rate_ppm - kRatePpm) < 2 && std::fabs(c.imu_rate_ppm - kRatePpm) < 2,
          "clock rates %.2f / %.2f ppm", c.lidar_rate_ppm, c.imu_rate_ppm);
    CHECK(std::fabs(c.imu_period_ns - kImuPeriodS * 1e9 * (1 + kRatePpm * 1e-6)) < 1, "IMU period %.3f ns", c.imu_period_ns);
    CHECK(r.report.find("\"imu_time_source\": \"sensor\"") != std::string::npos, "report:\n%s", r.report.c_str());
    // 3 IMU packets lost every 3 s, 2 point packets every 5 s: the IMU seq
    // (udp_cnt widened across its wrap) and the point (frame, udp_cnt) show them.
    uint64_t imu_lost = (uint64_t)(good.duration_s / good.imu_drop_every_s) * 3;
    CHECK(r.stats.lost_samples + 3 >= imu_lost && r.stats.lost_samples <= imu_lost, "lost IMU samples %llu, expected ~%llu",
          (unsigned long long)r.stats.lost_samples, (unsigned long long)imu_lost);
    uint64_t pts_lost = (uint64_t)(good.duration_s / good.point_drop_every_s) * 2;
    CHECK(r.point_seq_gaps + 2 >= pts_lost && r.point_seq_gaps <= pts_lost, "lost point packets %llu, expected ~%llu",
          (unsigned long long)r.point_seq_gaps, (unsigned long long)pts_lost);

    Sim other = good;
    other.duration_s = 180;
    other.imu_other_clock = true;
    r = Run(other, ImuTimeline::Mode::Auto);
    ExpectAligned("IMU stamps on another clock (auto)", r, ImuTimeline::Mode::HostAligned, 1.0, 0.5, 30000);

    Sim forced = good;
    forced.duration_s = 180;
    r = Run(forced, ImuTimeline::Mode::HostAligned);
    ExpectAligned("good stamps, forced host_aligned", r, ImuTimeline::Mode::HostAligned, 1.0, 0.5, 30000);

    return FinishTests("Livox time alignment");
}
