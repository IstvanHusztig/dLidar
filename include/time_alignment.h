#pragma once

#include "imu_timestamper.h"

#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// Puts the IMU times on the clock of the point times.
//
// Every L2 packet carries the L2's own stamp (DataInfo.stamp): for a point
// packet the time of its first point (point j adds j * time_increment), for
// an IMU packet the time of its sample. The points always use it. The IMU
// used to get the host arrival time instead (SDK use_system_timestamp), a
// different clock: written IMU times lagged the points by the IMU's minimum
// delivery latency plus the L2-to-host clock offset, and drifted with the
// clocks' rate difference.
//
// ImuTimeline writes every IMU time on the L2 clock, relative to `zero` (the
// L2 stamp of the first point packet, GetGlobalTimeOffsetNs()):
// - Sensor: t = IMU stamp - zero + imu_offset_ns. Nothing is rebuilt,
//   resampled or interpolated; a sample whose stamp is not after the previous
//   one is dropped.
// - HostAligned (fallback when the IMU stamps fail the startup check):
//   ImuTimestamper rebuilds the IMU clock from host arrivals (lower edge), and
//   the lower edge of the point packets' (L2 end-of-packet time, host arrival)
//   maps it onto the L2 clock, so the IMU period is measured against the L2
//   clock and does not drift. This assumes both packet types have the same
//   minimum delivery latency.
// - Auto (default): holds the first validate_ns of IMU samples, then checks
//   that the IMU stamps increase, are evenly spaced (sample times, not batch
//   send times), run at the host clock's rate, and are on the points' clock
//   (minimum arrival latencies within same_clock_tolerance_ns); Sensor if
//   they pass, else HostAligned.
//
// EpochTimestamp = LidarTimestamp + one constant per recording: zero plus the
// L2-to-Unix offset at the lower edge of the point packets seen so far.
// Report() describes the method and the measured clock relations; it is
// written to the recording as timingNNNN.json.

#include "domain/recording/packet_timing.h"

using dlidar::kPacketLogMagic;
using dlidar::kPacketTypeImu;
using dlidar::kPacketTypePoints;
using dlidar::PacketDurationNs;
using dlidar::PacketTiming;

// Lower convex hull of (x, y) added in increasing x (others are skipped).
// Fit() gives the hull edge spanning the mean x: the line under all points
// that is closest to them on average (the LP clock-skew estimator).
// Coordinates are kept relative to the first point, so ns stay exact.
class LowerEdge
{
public:
    struct Line
    {
        int64_t x0 = 0, y0 = 0;
        double slope = 1, b = 0;
        double At(int64_t x) const { return b + slope * (double)(x - x0); } // y - y0
        int64_t AtNs(int64_t x) const { return y0 + (int64_t)std::llround(At(x)); }
        int64_t InverseNs(int64_t y) const { return x0 + (int64_t)std::llround(((double)(y - y0) - b) / slope); }
    };

    void Add(int64_t x, int64_t y)
    {
        if (n_ == 0)
        {
            x0_ = x;
            y0_ = y;
        }
        double px = (double)(x - x0_), py = (double)(y - y0_);
        if (n_ > 0 && px <= last_x_)
            return;
        while (hull_.size() >= 2)
        {
            const P &a = hull_[hull_.size() - 2];
            const P &c = hull_.back();
            if ((c.x - a.x) * (py - a.y) - (c.y - a.y) * (px - a.x) > 0)
                break;
            hull_.pop_back();
        }
        hull_.push_back({px, py});
        n_++;
        sum_x_ += px;
        sum_y_ += py;
        last_x_ = px;
    }

    bool Fit(Line &line) const
    {
        if (hull_.size() < 2)
            return false;
        double mean_x = sum_x_ / (double)n_;
        size_t i = 0;
        while (i + 2 < hull_.size() && hull_[i + 1].x < mean_x)
            i++;
        double slope = (hull_[i + 1].y - hull_[i].y) / (hull_[i + 1].x - hull_[i].x);
        if (!(slope > 0))
            return false;
        line = {x0_, y0_, slope, hull_[i].y - slope * hull_[i].x};
        return true;
    }

    // Mean of y - line(x) over all points (line from Fit()).
    double MeanExcess(const Line &line) const { return n_ ? sum_y_ / (double)n_ - line.At(x0_ + (int64_t)(sum_x_ / (double)n_)) : 0; }
    uint64_t Count() const { return n_; }
    int64_t FirstX() const { return x0_; }
    int64_t LastX() const { return x0_ + (int64_t)last_x_; }

private:
    struct P
    {
        double x, y;
    };
    std::vector<P> hull_;
    uint64_t n_ = 0;
    int64_t x0_ = 0, y0_ = 0;
    double sum_x_ = 0, sum_y_ = 0, last_x_ = 0;
};

struct ImuInput
{
    OutputImuData data;   // values; timestamps are filled in by ImuTimeline
    int64_t sensor_ns;    // the sample's L2 stamp, absolute
    uint32_t seq;         // packet seq
    int64_t host_mono_ns; // arrival, CLOCK_MONOTONIC
    int64_t host_unix_ns; // arrival, CLOCK_REALTIME
};

class ImuTimeline
{
public:
    enum class Mode
    {
        Auto,
        Sensor,
        HostAligned
    };

    static const char *ModeName(Mode m)
    {
        return m == Mode::Sensor ? "sensor" : m == Mode::HostAligned ? "host_aligned" : "auto";
    }

    struct Config
    {
        Mode mode = Mode::Auto;
        int64_t imu_offset_ns = 0;                 // added to every IMU time (field-check correction)
        int64_t validate_ns = 2000000000;          // Auto: IMU data held before deciding
        uint64_t min_point_packets = 50;           // Auto: ... and point packets seen
        double min_increasing = 0.99;              // share of IMU stamps after the previous one
        double min_regular = 0.95;                 // share of stamp steps within 0.5..1.5 x the median step
        double max_rate_error = 0.01;              // |IMU stamp rate / host rate - 1|
        int64_t same_clock_tolerance_ns = 100000000; // |IMU min latency - point min latency|
        int64_t map_window_ns = 30000000000;       // HostAligned: L2 clock map fit window
        int64_t map_refit_ns = 500000000;          // ... refit interval
        int64_t stamp_jump_ns = 50000000;          // Sensor: larger stamp steps than seq explains are reported
        ImuTimestamper::Config host;               // HostAligned: IMU clock rebuild
        bool log = true;
    };

    struct Stats
    {
        uint64_t imu_in = 0;
        uint64_t imu_out = 0;
        uint64_t point_packets = 0;
        uint64_t non_increasing_dropped = 0; // Sensor: stamp not after the previous one
        uint64_t negative_dropped = 0;       // acquired before the zero point
        uint64_t duplicates_dropped = 0;     // Sensor: repeated seq
        uint64_t lost_samples = 0;           // Sensor: seq gaps (if seq counts IMU packets)
        uint64_t stamp_jumps = 0;            // Sensor: stamp jumped further than seq explains
        uint64_t nudged = 0;                 // HostAligned: moved forward to stay increasing
        uint64_t unmapped_dropped = 0;       // HostAligned: no L2 clock map at the end
    };

    ImuTimeline() : ImuTimeline(Config()) {}
    explicit ImuTimeline(const Config &config) : cfg_(config), host_(config.host), mode_(config.mode)
    {
        if (mode_ != Mode::Auto)
        {
            decided_ = true;
            reason_ = std::string("forced ") + ModeName(mode_);
        }
    }

    // L2 stamp of the first point packet: zero of all written times.
    void SetZero(int64_t zero_ns)
    {
        if (!zero_set_)
        {
            zero_ = zero_ns;
            zero_set_ = true;
        }
    }

    void PushPointPacket(int64_t sensor_start_ns, int64_t duration_ns, int64_t host_mono_ns, int64_t host_unix_ns,
                         std::vector<OutputImuData> &out)
    {
        NoteHost(host_mono_ns);
        stats_.point_packets++;
        int64_t end = sensor_start_ns + duration_ns;
        pts_edge_.Add(end, host_mono_ns);
        pts_min_latency_ = std::min(pts_min_latency_, host_mono_ns - end);
        pts_unix_offset_ = std::min(pts_unix_offset_, host_unix_ns - end);

        if (zero_set_)
        {
            int64_t y = host_mono_ns - host_zero_;
            map_pts_.push_back({end - zero_, y});
            while (map_pts_.size() > 2 && y - map_pts_.front().second > cfg_.map_window_ns)
                map_pts_.pop_front();
            if (!map_fitted_ || y - map_fit_y_ >= cfg_.map_refit_ns)
            {
                LowerEdge e;
                for (const auto &p : map_pts_)
                    e.Add(p.first, p.second);
                if (e.Fit(map_))
                {
                    map_fitted_ = true;
                    map_fit_y_ = y;
                }
            }
        }

        if (!decided_)
            TryDecide(out, false);
        else if (mode_ == Mode::Sensor)
            DrainRaw(out);
        else
            EmitHost(out);
    }

    void PushImu(const ImuInput &in, std::vector<OutputImuData> &out)
    {
        NoteHost(in.host_mono_ns);
        stats_.imu_in++;
        imu_edge_.Add(in.sensor_ns, in.host_mono_ns);
        imu_unix_edge_.Add(in.sensor_ns, in.host_unix_ns);
        if (imu_n_ > 0 && in.seq - imu_last_.seq == 1 && in.sensor_ns > imu_last_.sensor_ns)
            spacing_hist_[(size_t)std::min<int64_t>((in.sensor_ns - imu_last_.sensor_ns) / kSpacingBinNs,
                                                    (int64_t)spacing_hist_.size() - 1)]++;
        if (imu_n_ == 0)
            imu_first_ = in;
        imu_last_ = in;
        imu_n_++;

        if (!decided_ || mode_ == Mode::HostAligned)
            host_.Push(in.data, in.host_mono_ns - host_zero_, in.seq, in.host_unix_ns, host_out_);
        if (!decided_ || mode_ == Mode::Sensor)
            raw_.push_back(in);

        if (!decided_)
            TryDecide(out, false);
        else if (mode_ == Mode::Sensor)
            DrainRaw(out);
        else
            EmitHost(out);
    }

    // End of the capture: decide if still undecided and write what is held.
    void Flush(std::vector<OutputImuData> &out)
    {
        if (!decided_)
            TryDecide(out, true);
        if (mode_ == Mode::Sensor)
        {
            DrainRaw(out);
            return;
        }
        host_.Flush(host_out_);
        EmitHost(out);
        if (!host_out_.empty())
        {
            stats_.unmapped_dropped += host_out_.size();
            host_out_.clear();
        }
    }

    Mode ActiveMode() const { return mode_; }
    bool Decided() const { return decided_; }
    const Stats &GetStats() const { return stats_; }
    const std::string &Decision() const { return reason_; }

    // Measured clock relations, in one place for the report and the console.
    struct Clocks
    {
        bool valid = false;
        double lidar_rate_ppm = 0;      // L2 clock (points) vs host monotonic
        double imu_rate_ppm = 0;        // IMU stamps vs host monotonic
        double imu_period_ns = 0;       // IMU sample spacing on the L2 clock
        double min_latency_diff_start_ms = 0; // IMU - point minimum latency (same clock: small, constant)
        double min_latency_diff_end_ms = 0;
        double imu_extra_latency_ms = 0;   // mean arrival delay above the lower edge
        double lidar_extra_latency_ms = 0;
        double old_lag_start_ms = 0;    // IMU lag under the old host stamps
        double old_lag_end_ms = 0;
        double span_s = 0;
        double spacing_p1_ms = NAN; // IMU stamp steps of consecutive seq, on the L2 clock
        double spacing_p50_ms = NAN;
        double spacing_p99_ms = NAN;
    };

    Clocks MeasureClocks() const
    {
        Clocks c;
        LowerEdge::Line li, lp, lu;
        if (!imu_edge_.Fit(li) || !pts_edge_.Fit(lp) || !imu_unix_edge_.Fit(lu))
            return c;
        c.valid = true;
        c.lidar_rate_ppm = (1.0 / lp.slope - 1.0) * 1e6;
        c.imu_rate_ppm = (1.0 / li.slope - 1.0) * 1e6;
        if (imu_n_ > 1)
        {
            uint32_t dseq = imu_last_.seq - imu_first_.seq;
            uint64_t steps = (seq_ones_ * 10 >= seq_steps_ * 9 && dseq > 0) ? dseq : imu_n_ - 1;
            c.imu_period_ns = (double)(imu_last_.sensor_ns - imu_first_.sensor_ns) / (double)steps;
        }
        int64_t x0 = std::max(imu_edge_.FirstX(), pts_edge_.FirstX());
        int64_t x1 = std::min(imu_edge_.LastX(), pts_edge_.LastX());
        if (x1 < x0)
            x1 = x0;
        c.span_s = (double)(x1 - x0) / 1e9;
        c.min_latency_diff_start_ms = (double)(li.AtNs(x0) - lp.AtNs(x0)) / 1e6;
        c.min_latency_diff_end_ms = (double)(li.AtNs(x1) - lp.AtNs(x1)) / 1e6;
        c.imu_extra_latency_ms = imu_edge_.MeanExcess(li) / 1e6;
        c.lidar_extra_latency_ms = pts_edge_.MeanExcess(lp) / 1e6;
        c.old_lag_start_ms = (double)(lu.AtNs(x0) - x0) / 1e6;
        c.old_lag_end_ms = (double)(lu.AtNs(x1) - x1) / 1e6;
        uint64_t total = 0;
        for (uint32_t v : spacing_hist_)
            total += v;
        if (total > 0)
        {
            double *targets[3] = {&c.spacing_p1_ms, &c.spacing_p50_ms, &c.spacing_p99_ms};
            const double q[3] = {0.01, 0.5, 0.99};
            uint64_t acc = 0;
            size_t t = 0;
            for (size_t i = 0; i < spacing_hist_.size() && t < 3; i++)
            {
                acc += spacing_hist_[i];
                while (t < 3 && (double)acc >= q[t] * (double)total)
                    *targets[t++] = (double)i * kSpacingBinNs / 1e6;
            }
        }
        return c;
    }

    // JSON for timingNNNN.json.
    std::string Report() const
    {
        Clocks c = MeasureClocks();
        std::string j = "{\n";
        auto str = [&](const char *k, const std::string &v, bool last = false) {
            std::string e;
            for (char ch : v)
                e += (ch == '"' || ch == '\\') ? std::string("\\") + ch : std::string(1, ch);
            j += std::string("  \"") + k + "\": \"" + e + "\"" + (last ? "\n" : ",\n");
        };
        auto num = [&](const char *k, double v) {
            char b[64];
            snprintf(b, sizeof b, "%.6f", v);
            j += std::string("  \"") + k + "\": " + (std::isfinite(v) ? b : "null") + ",\n";
        };
        auto cnt = [&](const char *k, int64_t v) { j += std::string("  \"") + k + "\": " + std::to_string(v) + ",\n"; };

        cnt("format_version", 1);
        str("imu_time_source", decided_ ? ModeName(mode_) : "undecided");
        str("requested", ModeName(cfg_.mode));
        str("decision", reason_);
        str("method", mode_ == Mode::Sensor
                          ? "IMU timestamp = L2 IMU packet stamp - zero + imu_offset_ns; point gps_time = (L2 point packet stamp - zero) / 1e9 + j * time_increment. Both on the L2 clock."
                          : "IMU clock rebuilt from host arrivals (lower edge, ImuTimestamper), mapped onto the L2 clock by the lower edge of point packet (L2 end-of-packet time, host arrival) over map_window_ns, + imu_offset_ns. Assumes equal minimum delivery latency of IMU and point packets.");
        cnt("zero_l2_ns", zero_);
        cnt("imu_offset_ns", cfg_.imu_offset_ns);
        cnt("epoch_offset_ns", epoch_offset_);
        cnt("imu_samples_in", (int64_t)stats_.imu_in);
        cnt("imu_samples_out", (int64_t)stats_.imu_out);
        cnt("point_packets", (int64_t)stats_.point_packets);
        cnt("dropped_non_increasing", (int64_t)stats_.non_increasing_dropped);
        cnt("dropped_before_zero", (int64_t)stats_.negative_dropped);
        cnt("dropped_duplicate", (int64_t)stats_.duplicates_dropped);
        cnt("dropped_unmapped", (int64_t)stats_.unmapped_dropped);
        cnt("lost_samples", (int64_t)stats_.lost_samples);
        cnt("stamp_jumps", (int64_t)stats_.stamp_jumps);
        cnt("nudged", (int64_t)stats_.nudged);
        if (mode_ == Mode::HostAligned && map_fitted_)
        {
            num("map_l2_per_host_ppm", (1.0 / map_.slope - 1.0) * 1e6);
            num("host_imu_period_ns", host_.GetStats().period_ns);
        }
        num("span_s", c.span_s);
        num("lidar_clock_rate_ppm_vs_host", c.valid ? c.lidar_rate_ppm : NAN);
        num("imu_stamp_rate_ppm_vs_host", c.valid ? c.imu_rate_ppm : NAN);
        num("imu_period_ns_l2", c.valid ? c.imu_period_ns : NAN);
        num("imu_stamp_spacing_ms_p1", c.spacing_p1_ms);
        num("imu_stamp_spacing_ms_p50", c.spacing_p50_ms);
        num("imu_stamp_spacing_ms_p99", c.spacing_p99_ms);
        num("imu_minus_lidar_min_latency_ms_start", c.valid ? c.min_latency_diff_start_ms : NAN);
        num("imu_minus_lidar_min_latency_ms_end", c.valid ? c.min_latency_diff_end_ms : NAN);
        num("imu_mean_extra_latency_ms", c.valid ? c.imu_extra_latency_ms : NAN);
        num("lidar_mean_extra_latency_ms", c.valid ? c.lidar_extra_latency_ms : NAN);
        num("old_method_imu_lag_ms_start", c.valid ? c.old_lag_start_ms : NAN);
        num("old_method_imu_lag_ms_end", c.valid ? c.old_lag_end_ms : NAN);
        j.erase(j.size() - 2); // last ",\n"
        j += "\n}\n";
        return j;
    }

private:
    Config cfg_;
    Stats stats_;
    ImuTimestamper host_;
    Mode mode_;
    bool decided_ = false;
    std::string reason_;

    bool zero_set_ = false;
    int64_t zero_ = 0;
    bool host_zero_set_ = false;
    int64_t host_zero_ = 0;

    std::vector<ImuInput> raw_;             // Sensor / undecided: not yet written
    std::vector<OutputImuData> host_out_;   // HostAligned / undecided: host-clock times, not yet mapped

    // Diagnostics over the whole recording.
    LowerEdge imu_edge_;      // (IMU stamp, host monotonic arrival)
    LowerEdge imu_unix_edge_; // (IMU stamp, host Unix arrival): what the old method wrote
    LowerEdge pts_edge_;      // (L2 end-of-packet time, host monotonic arrival)
    ImuInput imu_first_{}, imu_last_{};
    uint64_t imu_n_ = 0;
    static constexpr int64_t kSpacingBinNs = 1000;
    std::vector<uint32_t> spacing_hist_ = std::vector<uint32_t>(20001, 0); // stamp step, 1 us bins
    int64_t pts_min_latency_ = std::numeric_limits<int64_t>::max();
    int64_t pts_unix_offset_ = std::numeric_limits<int64_t>::max();

    // HostAligned: L2 time (x, rel. zero) of host monotonic time (y, rel. host_zero_).
    std::deque<std::pair<int64_t, int64_t>> map_pts_;
    LowerEdge::Line map_;
    bool map_fitted_ = false;
    int64_t map_fit_y_ = 0;

    // Output state.
    bool epoch_set_ = false;
    int64_t epoch_offset_ = 0;
    bool has_last_out_ = false;
    int64_t last_out_ = 0;

    // Sensor: stamp / seq history.
    bool has_prev_ = false;
    int64_t prev_sensor_ = 0;
    uint32_t prev_seq_ = 0;
    uint64_t seq_steps_ = 0, seq_ones_ = 0;
    double period_est_ = 0;

    void NoteHost(int64_t mono)
    {
        if (!host_zero_set_)
        {
            host_zero_ = mono;
            host_zero_set_ = true;
        }
    }

    void TryDecide(std::vector<OutputImuData> &out, bool final)
    {
        if (!final)
        {
            if (!zero_set_ || raw_.size() < 2 || stats_.point_packets < cfg_.min_point_packets)
                return;
            if (raw_.back().host_mono_ns - raw_.front().host_mono_ns < cfg_.validate_ns)
                return;
        }

        size_t n = raw_.size();
        size_t increasing = 0;
        int64_t imu_min_latency = std::numeric_limits<int64_t>::max();
        for (size_t i = 0; i < n; i++)
        {
            if (i > 0 && raw_[i].sensor_ns > raw_[i - 1].sensor_ns)
                increasing++;
            imu_min_latency = std::min(imu_min_latency, raw_[i].host_mono_ns - raw_[i].sensor_ns);
        }
        double inc = n > 1 ? (double)increasing / (double)(n - 1) : 0;

        // Evenly spaced: sample stamps, not the send times of batches.
        std::vector<int64_t> steps;
        for (size_t i = 1; i < n; i++)
        {
            uint32_t dseq = raw_[i].seq - raw_[i - 1].seq;
            int64_t ds = raw_[i].sensor_ns - raw_[i - 1].sensor_ns;
            if (dseq >= 1 && dseq <= 10)
                steps.push_back(ds / (int64_t)dseq);
        }
        double regular = 0;
        int64_t median_step = 0;
        if (!steps.empty())
        {
            std::vector<int64_t> sorted = steps;
            std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
            median_step = sorted[sorted.size() / 2];
            size_t within = 0;
            for (int64_t d : steps)
                within += median_step > 0 && 2 * d >= median_step && 2 * d <= 3 * median_step;
            regular = (double)within / (double)steps.size();
        }
        int64_t dh = n > 1 ? raw_.back().host_mono_ns - raw_.front().host_mono_ns : 0;
        double rate = dh > 0 ? (double)(raw_.back().sensor_ns - raw_.front().sensor_ns) / (double)dh : 0;
        bool have_points = stats_.point_packets > 0;
        double diff_ms = have_points && n > 0 ? (double)(imu_min_latency - pts_min_latency_) / 1e6 : NAN;

        bool ok = n > 1 && have_points && inc >= cfg_.min_increasing && regular >= cfg_.min_regular &&
                  std::fabs(rate - 1.0) <= cfg_.max_rate_error &&
                  std::fabs(diff_ms) * 1e6 <= (double)cfg_.same_clock_tolerance_ns;
        char b[400];
        snprintf(b, sizeof b,
                 "%s: %zu IMU samples, %.2f%% stamps increasing, %.2f%% steps within 0.5-1.5 x median %.4f ms, "
                 "stamp rate / host rate - 1 = %.2e, IMU - point min latency = %.3f ms (tolerance %.0f ms)%s",
                 ok ? "sensor stamps accepted" : "sensor stamps rejected", n, inc * 100, regular * 100,
                 (double)median_step / 1e6, rate - 1.0, diff_ms, (double)cfg_.same_clock_tolerance_ns / 1e6,
                 final ? ", decided at end of capture" : "");
        reason_ = b;
        mode_ = ok ? Mode::Sensor : Mode::HostAligned;
        decided_ = true;
        if (cfg_.log)
            std::cerr << "IMU time source: " << ModeName(mode_) << " (" << reason_ << ")" << std::endl;

        if (mode_ == Mode::Sensor)
        {
            host_out_.clear();
            DrainRaw(out);
        }
        else
        {
            raw_.clear();
            EmitHost(out);
        }
    }

    void SetEpoch()
    {
        if (epoch_set_)
            return;
        epoch_set_ = true;
        epoch_offset_ = zero_ + (pts_unix_offset_ != std::numeric_limits<int64_t>::max() ? pts_unix_offset_ : 0);
    }

    void Output(OutputImuData data, int64_t t, std::vector<OutputImuData> &out)
    {
        SetEpoch();
        data.LidarTimestamp = t;
        data.EpochTimestamp = t + epoch_offset_;
        out.push_back(data);
        has_last_out_ = true;
        last_out_ = t;
        stats_.imu_out++;
    }

    void DrainRaw(std::vector<OutputImuData> &out)
    {
        if (!zero_set_)
            return;
        for (const ImuInput &in : raw_)
            EmitSensor(in, out);
        raw_.clear();
    }

    void EmitSensor(const ImuInput &in, std::vector<OutputImuData> &out)
    {
        uint32_t dseq = 1;
        if (has_prev_)
        {
            dseq = in.seq - prev_seq_;
            seq_steps_++;
            seq_ones_ += dseq == 1;
            bool seq_counts_imu = seq_ones_ * 10 >= seq_steps_ * 9;
            if (seq_counts_imu && dseq == 0)
            {
                stats_.duplicates_dropped++;
                return;
            }
            if (in.sensor_ns <= prev_sensor_)
            {
                stats_.non_increasing_dropped++;
                return;
            }
            int64_t ds = in.sensor_ns - prev_sensor_;
            if (seq_counts_imu && dseq > 1 && dseq < 100000)
                stats_.lost_samples += dseq - 1;
            double explained = seq_counts_imu && dseq < 100000 ? 2.0 * dseq * period_est_ : 0;
            if (ds > cfg_.stamp_jump_ns && (double)ds > explained)
            {
                stats_.stamp_jumps++;
                if (cfg_.log)
                    std::cerr << "WARNING: IMU stamp jumped " << ds / 1e6 << " ms (seq +" << dseq << ")." << std::endl;
            }
            if (dseq == 1 && ds < 20000000)
                period_est_ = period_est_ > 0 ? 0.99 * period_est_ + 0.01 * (double)ds : (double)ds;
        }
        has_prev_ = true;
        prev_sensor_ = in.sensor_ns;
        prev_seq_ = in.seq;

        int64_t t = in.sensor_ns - zero_ + cfg_.imu_offset_ns;
        if (t < 0)
        {
            stats_.negative_dropped++;
            return;
        }
        if (has_last_out_ && t <= last_out_)
        {
            stats_.non_increasing_dropped++;
            return;
        }
        Output(in.data, t, out);
    }

    void EmitHost(std::vector<OutputImuData> &out)
    {
        if (!zero_set_ || !map_fitted_)
            return;
        for (const OutputImuData &s : host_out_)
        {
            int64_t t = map_.InverseNs(s.LidarTimestamp) + cfg_.imu_offset_ns;
            if (t < 0)
            {
                stats_.negative_dropped++;
                continue;
            }
            if (has_last_out_ && t <= last_out_)
            {
                t = last_out_ + 1;
                stats_.nudged++;
            }
            Output(s, t, out);
        }
        host_out_.clear();
    }
};
