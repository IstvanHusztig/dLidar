#pragma once

#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <vector>

struct OutputImuData
{
    int64_t LidarTimestamp; // ns, relative to GetGlobalTimeOffsetNs() (same base as point times)
    int64_t EpochTimestamp; // ns, Unix epoch; LidarTimestamp + a fixed offset
    int ImuId;

    float GyroX;
    float GyroY;
    float GyroZ;

    float AccelerationX;
    float AccelerationY;
    float AccelerationZ;
};

// Rebuilds IMU acquisition times from host arrival times.
//
// With the SDK's use_system_timestamp (the default), every packet stamp is
// the host arrival time. The IMU samples at a constant period T, and every
// sample arrives no earlier than it was acquired, often much later (the L2
// sends samples in batches, the host buffers them). So the acquisition clock
// is the lower edge of the arrival times:
//
//   t_k = t_0 + k*T + o_k,   t_k <= arrival_k
//
// - k: sample index. Taken from the packet seq if it counts IMU packets
//   (detected at startup), else counted. Without a usable seq, lost samples
//   are found from the arrival times: if arrival - t stays above
//   kDropLeadPeriods*T for kDropPersistNs, round(excess / T) samples were
//   lost and k is advanced instead of compressing the timestamps.
// - T: slope of the lower convex hull of (k, arrival) over the last
//   fit_window_ns, taken at the hull edge that spans the mean k. This is the
//   line under all arrivals that is closest to them on average (the LP clock
//   skew estimator); it needs no prior T and ignores late arrivals entirely.
// - o_k: output is delayed by lookahead_ns. Every arrival j within that
//   window on either side bounds the acquisition time from above:
//   t_k <= arrival_j - (j-k)*T. t_k follows the lowest of those bounds,
//   moving at most track_slew*T per step away from t_prev + n*T. The later
//   arrivals are also checked with max_slew*T per sample, so t never ends up
//   later than any arrival. Slew, never step.
//
// No output until the arrivals span warmup_ns and T has settled; the held
// samples are then written with times computed backwards from the fitted
// clock. Output is strictly increasing. Samples that would get t < 0 (they
// were acquired before the shared zero point) are dropped. EpochTimestamp is
// LidarTimestamp plus one constant offset.
//
// Call Flush() at the end of the capture to write the held samples.
class ImuTimestamper
{
public:
    struct Config
    {
        int64_t warmup_ns = 2000000000;       // no output before this much data
        int64_t first_fit_ns = 500000000;     // first period fit (enables drop detection)
        int64_t lookahead_ns = 500000000;     // output delay, envelope window on each side
        int64_t fit_window_ns = 20000000000;  // period fit window
        int64_t refit_every_ns = 500000000;   // period refit interval
        double converged_change = 0.001;      // output starts once T changes less than this between fits
        double track_slew = 0.0025;           // normal |step - T| / T while following the envelope
        double max_slew = 0.01;               // hard |step - T| / T used to plan ahead
        double drop_lead_periods = 0.75;      // lead (in T) that suggests lost samples
        int64_t drop_persist_ns = 200000000;  // ... if it persists this long
        int64_t clock_step_ns = 50000000;     // larger jumps are clock steps / resyncs
        uint32_t max_seq_gap = 100000;        // larger seq jumps are treated as a seq reset
        bool log = true;                      // print events to stderr
    };

    struct Stats
    {
        uint64_t samples_in = 0;
        uint64_t samples_out = 0;
        double period_ns = 0;
        bool seq_used = false;
        uint64_t lost_samples = 0;        // from seq gaps or detected drops
        uint64_t drop_events = 0;         // detected drops (no seq)
        uint64_t index_corrections = 0;   // k moved back (arrival earlier than possible)
        uint64_t resyncs = 0;             // timeline jumped (clock step forward, seq reset)
        uint64_t clock_steps_back = 0;    // arrival clock went backwards, estimator restarted
        uint64_t slew_violations = 0;     // t had to move faster than max_slew
        uint64_t conflicts_dropped = 0;   // could not be both increasing and <= arrival
        uint64_t negative_dropped = 0;    // acquired before the zero point
        uint64_t duplicates_dropped = 0;  // repeated seq
        double lead_sum_ns = 0;           // sum of (arrival - t)
        int64_t max_lead_ns = 0;
    };

    ImuTimestamper() : ImuTimestamper(Config()) {}
    explicit ImuTimestamper(const Config &config) : cfg_(config) {}

    // arrival_ns: packet stamp relative to GetGlobalTimeOffsetNs().
    // host_unix_ns: host receive time, only used once to anchor EpochTimestamp.
    void Push(const OutputImuData &sample, int64_t arrival_ns, uint32_t seq,
              int64_t host_unix_ns, std::vector<OutputImuData> &out)
    {
        if (!has_epoch_offset_)
        {
            epoch_offset_ns_ = host_unix_ns - arrival_ns;
            has_epoch_offset_ = true;
        }
        stats_.samples_in++;

        if (any_ && arrival_ns < arrival_last_ - cfg_.clock_step_ns)
        {
            if (cfg_.log)
                std::cerr << "WARNING: IMU arrival clock went back "
                          << (double)(arrival_last_ - arrival_ns) / 1e6 << " ms, restarting IMU timeline." << std::endl;
            stats_.clock_steps_back++;
            Restart(out);
        }

        bool seq_mode = seq_decided_ && seq_usable_;
        bool resync = false;
        int64_t k = 0;
        if (any_)
        {
            k = k_last_ + 1;
            if (seq_mode)
            {
                uint32_t d = seq - seq_last_;
                if (d == 0)
                {
                    stats_.duplicates_dropped++;
                    return;
                }
                if (d <= cfg_.max_seq_gap)
                {
                    k = k_last_ + d;
                    stats_.lost_samples += d - 1;
                }
                else
                {
                    if (cfg_.log)
                        std::cerr << "WARNING: IMU seq jumped from " << seq_last_ << " to " << seq
                                  << ", resyncing IMU timeline." << std::endl;
                    resync = true;
                    stats_.resyncs++;
                }
            }
        }

        double lead = 0;
        if (fitted_ && !resync)
        {
            lead = (double)arrival_ns - LineAt(k);
            if (!seq_mode && lead < -0.5 * period_)
            {
                // Earlier than this index can have been acquired: k is too high
                // (an earlier drop was overestimated).
                int64_t nk = std::max(k - (int64_t)std::llround(-lead / period_), k_last_ + 1);
                if (nk != k)
                {
                    stats_.index_corrections++;
                    k = nk;
                    lead = (double)arrival_ns - LineAt(k);
                }
            }

            double threshold = seq_mode ? (double)cfg_.clock_step_ns : cfg_.drop_lead_periods * period_;
            if (lead > threshold)
            {
                if (!streak_)
                {
                    streak_ = true;
                    streak_n_ = n_next_;
                    streak_arrival_ = arrival_ns;
                }
            }
            else
            {
                streak_ = false;
            }
        }

        if (!any_)
            start_arrival_ = arrival_ns;
        pending_.push_back({sample, k, arrival_ns, seq, n_next_, lead, resync});
        fit_.push_back({k, arrival_ns, n_next_});
        n_next_++;
        k_last_ = k;
        seq_last_ = seq;
        arrival_last_ = any_ ? std::max(arrival_last_, arrival_ns) : arrival_ns;
        any_ = true;

        while (fit_.size() > 2 && arrival_last_ - fit_.front().arrival > cfg_.fit_window_ns)
            fit_.pop_front();

        if (streak_ && arrival_ns - streak_arrival_ >= cfg_.drop_persist_ns)
            ResolveStreak(seq_mode);

        if (!fitted_)
        {
            if (arrival_last_ - start_arrival_ >= cfg_.first_fit_ns)
            {
                DecideSeq();
                if (Refit())
                    last_fit_arrival_ = arrival_ns;
            }
        }
        else if (arrival_ns - last_fit_arrival_ >= cfg_.refit_every_ns)
        {
            double previous = period_;
            if (Refit())
            {
                last_fit_arrival_ = arrival_ns;
                if (!converged_ && arrival_last_ - start_arrival_ >= cfg_.warmup_ns &&
                    std::fabs(period_ - previous) < cfg_.converged_change * previous)
                {
                    converged_ = true;
                }
            }
        }

        if (converged_)
            EmitReady(out, false);
    }

    void Flush(std::vector<OutputImuData> &out)
    {
        if (pending_.empty())
            return;
        if (!fitted_)
        {
            DecideSeq();
            Refit();
        }
        if (!fitted_)
        {
            // Too little data for a period fit: keep arrival times.
            for (const Sample &s : pending_)
            {
                int64_t t = s.arrival;
                if (has_floor_ && t <= t_p_)
                {
                    stats_.conflicts_dropped++;
                    continue;
                }
                Output(s, t, out);
            }
            pending_.clear();
            return;
        }
        streak_ = false;
        EmitReady(out, true);
    }

    const Stats &GetStats() const { return stats_; }

private:
    struct Sample
    {
        OutputImuData data;
        int64_t k;
        int64_t arrival;
        uint32_t seq;
        uint64_t n; // push order
        double lead;
        bool resync;
    };

    struct Arrival
    {
        int64_t k;
        int64_t arrival;
        uint64_t n;
    };

    struct HullPoint
    {
        double x, y;
    };

    Config cfg_;
    Stats stats_;

    std::deque<Sample> pending_;  // not yet emitted
    std::deque<Arrival> history_; // emitted, within the lookahead window
    std::deque<Arrival> fit_;     // period fit window
    std::vector<HullPoint> hull_;

    bool any_ = false;
    int64_t k_last_ = 0;
    uint32_t seq_last_ = 0;
    int64_t arrival_last_ = 0;
    int64_t start_arrival_ = 0;
    uint64_t n_next_ = 0;

    bool seq_decided_ = false;
    bool seq_usable_ = false;

    bool fitted_ = false;
    bool converged_ = false;
    double period_ = 0;
    int64_t line_k_ = 0; // a point on the fitted lower edge
    double line_t_ = 0;
    int64_t last_fit_arrival_ = 0;

    bool emitted_ = false;  // anchor (k_p_, t_p_) is valid for slewing
    bool has_floor_ = false; // t_p_ is a floor for strictly increasing output
    int64_t k_p_ = 0;
    int64_t t_p_ = 0;

    bool streak_ = false; // samples currently arriving with a lead above threshold
    uint64_t streak_n_ = 0;
    int64_t streak_arrival_ = 0;

    bool has_epoch_offset_ = false;
    int64_t epoch_offset_ns_ = 0;

    double LineAt(int64_t k) const
    {
        if (emitted_)
            return (double)t_p_ + (double)(k - k_p_) * period_;
        return line_t_ + (double)(k - line_k_) * period_;
    }

    // At the first fit: does seq count IMU packets (steps of 1), or is it a
    // counter shared with other packets? If usable, re-index from it.
    void DecideSeq()
    {
        if (seq_decided_)
            return;
        seq_decided_ = true;
        size_t unit = 0;
        for (size_t i = 1; i < pending_.size(); i++)
        {
            if (pending_[i].seq - pending_[i - 1].seq == 1)
                unit++;
        }
        seq_usable_ = pending_.size() > 1 && unit * 10 >= (pending_.size() - 1) * 9;
        stats_.seq_used = seq_usable_;
        if (!seq_usable_)
            return;

        size_t kept = 1;
        for (size_t i = 1; i < pending_.size(); i++)
        {
            uint32_t d = pending_[i].seq - pending_[kept - 1].seq;
            if (d == 0)
            {
                stats_.duplicates_dropped++;
                continue;
            }
            if (d > cfg_.max_seq_gap)
                d = 1;
            stats_.lost_samples += d - 1;
            Sample s = pending_[i];
            s.k = pending_[kept - 1].k + d;
            pending_[kept++] = s;
        }
        pending_.resize(kept);
        fit_.clear();
        for (const Sample &s : pending_)
            fit_.push_back({s.k, s.arrival, s.n});
        k_last_ = pending_.back().k;
    }

    // Lower convex hull of (k, arrival); the edge spanning the mean k gives
    // the period and a point on the lower edge.
    bool Refit()
    {
        if (fit_.size() < 2)
            return false;
        const int64_t k0 = fit_.front().k;
        const int64_t a0 = fit_.front().arrival;
        hull_.clear();
        double sum_x = 0;
        for (const Arrival &p : fit_)
        {
            HullPoint q{(double)(p.k - k0), (double)(p.arrival - a0)};
            sum_x += q.x;
            while (hull_.size() >= 2)
            {
                const HullPoint &a = hull_[hull_.size() - 2];
                const HullPoint &b = hull_.back();
                double cross = (b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x);
                if (cross > 0)
                    break;
                hull_.pop_back();
            }
            hull_.push_back(q);
        }
        if (hull_.size() < 2)
            return false;
        double mean_x = sum_x / (double)fit_.size();
        size_t i = 0;
        while (i + 2 < hull_.size() && hull_[i + 1].x < mean_x)
            i++;
        double slope = (hull_[i + 1].y - hull_[i].y) / (hull_[i + 1].x - hull_[i].x);
        if (!(slope > 0))
            return false;
        period_ = slope;
        line_k_ = k0 + (int64_t)hull_[i].x;
        line_t_ = (double)a0 + hull_[i].y;
        fitted_ = true;
        stats_.period_ns = period_;
        return true;
    }

    // The lead stayed above threshold for drop_persist_ns. Without seq:
    // samples were lost, advance k. With seq: the arrival clock jumped
    // forward, let the timeline jump at that sample.
    void ResolveStreak(bool seq_mode)
    {
        streak_ = false;

        // The excess is measured on the later part of the streak, so late
        // samples from before the drop do not shrink it.
        double min_lead = std::numeric_limits<double>::max();
        for (auto it = pending_.rbegin(); it != pending_.rend() && it->n >= streak_n_; ++it)
        {
            if (it->arrival >= streak_arrival_ + cfg_.drop_persist_ns / 2)
                min_lead = std::min(min_lead, it->lead);
        }
        if (min_lead == std::numeric_limits<double>::max())
            return;

        // The jump sits before the first sample of the streak that already
        // shows it; earlier streak samples are just late.
        int64_t shift = seq_mode ? 0 : std::llround(min_lead / period_);
        double split_lead = seq_mode ? min_lead - 0.5 * period_ : ((double)shift - 0.5) * period_;
        uint64_t split_n = n_next_;
        for (const Sample &s : pending_)
        {
            if (s.n >= streak_n_ && s.lead > split_lead)
            {
                split_n = s.n;
                break;
            }
        }
        if (split_n == n_next_)
            return;

        if (seq_mode)
        {
            for (Sample &s : pending_)
            {
                if (s.n == split_n)
                    s.resync = true;
            }
            stats_.resyncs++;
            if (cfg_.log)
                std::cerr << "WARNING: IMU arrival clock jumped forward " << min_lead / 1e6
                          << " ms, resyncing IMU timeline." << std::endl;
            return;
        }
        if (shift < 1)
            return;

        for (auto it = pending_.rbegin(); it != pending_.rend() && it->n >= split_n; ++it)
        {
            it->k += shift;
            it->lead -= (double)shift * period_;
        }
        for (auto it = fit_.rbegin(); it != fit_.rend() && it->n >= split_n; ++it)
            it->k += shift;
        k_last_ += shift;
        stats_.lost_samples += shift;
        stats_.drop_events++;
        if (cfg_.log && shift * period_ >= cfg_.clock_step_ns)
            std::cerr << "WARNING: IMU gap, " << shift << " samples (" << shift * period_ / 1e6
                      << " ms) missing." << std::endl;
    }

    void EmitReady(std::vector<OutputImuData> &out, bool flush)
    {
        const double window_k = (double)cfg_.lookahead_ns / period_;
        while (!pending_.empty())
        {
            const Sample &f = pending_.front();
            if (!flush)
            {
                if ((double)(pending_.back().k - f.k) < window_k)
                    break;
                if (streak_ && f.n >= streak_n_)
                    break;
            }
            EmitFront(window_k);
            if (!out_buffer_.empty())
            {
                out.insert(out.end(), out_buffer_.begin(), out_buffer_.end());
                out_buffer_.clear();
            }
        }
    }

    std::vector<OutputImuData> out_buffer_;

    void EmitFront(double window_k)
    {
        const Sample f = pending_.front();
        pending_.pop_front();
        const double T = period_;

        // Envelope: lowest acquisition-time bound from arrivals around f.
        // Ceiling: highest t from which every later arrival stays reachable
        // at max_slew.
        double envelope = (double)f.arrival;
        double ceiling = (double)f.arrival;
        for (const Arrival &h : history_)
        {
            double v = (double)h.arrival + (double)(f.k - h.k) * T;
            envelope = std::min(envelope, v);
        }
        for (const Sample &p : pending_)
        {
            double dk = (double)(p.k - f.k);
            double v = (double)p.arrival - dk * T;
            if (dk <= window_k)
                envelope = std::min(envelope, v);
            ceiling = std::min(ceiling, v + cfg_.max_slew * T * dk);
        }

        double t = envelope;
        if (emitted_ && !f.resync)
        {
            // Across a gap of lost samples the step is exactly n*T; the
            // offset moves no more than over a single step.
            double base = (double)t_p_ + (double)(f.k - k_p_) * T;
            t = std::clamp(envelope, base - cfg_.track_slew * T, base + cfg_.track_slew * T);
            if (ceiling < base - cfg_.max_slew * T - 1)
                stats_.slew_violations++;
        }
        t = std::min(t, ceiling);

        int64_t ti = (int64_t)std::floor(t);
        if (has_floor_ && ti <= t_p_)
        {
            ti = t_p_ + 1;
            if (ti > f.arrival)
            {
                stats_.conflicts_dropped++;
                history_.push_back({f.k, f.arrival, f.n});
                return;
            }
        }

        k_p_ = f.k;
        t_p_ = ti;
        emitted_ = true;
        has_floor_ = true;
        history_.push_back({f.k, f.arrival, f.n});
        while (!history_.empty() && (double)(f.k - history_.front().k) > window_k)
            history_.pop_front();

        if (ti < 0)
        {
            stats_.negative_dropped++;
            return;
        }
        Output(f, ti, out_buffer_);
    }

    void Output(const Sample &s, int64_t t, std::vector<OutputImuData> &out)
    {
        OutputImuData sample = s.data;
        sample.LidarTimestamp = t;
        sample.EpochTimestamp = t + epoch_offset_ns_;
        out.push_back(sample);
        t_p_ = t;
        has_floor_ = true;

        int64_t lead = s.arrival - t;
        stats_.samples_out++;
        stats_.lead_sum_ns += (double)lead;
        stats_.max_lead_ns = std::max(stats_.max_lead_ns, lead);
    }

    // The arrival clock stepped back: write what is held, then start over
    // with a new fit. The last written time stays the floor.
    void Restart(std::vector<OutputImuData> &out)
    {
        Flush(out);
        history_.clear();
        fit_.clear();
        any_ = false;
        fitted_ = false;
        converged_ = false;
        emitted_ = false;
        streak_ = false;
    }
};
