#pragma once

#include <stddef.h>
#include <stdint.h>
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

// Rebuilds evenly spaced, strictly increasing IMU sample times from packet
// stamps that carry transport jitter.
//
// With the SDK's use_system_timestamp (the default), every packet stamp is
// the host arrival time. The L2 sends IMU samples in batches of 1-5, so
// arrival times bunch up (~20 us apart inside a batch, 1-6 ms between
// batches) although the samples were measured ~1.1 ms apart.
//
// Model: arrival = measurement + latency, latency >= 0. The period is a
// least-squares fit of arrival time vs sample index over a sliding window
// (not hard-coded). Each sample gets t = t_prev + n * period (n from the
// packet seq when it counts IMU packets, so lost packets keep their slot),
// then:
//   - t is clamped to never be later than its arrival time,
//   - t is pulled forward by a small fraction of its lead on arrival, so it
//     tracks the low-latency envelope of arrivals without long-term drift,
//   - t is kept within kMaxLagNs of arrival.
// A gap in arrivals much longer than a batch is a real dropout: the
// timeline resyncs to arrival time and the event is logged; so does a
// backward clock step of the same size.
//
// The first kWarmupSamples samples are held back until the first period
// fit; after that samples are emitted immediately. Call Flush() at the end
// of the capture in case warm-up never completed.
class ImuTimestamper
{
public:
    struct Stats
    {
        uint64_t samples = 0;
        double period_ns = 0;
        bool seq_used = false;
        uint64_t lost_packets = 0; // from seq gaps
        uint64_t resyncs = 0;      // dropouts or clock steps
        uint64_t lag_clamps = 0;   // times t had to be pulled to arrival - kMaxLagNs
        double lead_sum_ns = 0;    // sum of (arrival - t)
        int64_t max_lead_ns = 0;
    };

    // stamp_ns: packet stamp relative to GetGlobalTimeOffsetNs().
    // host_unix_ns: host receive time, only used once to anchor EpochTimestamp.
    void Push(const OutputImuData &sample, int64_t stamp_ns, uint32_t seq,
              int64_t host_unix_ns, std::vector<OutputImuData> &out)
    {
        if (!has_epoch_offset_)
        {
            epoch_offset_ns_ = host_unix_ns - stamp_ns;
            has_epoch_offset_ = true;
        }

        if (!tracking_)
        {
            if (!warmup_.empty() && IsDropout(stamp_ns, warmup_.back().stamp_ns))
            {
                StartTracking(out); // the current sample then resyncs in Track()
            }
            else
            {
                warmup_.push_back({sample, stamp_ns, seq});
                if (warmup_.size() >= kWarmupSamples)
                {
                    StartTracking(out);
                }
                return;
            }
        }

        Track(sample, stamp_ns, seq, out);
    }

    void Flush(std::vector<OutputImuData> &out)
    {
        if (!tracking_ && !warmup_.empty())
        {
            StartTracking(out);
        }
    }

    const Stats &GetStats() const { return stats_; }

private:
    static constexpr size_t kWarmupSamples = 256;   // ~0.3 s before the first fit
    static constexpr size_t kFitWindow = 4096;      // ~4.5 s of samples in the period fit
    static constexpr size_t kRefitEvery = 256;
    static constexpr int64_t kDropoutNs = 20000000; // batches are <= ~6 ms apart
    static constexpr int64_t kMaxLagNs = 10000000;  // t stays within 10 ms of arrival
    static constexpr double kLeadGain = 0.002;      // share of the lead recovered per sample
    static constexpr uint32_t kMaxSeqGap = 1000;

    struct Pending
    {
        OutputImuData data;
        int64_t stamp_ns;
        uint32_t seq;
    };

    struct FitPoint
    {
        int64_t index;
        int64_t stamp_ns;
    };

    std::vector<Pending> warmup_;
    std::deque<FitPoint> fit_;
    size_t since_refit_ = 0;

    bool tracking_ = false;
    bool seq_usable_ = false;
    double period_ns_ = 0;
    int64_t t_prev_ = 0;
    int64_t stamp_prev_ = 0;
    uint32_t seq_prev_ = 0;
    int64_t index_prev_ = 0;

    bool has_epoch_offset_ = false;
    int64_t epoch_offset_ns_ = 0;

    Stats stats_;

    static bool IsDropout(int64_t stamp_ns, int64_t prev_stamp_ns)
    {
        int64_t gap = stamp_ns - prev_stamp_ns;
        return gap > kDropoutNs || gap < -kDropoutNs;
    }

    // Samples covered since the previous packet. The seq is only trusted if
    // it counts IMU packets (steps of 1), not a counter shared with points.
    uint32_t SampleSteps(uint32_t seq, uint32_t prev_seq)
    {
        if (!seq_usable_)
            return 1;
        uint32_t step = seq - prev_seq;
        if (step == 0 || step > kMaxSeqGap)
            return 1;
        stats_.lost_packets += step - 1;
        return step;
    }

    // Least-squares slope of stamp vs sample index over the fit window.
    bool FitPeriod(double &period_ns) const
    {
        if (fit_.size() < 2)
            return false;
        const FitPoint &ref = fit_.front();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (const FitPoint &p : fit_)
        {
            double x = (double)(p.index - ref.index);
            double y = (double)(p.stamp_ns - ref.stamp_ns);
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        double n = (double)fit_.size();
        double den = n * sxx - sx * sx;
        if (den <= 0)
            return false;
        double slope = (n * sxy - sx * sy) / den;
        if (slope < 1)
            return false;
        period_ns = slope;
        return true;
    }

    void Emit(const OutputImuData &data, int64_t t, int64_t stamp_ns, std::vector<OutputImuData> &out)
    {
        OutputImuData sample = data;
        sample.LidarTimestamp = t;
        sample.EpochTimestamp = t + epoch_offset_ns_;
        out.push_back(sample);

        int64_t lead = stamp_ns - t;
        stats_.samples++;
        stats_.lead_sum_ns += (double)lead;
        if (lead > stats_.max_lead_ns)
            stats_.max_lead_ns = lead;
    }

    // Fits the period on the warm-up buffer and places those samples on the
    // lowest line that is nowhere later than their arrival times.
    void StartTracking(std::vector<OutputImuData> &out)
    {
        size_t unit_steps = 0;
        for (size_t i = 1; i < warmup_.size(); i++)
        {
            if (warmup_[i].seq - warmup_[i - 1].seq == 1)
                unit_steps++;
        }
        seq_usable_ = warmup_.size() > 1 && unit_steps * 10 >= (warmup_.size() - 1) * 9;
        stats_.seq_used = seq_usable_;

        std::vector<int64_t> index(warmup_.size(), 0);
        fit_.clear();
        for (size_t i = 0; i < warmup_.size(); i++)
        {
            if (i > 0)
                index[i] = index[i - 1] + SampleSteps(warmup_[i].seq, warmup_[i - 1].seq);
            fit_.push_back({index[i], warmup_[i].stamp_ns});
        }

        if (!FitPeriod(period_ns_) && period_ns_ <= 0)
        {
            period_ns_ = 1000000; // single sample: any positive value works until the next fit
        }
        stats_.period_ns = period_ns_;

        double intercept = std::numeric_limits<double>::max();
        for (size_t i = 0; i < warmup_.size(); i++)
        {
            double c = (double)warmup_[i].stamp_ns - (double)index[i] * period_ns_;
            if (c < intercept)
                intercept = c;
        }

        for (size_t i = 0; i < warmup_.size(); i++)
        {
            int64_t t = (int64_t)(intercept + (double)index[i] * period_ns_);
            if (t > warmup_[i].stamp_ns)
                t = warmup_[i].stamp_ns; // float rounding at the envelope point
            if (i > 0 && t <= t_prev_)
                t = t_prev_ + 1;
            t_prev_ = t;
            Emit(warmup_[i].data, t, warmup_[i].stamp_ns, out);
        }

        stamp_prev_ = warmup_.back().stamp_ns;
        seq_prev_ = warmup_.back().seq;
        index_prev_ = index.back();
        since_refit_ = 0;
        warmup_.clear();
        tracking_ = true;
    }

    void Track(const OutputImuData &sample, int64_t stamp_ns, uint32_t seq, std::vector<OutputImuData> &out)
    {
        int64_t t;
        int64_t index;

        if (IsDropout(stamp_ns, stamp_prev_))
        {
            std::cerr << "WARNING: IMU dropout/clock step: arrival gap "
                      << (double)(stamp_ns - stamp_prev_) / 1e6 << " ms, resyncing IMU timeline to arrival time."
                      << std::endl;
            stats_.resyncs++;
            t = stamp_ns;
            index = index_prev_ + 1;
            fit_.clear(); // keep period_ns_ until the window refills
            since_refit_ = 0;
        }
        else
        {
            uint32_t steps = SampleSteps(seq, seq_prev_);
            index = index_prev_ + steps;

            double predicted = (double)t_prev_ + steps * period_ns_;
            double lead = (double)stamp_ns - predicted;
            t = lead < 0 ? stamp_ns : (int64_t)(predicted + kLeadGain * lead);

            if (stamp_ns - t > kMaxLagNs)
            {
                t = stamp_ns - kMaxLagNs;
                stats_.lag_clamps++;
            }
        }

        if (t <= t_prev_)
            t = t_prev_ + 1;

        fit_.push_back({index, stamp_ns});
        if (fit_.size() > kFitWindow)
            fit_.pop_front();
        if (++since_refit_ >= kRefitEvery && fit_.size() >= kWarmupSamples)
        {
            FitPeriod(period_ns_);
            stats_.period_ns = period_ns_;
            since_refit_ = 0;
        }

        t_prev_ = t;
        stamp_prev_ = stamp_ns;
        seq_prev_ = seq;
        index_prev_ = index;
        Emit(sample, t, stamp_ns, out);
    }
};
