// Feeds ImuTimestamper simulated arrival times and checks the rebuilt
// timestamps against the true acquisition times.
//
// True acquisition: t_k = sum of periods, T = 1.1947 ms drifting by 50 ppm
// over the run. Pass criteria for every case:
//   - after the first 2 s, |t_k - true_k| < 0.2 ms
//   - every step within T +- 2% (across real drops: missing samples * T)
//   - strictly increasing, never later than arrival, never negative
#include "imu_timestamper.h"

#include <cstdio>
#include <functional>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace
{
const double kT = 1.1947e6; // ns
const double kDriftPpm = 50.0;

struct SimSample
{
    int id;           // true sample index
    double true_t;    // ns
    int64_t arrival;  // ns
    uint32_t seq;
};

struct Scenario
{
    std::string name;
    double duration_s = 60.0;
    // latency model: fills arrivals from true times (vector is in true order)
    std::function<void(std::vector<SimSample> &, std::mt19937_64 &)> deliver;
    std::vector<std::pair<int, int>> drops; // [first id, count]
    bool seq_counts_imu = false;            // seq increments by 1 per IMU sample
    bool seq_shared = false;                // seq also counts other packets
    // Without a usable seq, a sample lost from inside a batch cannot be
    // located: the batch arrives at once. Allow the gap to land this many
    // samples early (ids that close to a drop skip the step/accuracy checks).
    int drop_ambiguity = 0;
    // As in the recorder: time zero is the first arrival, so samples of a
    // startup dump were acquired before zero and must be dropped, not
    // written with negative times.
    bool zero_at_first_arrival = false;
};

std::vector<double> TrueTimes(double duration_s)
{
    std::vector<double> t;
    int n = (int)(duration_s * 1e9 / kT);
    double now = 0;
    for (int k = 0; k < n; k++)
    {
        t.push_back(now);
        now += kT * (1.0 + kDriftPpm * 1e-6 * k / n);
    }
    return t;
}

// Arrival order is reception order: stamps never decrease.
void MakeMonotonic(std::vector<SimSample> &s)
{
    for (size_t i = 1; i < s.size(); i++)
        s[i].arrival = std::max(s[i].arrival, s[i - 1].arrival);
}

void Bursts(std::vector<SimSample> &s, std::mt19937_64 &)
{
    // 4 samples delivered within 0.02 ms, then a gap of ~4.5 ms.
    for (size_t g = 0; g < s.size(); g += 4)
    {
        size_t last = std::min(g + 3, s.size() - 1);
        for (size_t i = g; i <= last; i++)
            s[i].arrival = (int64_t)(s[last].true_t + 30000 + (i - g) * 6000);
    }
}

void RandomLatency(std::vector<SimSample> &s, std::mt19937_64 &rng)
{
    std::uniform_real_distribution<double> lat(0, 3e6);
    for (SimSample &x : s)
        x.arrival = (int64_t)(x.true_t + lat(rng));
    MakeMonotonic(s);
}

void StartupDump(std::vector<SimSample> &s, std::mt19937_64 &rng)
{
    Bursts(s, rng);
    // The first 300 samples are delivered at once.
    int64_t dump = (int64_t)s[299].true_t + 20000;
    for (int i = 0; i < 300; i++)
        s[i].arrival = dump + i * 300;
    MakeMonotonic(s);
}

void LongStall(std::vector<SimSample> &s, std::mt19937_64 &rng)
{
    RandomLatency(s, rng);
    // A 60 ms stall at 30 s; the delayed samples arrive in a rush after it.
    double stall_start = 30e9, stall_end = 30.06e9;
    int64_t rush = (int64_t)stall_end;
    for (SimSample &x : s)
    {
        if (x.arrival >= stall_start && x.arrival < stall_end)
            x.arrival = (rush += 15000);
    }
    MakeMonotonic(s);
}

void Combined(std::vector<SimSample> &s, std::mt19937_64 &rng)
{
    // Batches of 1-5 with a small random floor, plus a stall and a dump.
    std::uniform_int_distribution<int> batch(1, 5);
    std::uniform_real_distribution<double> lat(0, 0.4e6);
    size_t i = 0;
    while (i < s.size())
    {
        size_t last = std::min(i + batch(rng) - 1, s.size() - 1);
        int64_t a = (int64_t)(s[last].true_t + 20000 + lat(rng));
        for (size_t j = i; j <= last; j++)
            s[j].arrival = a + (int64_t)(j - i) * 20000;
        i = last + 1;
    }
    for (int j = 0; j < 300; j++)
        s[j].arrival = (int64_t)s[299].true_t + 20000;
    int64_t rush = 45060000000;
    for (SimSample &x : s)
    {
        if (x.arrival >= 45000000000 && x.arrival < 45060000000)
            x.arrival = (rush += 15000);
    }
    MakeMonotonic(s);
}

struct Result
{
    bool ok = true;
    int failures = 0;
    std::string first_failure;
};

void Fail(Result &r, const std::string &msg)
{
    if (r.failures++ == 0)
        r.first_failure = msg;
    r.ok = false;
}

std::string Fmt(const char *fmt, double a, double b = 0, double c = 0)
{
    char buf[256];
    snprintf(buf, sizeof buf, fmt, a, b, c);
    return buf;
}

Result Run(const Scenario &sc, uint64_t seed)
{
    Result r;
    std::mt19937_64 rng(seed);
    std::vector<double> truth = TrueTimes(sc.duration_s);

    std::vector<SimSample> sim;
    for (size_t k = 0; k < truth.size(); k++)
        sim.push_back({(int)k, truth[k], 0, 0});
    sc.deliver(sim, rng);
    if (sc.zero_at_first_arrival)
    {
        double zero = (double)sim.front().arrival;
        for (double &t : truth)
            t -= zero;
        for (SimSample &x : sim)
        {
            x.true_t -= zero;
            x.arrival -= (int64_t)zero;
        }
    }

    // Sequence numbers are assigned before drops so lost packets leave gaps.
    std::uniform_int_distribution<int> other(0, 2);
    uint32_t seq = 1000;
    for (SimSample &x : sim)
    {
        x.seq = seq;
        seq += 1 + (sc.seq_shared ? other(rng) : 0);
    }

    std::vector<bool> dropped(sim.size(), false);
    for (auto d : sc.drops)
        for (int i = d.first; i < d.first + d.second; i++)
            dropped[i] = true;

    ImuTimestamper::Config cfg;
    cfg.log = false;
    ImuTimestamper ts(cfg);
    std::vector<OutputImuData> out;
    std::map<int, int64_t> arrival_of;
    size_t pushed = 0;
    for (const SimSample &x : sim)
    {
        if (dropped[x.id])
            continue;
        OutputImuData d{};
        d.ImuId = x.id;
        arrival_of[x.id] = x.arrival;
        uint32_t s = (sc.seq_counts_imu || sc.seq_shared) ? x.seq : 0;
        ts.Push(d, x.arrival, s, 1700000000000000000LL + x.arrival, out);
        pushed++;
    }
    size_t before_flush = out.size();
    ts.Flush(out);

    const ImuTimestamper::Stats &st = ts.GetStats();
    if (st.seq_used != sc.seq_counts_imu)
        Fail(r, "seq usability detected wrongly");

    // Only samples acquired before the zero point may be missing (t < 0).
    size_t acquired_before_zero = 0;
    for (const SimSample &x : sim)
        acquired_before_zero += x.true_t < 0;
    if (out.size() + st.negative_dropped != pushed || st.negative_dropped > acquired_before_zero + 5)
        Fail(r, Fmt("output %g of %g samples, %g negative", (double)out.size(), (double)pushed, (double)st.negative_dropped));
    // Output is delayed ~0.5 s, not held until Flush.
    if (pushed - before_flush > 1000)
        Fail(r, Fmt("%g samples still held at flush", (double)(pushed - before_flush)));

    auto near_drop = [&](int id)
    {
        for (auto d : sc.drops)
            if (id >= d.first - sc.drop_ambiguity && id <= d.first + d.second)
                return true;
        return false;
    };

    double max_err = 0;
    double max_step_dev = 0;
    for (size_t i = 0; i < out.size(); i++)
    {
        const OutputImuData &o = out[i];
        double true_t = truth[o.ImuId];
        if (o.LidarTimestamp < 0)
            Fail(r, Fmt("negative timestamp at id %g", o.ImuId));
        if (o.LidarTimestamp > arrival_of[o.ImuId])
            Fail(r, Fmt("t later than arrival at id %g by %g ns", o.ImuId, (double)(o.LidarTimestamp - arrival_of[o.ImuId])));
        if (o.EpochTimestamp - o.LidarTimestamp != out[0].EpochTimestamp - out[0].LidarTimestamp)
            Fail(r, "timestampUnix offset not constant");
        if (sc.drop_ambiguity && near_drop(o.ImuId))
            continue;
        if (true_t >= truth.front() + 2e9)
        {
            double err = std::fabs((double)o.LidarTimestamp - true_t);
            max_err = std::max(max_err, err);
            if (err >= 0.2e6)
                Fail(r, Fmt("id %g: |t - true| = %g ms", o.ImuId, err / 1e6));
        }
        if (i > 0)
        {
            const OutputImuData &p = out[i - 1];
            if (o.LidarTimestamp <= p.LidarTimestamp)
                Fail(r, Fmt("not strictly increasing at id %g", o.ImuId));
            int missing = o.ImuId - p.ImuId;
            double expected = truth[o.ImuId] - truth[p.ImuId];
            double step = (double)(o.LidarTimestamp - p.LidarTimestamp);
            double dev = std::fabs(step - expected) / kT;
            max_step_dev = std::max(max_step_dev, dev);
            if (dev > 0.02)
                Fail(r, Fmt("id %g: step %g ms over %g samples", o.ImuId, step / 1e6, missing));
        }
    }

    printf("  %-28s seed %llu: %s  max|err| %.4f ms  max step dev %.3f%% T  period %.6f ms  lost %llu  out %zu  t<0 dropped %llu\n",
           sc.name.c_str(), (unsigned long long)seed, r.ok ? "PASS" : "FAIL", max_err / 1e6, max_step_dev * 100,
           st.period_ns / 1e6, (unsigned long long)st.lost_samples, out.size(), (unsigned long long)st.negative_dropped);
    if (!r.ok)
        printf("    %d failures, first: %s\n", r.failures, r.first_failure.c_str());
    return r;
}
} // namespace

int main()
{
    std::vector<Scenario> scenarios;
    scenarios.push_back({"bursts", 60, Bursts});
    scenarios.push_back({"random latency 0-3 ms", 60, RandomLatency});
    scenarios.push_back({"startup buffer dump", 60, StartupDump});
    {
        Scenario s{"startup dump, zero = arrival", 60, StartupDump};
        s.zero_at_first_arrival = true;
        scenarios.push_back(s);
    }
    scenarios.push_back({"dropped 5 (bursts)", 60, Bursts, {{20000, 5}}});
    scenarios.push_back({"dropped 5 (random latency)", 60, RandomLatency, {{20000, 5}, {35003, 5}}});
    {
        Scenario s{"dropped 5 (seq counts IMU)", 60, RandomLatency, {{20000, 5}}};
        s.seq_counts_imu = true;
        scenarios.push_back(s);
    }
    scenarios.push_back({"long stall 60 ms", 60, LongStall});
    {
        Scenario s{"combined, shared seq", 90, Combined, {{10000, 5}, {30000, 1}, {50000, 40}}};
        s.seq_shared = true;
        s.drop_ambiguity = 5;
        scenarios.push_back(s);
    }
    {
        Scenario s{"combined, seq counts IMU", 90, Combined, {{10000, 5}, {30000, 1}, {50000, 400}}};
        s.seq_counts_imu = true;
        scenarios.push_back(s);
    }

    int failed = 0;
    for (const Scenario &sc : scenarios)
    {
        for (uint64_t seed = 1; seed <= 3; seed++)
        {
            if (!Run(sc, seed).ok)
                failed++;
        }
    }
    printf(failed ? "IMU timestamper: %d runs FAILED\n" : "IMU timestamper: all runs passed\n", failed);
    return failed ? 1 : 0;
}
