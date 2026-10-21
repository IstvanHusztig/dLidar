// RecordSession and EventHub with fakes only (no L2, no files): a complete
// recording from FakeSensor into MemoryStorage, output channels that block
// or throw do not change it, and the lens dirtiness warning interval.
#include "application/capture/record_session.h"
#include "application/device/event_hub.h"
#include "fake_sensor.h"
#include "memory_storage.h"
#include "recording_events.h"
#include "../check.h"

#include <cstring>

using namespace dlidar;
using namespace dlidar::test;

namespace
{
const int64_t kS0 = 1790000000000000000LL;

SessionConfig Config(int target_chunks = 1000)
{
    SessionConfig c;
    c.timing.mode = ImuTimeline::Mode::Sensor;
    c.timing.log = false;
    c.timing.host.log = false;
    c.target_chunks = target_chunks;
    return c;
}

// 120 point packets of 300 points (36000 points: two full chunks and a rest)
// with an IMU sample before each.
void Script(FakeSensor &s)
{
    for (int i = 0; i < 120; i++)
    {
        int64_t t = i * 3000000LL;
        s.AddImu(10 + t, kS0 + t + 2000, (uint32_t)i, 9.80665f, 0.01f * i);
        s.AddPoints(20 + t, kS0 + t + 1000, (uint32_t)i, 300);
    }
}

bool SameChunks(const std::vector<SensorChunk> &a, const std::vector<SensorChunk> &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
    {
        const SensorChunk &x = a[i], &y = b[i];
        if (x.chunk_index != y.chunk_index || x.points.size() != y.points.size() || x.imu_data.size() != y.imu_data.size() ||
            x.packets.size() != y.packets.size() || x.timing_report != y.timing_report ||
            x.packet_state_rows != y.packet_state_rows)
            return false;
        if (memcmp(x.points.data(), y.points.data(), x.points.size() * sizeof(PointRecord)) ||
            memcmp(x.imu_data.data(), y.imu_data.data(), x.imu_data.size() * sizeof(ImuSample)) ||
            memcmp(x.packets.data(), y.packets.data(), x.packets.size() * sizeof(PacketTiming)))
            return false;
    }
    return true;
}

void TestFullRecording()
{
    printf("a full recording from FakeSensor into MemoryStorage\n");
    FakeSensor sensor;
    sensor.with_diagnostics = true;
    Script(sensor);
    MemoryStorage storage;
    RecordingEvents events;
    RecordSession session(sensor, storage, events, Config());
    CHECK(session.Run(StopToken()), "run failed");

    CHECK(sensor.prepared && sensor.stopped, "sensor prepared %d stopped %d", sensor.prepared, sensor.stopped);
    CHECK(storage.opens == 1 && storage.closes == 1, "storage opened %d closed %d", storage.opens, storage.closes);
    size_t points = 0, packets = 0, imu = 0, rows = 0;
    for (const SensorChunk &c : storage.chunks)
    {
        points += c.points.size();
        packets += c.packets.size();
        imu += c.imu_data.size();
        rows += std::count(c.packet_state_rows.begin(), c.packet_state_rows.end(), '\n');
    }
    CHECK(storage.chunks.size() == 3 && points == 36000 && packets == 240 && rows == 120, "%zu chunks, %zu points, %zu packets, %zu rows",
          storage.chunks.size(), points, packets, rows);
    CHECK(imu == 120, "%zu IMU samples", imu);
    CHECK(!storage.chunks.empty() && !storage.chunks.back().timing_report.empty(), "final chunk has the timing report");
    std::vector<std::string> states = events.States();
    CHECK((states == std::vector<std::string>{"preparing", "recording", "finalizing", "idle"}), "states %zu", states.size());
    std::vector<std::string> all = events.Events();
    CHECK(std::count(all.begin(), all.end(), "summary") == 1 && std::count(all.begin(), all.end(), "progress:2") == 1,
          "summary and progress events");
}

std::vector<SensorChunk> RecordWith(EventHub &hub)
{
    FakeSensor sensor;
    sensor.with_diagnostics = true;
    Script(sensor);
    MemoryStorage storage;
    RecordSession session(sensor, storage, hub, Config());
    session.Run(StopToken());
    return storage.chunks;
}

// A channel that throws and one that blocks (registered async) do not delay,
// stop or change the recording; the others still get every event in order.
void TestFailingChannels()
{
    printf("blocking and throwing output channels do not affect a recording\n");
    std::vector<SensorChunk> reference;
    {
        EventHub hub;
        reference = RecordWith(hub);
    }

    static BlockingEvents blocking; // outlives the hub's detached thread
    ThrowingEvents throwing_sync, throwing_async;
    RecordingEvents sync_seen, async_seen;
    std::vector<SensorChunk> chunks;
    {
        EventHub hub;
        hub.AddSync(throwing_sync);
        hub.AddSync(sync_seen);
        hub.AddAsync(blocking);
        hub.AddAsync(throwing_async);
        hub.AddAsync(async_seen);
        chunks = RecordWith(hub);
        CHECK(blocking.Calls() == 1, "blocking channel got %d calls", blocking.Calls());
    } // hub gives the stuck channel 1 s, then leaves it
    blocking.Release();

    CHECK(SameChunks(chunks, reference), "recording differs with failing channels");
    std::vector<std::string> want{"preparing", "recording", "finalizing", "idle"};
    CHECK(sync_seen.States() == want, "sync channel states");
    CHECK(async_seen.States() == want, "async channel states");
}

void TestLensMonitor()
{
    printf("lens dirtiness warning at most every 10 s\n");
    FakeSensor sensor;
    RecordingEvents events;
    LensMonitor lens;
    sensor.dirtiness = 3.0f;
    lens.Check(sensor, 0, events);
    sensor.dirtiness = 7.5f;
    lens.Check(sensor, 1000000000LL, events); // within 10 s of the first check: skipped
    lens.Check(sensor, 10000000000LL, events);
    lens.Check(sensor, 15000000000LL, events);
    sensor.dirtiness.reset();
    lens.Check(sensor, 30000000000LL, events);
    std::vector<std::string> e = events.Events();
    CHECK(e.size() == 1 && e[0] == "warning:WARNING: lens dirty percentage = 7.5 % (threshold 5 %) - clean the sensor housing, points may be degraded.",
          "%zu warnings: %s", e.size(), e.empty() ? "" : e[0].c_str());
}
} // namespace

int main()
{
    TestFullRecording();
    TestFailingChannels();
    TestLensMonitor();
    return FinishTests("Record session");
}
