// RecordSession and EventHub with fakes only (no L2, no files): a complete
// recording from FakeSensor into MemoryStorage, output channels that block
// or throw do not change it, the silent-sensor watchdog and the lens
// dirtiness warning interval.
#include "application/capture/record_session.h"
#include "application/device/event_hub.h"
#include "fake_sensor.h"
#include "memory_storage.h"
#include "recording_events.h"
#include "../check.h"

#include <array>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

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

    // The blocking channel is shared with the hub's thread that is stuck in
    // it, so it stays alive after the hub has left that thread behind.
    auto blocking = std::make_shared<BlockingEvents>();
    auto blocking2 = std::make_shared<BlockingEvents>();
    ThrowingEvents throwing_sync;
    RecordingEvents sync_seen;
    auto async_seen = std::make_shared<RecordingEvents>();
    std::vector<SensorChunk> chunks;
    std::chrono::steady_clock::time_point closed;
    auto t0 = std::chrono::steady_clock::now();
    {
        EventHub hub;
        hub.AddSync(throwing_sync);
        hub.AddSync(sync_seen);
        hub.AddAsync(blocking);
        hub.AddAsync(blocking2);
        hub.AddAsync(std::make_shared<ThrowingEvents>());
        hub.AddAsync(async_seen);
        chunks = RecordWith(hub);
        t0 = std::chrono::steady_clock::now();
        CHECK(blocking->Calls() == 1, "blocking channel got %d calls", blocking->Calls());
    } // hub gives the stuck channels 1 s together, then leaves them
    closed = std::chrono::steady_clock::now();
    CHECK(closed - t0 < std::chrono::milliseconds(1800), "two stuck channels took %lld ms",
          (long long)std::chrono::duration_cast<std::chrono::milliseconds>(closed - t0).count());
    blocking->Release();
    blocking2->Release();

    CHECK(SameChunks(chunks, reference), "recording differs with failing channels");
    std::vector<std::string> want{"preparing", "recording", "finalizing", "idle"};
    CHECK(sync_seen.States() == want, "sync channel states");
    CHECK(async_seen->States() == want, "async channel states");
}

// The storage cannot open: the capture stops instead of queueing the whole
// recording in memory, and the recording ends in Error.
void TestStorageOpenFails()
{
    printf("a storage that does not open stops the recording\n");
    FakeSensor sensor;
    sensor.endless = true;
    MemoryStorage storage;
    storage.fail_open = true;
    RecordingEvents events;
    RecordSession session(sensor, storage, events, Config(100000));
    CHECK(!session.Run(StopToken()), "run reported success");
    std::vector<std::string> e = events.Events();
    CHECK(std::count(e.begin(), e.end(), "warning:Fatal Error: Failed to open output streams!") == 1, "warning");
    CHECK(events.States().back() == "error" && storage.closes == 0 && sensor.stopped, "ends in error, sensor stopped");
}

// The storage throws while writing: the capture stops, the files are still
// closed, the recording ends in Error.
void TestStorageThrows()
{
    printf("a storage that throws stops the recording\n");
    FakeSensor sensor;
    sensor.endless = true;
    MemoryStorage storage;
    storage.throw_at_append = 2;
    RecordingEvents events;
    RecordSession session(sensor, storage, events, Config(100000));
    CHECK(!session.Run(StopToken()), "run reported success");
    std::vector<std::string> e = events.Events();
    CHECK(std::count(e.begin(), e.end(), "warning:ERROR: writing the recording failed: disk broke") == 1, "warning");
    CHECK(storage.chunks.size() == 2 && storage.closes == 1, "%zu chunks, %d closes", storage.chunks.size(), storage.closes);
    CHECK(events.States().back() == "error", "ends in error");
}

// The sensor throws in the middle of a recording: no std::terminate; what
// was captured is written, the files are closed, the state is Error.
void TestSensorThrows()
{
    printf("a sensor that throws ends the recording in Error\n");
    FakeSensor sensor;
    sensor.endless = true;
    sensor.on_poll = [](int n) {
        if (n == 200)
            throw std::runtime_error("socket gone");
    };
    MemoryStorage storage;
    RecordingEvents events;
    RecordSession session(sensor, storage, events, Config(100000));
    CHECK(!session.Run(StopToken()), "run reported success");
    std::vector<std::string> e = events.Events();
    CHECK(std::count(e.begin(), e.end(), "warning:ERROR: the recording stopped on an internal error: socket gone") == 1, "warning");
    size_t points = 0;
    for (const SensorChunk &c : storage.chunks)
        points += c.points.size();
    CHECK(storage.chunks.size() == 4 && points == 4 * 14100 && storage.closes == 1 && sensor.stopped,
          "%zu chunks %zu points, %d closes, stopped %d", storage.chunks.size(), points, storage.closes, sensor.stopped);
    CHECK(events.States().back() == "error", "ends in error");
}

// Records Script(), then `tail` (packets without points), each taking 1 ms of
// host time, against a 100 ms watchdog. The watchdog stops the capture: what
// was captured is written, the files are closed, the state is Error.
void CheckWatchdog(const std::function<void(FakeSensor &)> &tail)
{
    FakeSensor sensor;
    Script(sensor);
    tail(sensor);
    sensor.on_poll = [](int n) {
        if (n >= 240)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };
    MemoryStorage storage;
    RecordingEvents events;
    SessionConfig config = Config();
    config.sensor_silence_timeout = std::chrono::milliseconds(100);
    RecordSession session(sensor, storage, events, config);
    CHECK(!session.Run(StopToken()), "run reported success");
    std::vector<std::string> e = events.Events();
    CHECK(std::count(e.begin(), e.end(),
                     "warning:ERROR: no points from the sensor for 0.1 s (power or connection lost, or the sensor "
                     "restarted); the recording was stopped.") == 1,
          "warning");
    size_t points = 0, imu = 0;
    for (const SensorChunk &c : storage.chunks)
    {
        points += c.points.size();
        imu += c.imu_data.size();
    }
    CHECK(storage.chunks.size() == 3 && points == 36000 && imu == 120 && storage.closes == 1 && sensor.stopped,
          "%zu chunks %zu points %zu imu, %d closes, stopped %d", storage.chunks.size(), points, imu, storage.closes,
          sensor.stopped);
    CHECK(sensor.Pending() > 0, "stopped by the watchdog, not by the end of the packets");
    std::vector<std::string> states = events.States();
    CHECK((states == std::vector<std::string>{"preparing", "recording", "finalizing", "error"}), "states %zu", states.size());
}

void TestSensorSilent()
{
    printf("a sensor that goes silent ends the recording in Error\n");
    CheckWatchdog([](FakeSensor &s) {
        for (int i = 0; i < 2000; i++)
            s.AddNothing();
    });
}

// What an L2 that restarted sends: IMU packets on its reset clock and point
// packets without points. They do not keep the recording alive.
void TestSensorRestarted()
{
    printf("a sensor that restarts and sends no points ends the recording in Error\n");
    CheckWatchdog([](FakeSensor &s) {
        for (int i = 0; i < 1000; i++)
        {
            int64_t t = 5000000000LL + i * 1000000LL;
            s.AddImu(t, t, (uint32_t)i);
            s.AddPoints(t + 10, t + 10, (uint32_t)i, 0);
        }
    });
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
// The timing report names the sensor and, if the sensor gives it, the IMU
// position mapped through the mount (format 3); without one the key is absent.
void TestRecordingIdentity()
{
    printf("the timing report names the sensor and its IMU position\n");
    for (bool with_position : {false, true})
    {
        FakeSensor sensor;
        sensor.versions.sensor_model = "fake_sensor";
        if (with_position)
            sensor.versions.imu_position_m = std::array<float, 3>{0.011f, 0.02329f, -0.04412f};
        Script(sensor);
        MemoryStorage storage;
        RecordingEvents events;
        SessionConfig config = Config();
        config.setup.mount = AxisMap{Axis::PosZ, Axis::PosY, Axis::NegX}; // front = sensor +Z, up = sensor -X
        RecordSession session(sensor, storage, events, config);
        CHECK(session.Run(StopToken()), "run failed");
        std::string report = storage.chunks.empty() ? std::string() : storage.chunks.back().timing_report;
        const std::string head = "{\n  \"recording_format_version\": 3,\n  \"sensor_model\": \"fake_sensor\",\n";
        CHECK(report.compare(0, head.size(), head) == 0, "%s", report.substr(0, 120).c_str());
        bool has = report.find("\"imu_position_m\"") != std::string::npos;
        CHECK(has == with_position, "imu_position_m present %d, expected %d", has, with_position);
        if (with_position)
            CHECK(report.find("\"imu_position_m\": [-0.04412, 0.02329, -0.01100],\n") != std::string::npos, "%s",
                  report.substr(0, 300).c_str());
    }
}
} // namespace

int main()
{
    TestFullRecording();
    TestFailingChannels();
    TestStorageOpenFails();
    TestStorageThrows();
    TestSensorThrows();
    TestSensorSilent();
    TestSensorRestarted();
    TestLensMonitor();
    TestRecordingIdentity();
    return FinishTests("Record session");
}
