// ScannerController against the scenarios of specs/device-control: one
// recording at a time, commands from any input channel, stop from a signal
// flag, several output channels, sensor start failure. Fakes only.
#include "application/device/event_hub.h"
#include "application/device/scanner_controller.h"
#include "fake_sensor.h"
#include "memory_storage.h"
#include "recording_events.h"
#include "../check.h"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

using namespace dlidar;
using namespace dlidar::test;

namespace
{
SessionConfig Config(int target_chunks)
{
    SessionConfig c;
    c.timing.mode = ImuTimeline::Mode::Sensor;
    c.timing.log = false;
    c.timing.host.log = false;
    c.target_chunks = target_chunks;
    return c;
}

// Waits up to 10 s for `cond`.
template <typename Cond>
bool WaitFor(Cond cond)
{
    for (int i = 0; i < 10000 && !cond(); i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return cond();
}

// Holds the sensor's start-up until Open() is called.
struct Gate
{
    std::mutex m;
    std::condition_variable cv;
    bool open = false;
    void Wait()
    {
        std::unique_lock<std::mutex> l(m);
        cv.wait(l, [&] { return open; });
    }
    void Open()
    {
        {
            std::lock_guard<std::mutex> l(m);
            open = true;
        }
        cv.notify_all();
    }
};

const std::vector<std::string> kFullCycle{"preparing", "recording", "finalizing", "idle"};

// Start while Preparing or Recording is rejected with the current state, and
// the running recording goes on unaffected.
void TestStartWhileBusy()
{
    printf("start while a recording runs is rejected\n");
    FakeSensor sensor;
    sensor.endless = true;
    Gate gate;
    sensor.on_prepare = [&] { gate.Wait(); };
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config(100000));

    DeviceState seen;
    CHECK(c.StartRecording(&seen) && seen == DeviceState::Idle, "first start");
    CHECK(WaitFor([&] { return c.Status().state == DeviceState::Preparing; }), "not preparing");
    CHECK(!c.StartRecording(&seen) && seen == DeviceState::Preparing, "start while preparing: %s", StateName(seen));
    gate.Open();
    CHECK(WaitFor([&] { return c.Status().chunks >= 2; }), "no chunks");
    CHECK(!c.StartRecording(&seen) && seen == DeviceState::Recording, "start while recording: %s", StateName(seen));
    c.StopRecording();
    c.WaitIdle();
    CHECK(storage.opens == 1 && storage.closes == 1, "one recording: opened %d closed %d", storage.opens, storage.closes);
    CHECK(events.States() == kFullCycle, "states");
    CHECK(c.Status().state == DeviceState::Idle && c.Status().last_ok, "ends idle and complete");
}

// The recording reaches its length: Finalizing, then Idle, complete.
void TestEndsByItself()
{
    printf("a recording that reaches its length ends by itself\n");
    FakeSensor sensor;
    sensor.endless = true;
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config(3));
    c.StartRecording();
    c.WaitIdle();
    CHECK(storage.chunks.size() == 4, "%zu chunks (3 + the final one)", storage.chunks.size());
    CHECK(events.States() == kFullCycle && c.Status().last_ok, "states / complete");
}

// A stop from the signal flag or from another input channel finalizes the
// recording like the chunk limit does; afterwards the next start works.
void TestStopFromAnyChannel()
{
    printf("stop from a signal or another input channel finalizes\n");
    FakeSensor sensor;
    sensor.endless = true;
    MemoryStorage storage;
    RecordingEvents events;
    std::atomic<bool> signal_flag{false};
    ScannerController c(sensor, storage, events, Config(100000), &signal_flag);

    c.StartRecording();
    CHECK(WaitFor([&] { return c.Status().chunks >= 1; }), "no chunks");
    signal_flag = true; // what SIGINT does
    c.WaitIdle();
    CHECK(storage.closes == 1 && c.Status().last_ok, "signal stop finalized");
    signal_flag = false;

    // A fake input channel (e.g. a button) on its own thread.
    std::thread button([&] {
        c.StartRecording();
        WaitFor([&] { return c.Status().chunks >= 1; });
        c.StopRecording();
    });
    button.join();
    c.WaitIdle();
    CHECK(storage.opens == 2 && storage.closes == 2 && c.Status().state == DeviceState::Idle, "button recording finalized");
    std::vector<std::string> twice = kFullCycle;
    twice.insert(twice.end(), kFullCycle.begin(), kFullCycle.end());
    CHECK(events.States() == twice, "two full cycles");
}

// Several output channels each get the state changes in order.
void TestSeveralOutputs()
{
    printf("every output channel gets the states in order\n");
    FakeSensor sensor;
    sensor.endless = true;
    MemoryStorage storage;
    RecordingEvents a, b;
    {
        EventHub hub;
        hub.AddSync(a);
        hub.AddAsync(b);
        ScannerController c(sensor, storage, hub, Config(2));
        c.StartRecording();
        c.WaitIdle();
    }
    CHECK(a.States() == kFullCycle && b.States() == kFullCycle, "a %zu b %zu states", a.States().size(), b.States().size());
}

// The sensor does not start: Error, the storage is still closed cleanly, and
// a new start is accepted.
void TestSensorFailure()
{
    printf("a sensor that does not start ends in Error\n");
    FakeSensor sensor;
    sensor.prepare_ok = false;
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config(2));
    c.StartRecording();
    c.WaitIdle();
    CHECK(c.Status().state == DeviceState::Error && !c.Status().last_ok, "state %s", StateName(c.Status().state));
    CHECK(!sensor.stopped && storage.closes == 1, "sensor stopped %d, storage closed %d", sensor.stopped, storage.closes);
    std::vector<std::string> ev = events.Events();
    CHECK(std::count(ev.begin(), ev.end(), "warning:ERROR: the sensor did not start.") == 1, "warning");
    sensor.prepare_ok = true;
    sensor.endless = true;
    CHECK(c.StartRecording(), "start after Error");
    c.WaitIdle();
    CHECK(c.Status().state == DeviceState::Idle && c.Status().last_ok, "second recording");
}
} // namespace

int main()
{
    TestStartWhileBusy();
    TestEndsByItself();
    TestStopFromAnyChannel();
    TestSeveralOutputs();
    TestSensorFailure();
    return FinishTests("Device control");
}
