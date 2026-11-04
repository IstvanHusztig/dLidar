// DeviceApi (openspec phone-link-core, specs/phone-api: status.get and
// info.get): the status in each state through ScannerController, free space
// known and unknown, the info fields. Fakes only.
#include "application/device/scanner_controller.h"
#include "application/remote/device_api.h"
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
SessionConfig Config()
{
    SessionConfig c;
    c.timing.mode = ImuTimeline::Mode::Sensor;
    c.timing.log = false;
    c.timing.host.log = false;
    return c;
}

template <typename Cond>
bool WaitFor(Cond cond)
{
    for (int i = 0; i < 10000 && !cond(); i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return cond();
}

struct FixedSpace : StorageSpace
{
    std::optional<uint64_t> free;
    std::optional<uint64_t> FreeBytes() const override { return free; }
};

DeviceInfo Info()
{
    return {"0e1e413-dirty", "livox_mid360s", "livox_mid360s", 3};
}

void TestIdle()
{
    printf("status before the first recording\n");
    FakeSensor sensor;
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config());
    FixedSpace space;
    space.free = 123456789ull;
    DeviceApi api(c, &space, Info());

    StatusView v = api.Status();
    CHECK(v.status.state == DeviceState::Idle && v.status.last_ok, "idle, last_ok true");
    CHECK(v.status.chunks == 0 && v.status.points == 0, "no counts");
    CHECK(v.status.location == storage.Location(), "location %s", v.status.location.c_str());
    CHECK(v.free_bytes && *v.free_bytes == 123456789ull, "free bytes known");

    space.free.reset();
    CHECK(!api.Status().free_bytes, "free bytes unknown");
    DeviceApi no_space(c, nullptr, Info());
    CHECK(!no_space.Status().free_bytes, "no StorageSpace: unknown");
}

void TestRecordingAndSaved()
{
    printf("status while recording and after the save\n");
    FakeSensor sensor;
    sensor.endless = true;
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config());
    DeviceApi api(c, nullptr, Info());

    CHECK(c.StartRecording(), "start");
    CHECK(WaitFor([&] { return api.Status().status.state == DeviceState::Recording && api.Status().status.points > 0; }),
          "recording with points");
    c.StopRecording();
    c.WaitIdle();
    StatusView v = api.Status();
    CHECK(v.status.state == DeviceState::Idle && v.status.last_ok, "saved: idle, last_ok");
    CHECK(v.status.chunks > 0 && v.status.points > 0, "counts of the last recording kept");
}

void TestError()
{
    printf("status after a failed recording\n");
    FakeSensor sensor;
    sensor.prepare_ok = false;
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config());
    DeviceApi api(c, nullptr, Info());

    CHECK(c.StartRecording(), "start");
    c.WaitIdle();
    StatusView v = api.Status();
    CHECK(v.status.state == DeviceState::Error && !v.status.last_ok, "error, last_ok false");
}

void TestInfo()
{
    printf("info fields\n");
    FakeSensor sensor;
    MemoryStorage storage;
    RecordingEvents events;
    ScannerController c(sensor, storage, events, Config());
    DeviceApi api(c, nullptr, Info());
    const DeviceInfo &i = api.Info();
    CHECK(i.version == "0e1e413-dirty" && i.sensor == "livox_mid360s" && i.sensor_model == "livox_mid360s" &&
              i.recording_format == 3,
          "info");
    CHECK(kProtocolVersion == 1, "protocol 1");
}
// Start/Stop decided per state, with a fake control (design D1).
struct FakeControl : DeviceControl
{
    DeviceState state = DeviceState::Idle;
    int starts = 0, stops = 0;
    DeviceStatus Status() const override
    {
        DeviceStatus s;
        s.state = state;
        return s;
    }
    bool StartRecording(DeviceState *seen) override
    {
        if (seen)
            *seen = state;
        if (state != DeviceState::Idle && state != DeviceState::Error)
            return false;
        starts++;
        state = DeviceState::Preparing;
        return true;
    }
    void StopRecording() override { stops++; }
};

void TestCommandsPerState()
{
    printf("start and stop in every state\n");
    struct Want
    {
        DeviceState state;
        bool start_accepted, start_busy, stop_accepted;
    } cases[] = {
        {DeviceState::Idle, true, false, false},       {DeviceState::Preparing, false, false, true},
        {DeviceState::Recording, false, false, true},  {DeviceState::Finalizing, false, true, false},
        {DeviceState::Error, true, false, false},
    };
    for (const Want &w : cases)
    {
        FakeControl c;
        c.state = w.state;
        DeviceApi api(c, nullptr, Info());
        CommandResult r = api.Start();
        CHECK(r.accepted == w.start_accepted && r.busy == w.start_busy && r.state_seen == w.state, "start in %s",
              StateName(w.state));
        CHECK(c.starts == (w.start_accepted ? 1 : 0), "start called in %s", StateName(w.state));

        FakeControl d;
        d.state = w.state;
        DeviceApi api2(d, nullptr, Info());
        r = api2.Stop();
        CHECK(r.accepted == w.stop_accepted && !r.busy && r.state_seen == w.state, "stop in %s", StateName(w.state));
        CHECK(d.stops == (w.stop_accepted ? 1 : 0), "stop called in %s", StateName(w.state));
    }
}

void TestCommandsWithController()
{
    printf("start and stop through ScannerController\n");
    {
        // Start from Idle, nothing to do while recording, stop saves it.
        FakeSensor sensor;
        sensor.endless = true;
        MemoryStorage storage;
        RecordingEvents events;
        ScannerController c(sensor, storage, events, Config());
        DeviceApi api(c, nullptr, Info());
        CommandResult r = api.Start();
        CHECK(r.accepted && r.state_seen == DeviceState::Idle, "start from idle");
        CHECK(WaitFor([&] { return c.Status().state == DeviceState::Recording; }), "recording");
        r = api.Start();
        CHECK(!r.accepted && !r.busy && r.state_seen == DeviceState::Recording, "start while recording");
        r = api.Stop();
        CHECK(r.accepted && r.state_seen == DeviceState::Recording, "stop while recording");
        c.WaitIdle();
        CHECK(c.Status().state == DeviceState::Idle && storage.closes == 1, "saved");
        r = api.Stop();
        CHECK(!r.accepted && r.state_seen == DeviceState::Idle, "stop while idle");
    }
    {
        // Busy while the last recording is being saved (a slow disk).
        FakeSensor sensor;
        sensor.endless = true;
        MemoryStorage storage;
        storage.append_delay_ms = 300;
        RecordingEvents events;
        ScannerController c(sensor, storage, events, Config());
        DeviceApi api(c, nullptr, Info());
        CHECK(api.Start().accepted, "start");
        CHECK(WaitFor([&] { return c.Status().state == DeviceState::Recording && c.Status().chunks > 0; }),
              "recording");
        api.Stop();
        CHECK(WaitFor([&] { return c.Status().state == DeviceState::Finalizing; }), "finalizing");
        CommandResult r = api.Start();
        CHECK(!r.accepted && r.busy && r.state_seen == DeviceState::Finalizing, "busy while saving");
        r = api.Stop();
        CHECK(!r.accepted && !r.busy && r.state_seen == DeviceState::Finalizing, "stop while saving: nothing to do");
        c.WaitIdle();
        CHECK(storage.opens == 1, "only the one recording");
    }
    {
        // A stop during the start-up writes no files.
        FakeSensor sensor;
        sensor.endless = true;
        std::mutex m;
        std::condition_variable cv;
        bool go = false;
        sensor.on_prepare = [&] {
            std::unique_lock<std::mutex> l(m);
            cv.wait(l, [&] { return go; });
        };
        MemoryStorage storage;
        RecordingEvents events;
        ScannerController c(sensor, storage, events, Config());
        DeviceApi api(c, nullptr, Info());
        CHECK(api.Start().accepted, "start");
        CommandResult r = api.Stop();
        CHECK(r.accepted && r.state_seen == DeviceState::Preparing, "stop while preparing");
        {
            std::lock_guard<std::mutex> l(m);
            go = true;
        }
        cv.notify_all();
        c.WaitIdle();
        CHECK(storage.opens == 0 && c.Status().state == DeviceState::Idle, "no files (%d opens)", storage.opens);
    }
    {
        // Start after a failure.
        FakeSensor sensor;
        sensor.prepare_ok = false;
        MemoryStorage storage;
        RecordingEvents events;
        ScannerController c(sensor, storage, events, Config());
        DeviceApi api(c, nullptr, Info());
        api.Start();
        c.WaitIdle();
        CHECK(c.Status().state == DeviceState::Error, "failed");
        CommandResult r = api.Start();
        CHECK(r.accepted && r.state_seen == DeviceState::Error, "start from error");
        c.WaitIdle();
    }
}
} // namespace

int main()
{
    TestIdle();
    TestRecordingAndSaved();
    TestError();
    TestCommandsPerState();
    TestCommandsWithController();
    TestInfo();
    return FinishTests("device_api");
}
