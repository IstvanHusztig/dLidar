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
    return {"0e1e413-dirty", "livox_mid360s", "Livox Mid-360S", 3};
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
    CHECK(i.version == "0e1e413-dirty" && i.sensor == "livox_mid360s" && i.sensor_model == "Livox Mid-360S" &&
              i.recording_format == 3,
          "info");
    CHECK(kProtocolVersion == 1, "protocol 1");
}
} // namespace

int main()
{
    TestIdle();
    TestRecordingAndSaved();
    TestError();
    TestInfo();
    return FinishTests("device_api");
}
