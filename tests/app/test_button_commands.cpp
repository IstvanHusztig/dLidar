// ButtonCommands with a real ScannerController, FakeSensor and MemoryStorage:
// presses start and stop recordings, a press while saving is ignored, a long
// press saves before powering down, and a refused power-down keeps running.
#include "app/button_commands.h"
#include "../application/fake_sensor.h"
#include "../application/memory_storage.h"
#include "../application/recording_events.h"
#include "../check.h"

#include <atomic>
#include <chrono>
#include <thread>

using namespace dlidar;
using namespace dlidar::test;
using namespace std::chrono_literals;

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

void WaitFor(ScannerController &c, DeviceState s)
{
	for (int i = 0; i < 300 && c.Status().state != s; i++)
		std::this_thread::sleep_for(10ms);
}

void TestStartStop()
{
	FakeSensor sensor;
	sensor.endless = true;
	sensor.on_poll = [](int) { std::this_thread::sleep_for(std::chrono::microseconds(100)); };
	MemoryStorage storage;
	RecordingEvents events;
	ScannerController controller(sensor, storage, events, Config());
	int power_offs = 0;
	ButtonCommands button(controller, [&] { power_offs++; return true; });

	for (int i = 0; i < 2; i++)
	{
		button.OnShort();
		WaitFor(controller, DeviceState::Recording);
		std::this_thread::sleep_for(100ms);
		button.OnShort();
		controller.WaitIdle();
	}
	CHECK(storage.opens == 2 && storage.closes == 2 && controller.Status().state == DeviceState::Idle && power_offs == 0,
		  "two recordings: %d opens, %d closes", storage.opens, storage.closes);
}

void TestPressWhileSaving()
{
	FakeSensor sensor;
	sensor.endless = true;
	sensor.on_poll = [](int) { std::this_thread::sleep_for(std::chrono::microseconds(100)); };
	MemoryStorage storage;
	storage.append_delay_ms = 50; // a slow disk: saving takes a while
	RecordingEvents events;
	ScannerController controller(sensor, storage, events, Config());
	ButtonCommands button(controller, [] { return true; });
	button.OnShort();
	WaitFor(controller, DeviceState::Recording);
	std::this_thread::sleep_for(300ms);
	button.OnShort();
	WaitFor(controller, DeviceState::Finalizing);
	bool finalizing = controller.Status().state == DeviceState::Finalizing;
	button.OnShort(); // ignored
	controller.WaitIdle();
	std::this_thread::sleep_for(100ms);
	CHECK(finalizing && storage.opens == 1 && controller.Status().state == DeviceState::Idle,
		  "the press while saving started nothing: %d opens", storage.opens);
}

void TestLongPress()
{
	FakeSensor sensor;
	sensor.endless = true;
	sensor.on_poll = [](int) { std::this_thread::sleep_for(std::chrono::microseconds(100)); };
	MemoryStorage storage;
	RecordingEvents events;
	ScannerController controller(sensor, storage, events, Config());
	std::atomic<int> closes_at_power_off{-1};
	bool refuse = false;
	ButtonCommands button(controller, [&] {
		closes_at_power_off = storage.closes;
		return !refuse;
	});
	button.OnShort();
	WaitFor(controller, DeviceState::Recording);
	std::this_thread::sleep_for(100ms);
	button.OnLong();
	CHECK(closes_at_power_off == 1 && controller.Status().state == DeviceState::Idle,
		  "the recording was saved before the power-down (%d closes then)", closes_at_power_off.load());

	refuse = true;
	button.OnLong(); // refused: a warning, and the controller still takes commands
	button.OnShort();
	WaitFor(controller, DeviceState::Recording);
	CHECK(controller.Status().state == DeviceState::Recording, "still running after a refused power-down");
	controller.StopRecording();
	controller.WaitIdle();
}
} // namespace

int main()
{
	TestStartStop();
	TestPressWhileSaving();
	TestLongPress();
	return FinishTests("Button commands");
}
