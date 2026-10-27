// bin/dLidar: the composition root of the scanning device. Reads the device
// settings, builds the adapters (the chosen sensor, recording files, push
// button, status LED, console) and wires them to the core (ScannerController).
//
// Service mode (the default, started by systemd): waits for the push button;
// a short press starts a recording, the next one saves it, a long press saves
// and powers down. A signal saves a running recording and exits. --once: one
// recording at once, ended by a signal (tests over SSH). --finalize: only
// finish a leftover recording.
//
//   dLidar [--config PATH] [--once | --finalize] [--imu-time=auto|sensor|host]
#include "adapters/console/console_events.h"
#include "adapters/gpio/button_input.h"
#include "adapters/gpio/gpio_line.h"
#include "adapters/gpio/status_led.h"
#include "adapters/storage/file_recording_storage.h"
#include "app/button_commands.h"
#include "app/power.h"
#include "app/sensor_factory.h"
#include "app/settings.h"
#include "app/signals.h"
#include "application/device/event_hub.h"
#include "application/device/scanner_controller.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace
{
// Added to every IMU timestamp. Leave 0 unless the field check
// (docs/imu_lidar_time_alignment.md) measures a constant residual offset.
const int64_t kImuOffsetNs = 0;

const char *const kUsage = "usage: dLidar [--config PATH] [--once | --finalize] [--imu-time=auto|sensor|host]";

std::string OutputDirectory()
{
	const char *linuxUser = getenv("USER");
	return std::string("/home/") + (linuxUser ? linuxUser : "") + "/PointCloudDump/";
}
} // namespace

int main(int argc, char *argv[])
{
	InstallSignalHandlers();

	std::string config_path;
	bool finalize = false, once = false;
	std::optional<dlidar::ImuTimeline::Mode> imu_time;
	for (int i = 1; i < argc; i++)
	{
		std::string arg = argv[i];
		if (arg == "--config" && i + 1 < argc)
			config_path = argv[++i];
		else if (arg.compare(0, 9, "--config=") == 0)
			config_path = arg.substr(9);
		else if (arg == "--finalize")
			finalize = true;
		else if (arg == "--once")
			once = true;
		else if (arg == "--imu-time=auto")
			imu_time = dlidar::ImuTimeline::Mode::Auto;
		else if (arg == "--imu-time=sensor")
			imu_time = dlidar::ImuTimeline::Mode::Sensor;
		else if (arg == "--imu-time=host")
			imu_time = dlidar::ImuTimeline::Mode::HostAligned;
		else
		{
			std::cerr << kUsage << std::endl;
			return 2;
		}
	}
	if (once && finalize)
	{
		std::cerr << kUsage << std::endl;
		return 2;
	}

	// The settings come first: a bad file stops before anything is touched.
	DeviceSettings settings = DefaultSettings();
	std::string error;
	bool explicit_path = !config_path.empty();
	if (!LoadSettings(explicit_path ? config_path : DefaultSettingsPath(), explicit_path, settings, error))
	{
		std::cerr << error << std::endl;
		return 2;
	}
	std::cerr << "settings: " << settings.source << " (sensor " << SensorKindName(settings.sensor) << ")" << std::endl;

	// Recordings go to numbered files in OutputDirectory(), described by the
	// chosen sensor's profile (its diagnostics rows go to packetstateNNNN.tsv).
	dlidar::FileRecordingStorage storage(OutputDirectory(), ProfileFor(settings), AllProfiles());
	if (finalize)
	{
		// Only convert an interrupted recording in place, no capture.
		return storage.RecoverLeftovers(false) ? 0 : 1;
	}

	dlidar::SessionConfig config;
	config.timing.imu_offset_ns = kImuOffsetNs;
	config.timing.mode = imu_time ? *imu_time : settings.imu_time;
	config.setup.mount = settings.Mount(); // a proper rotation (checked by the settings)
	config.target_chunks = 0; // no limit: a recording ends on a stop, a failure or low disk space

	// A recording that was killed or lost power is finalized into its own
	// subdirectory first, so the new recording does not overwrite it.
	if (!storage.RecoverLeftovers(true))
	{
		std::cerr << "Could not finalize the previous recording; fix that first (or run with --finalize)." << std::endl;
		return 1;
	}

	// The push button (service mode): a problem with it stops here, before
	// the sensor is touched.
	std::unique_ptr<dlidar::gpio::GpioLine> button_line;
	if (!once)
	{
		button_line = dlidar::gpio::GpioLine::OpenButton(settings.gpio.button_gpio, error);
		if (!button_line)
		{
			std::cerr << "Cannot use the push button on GPIO " << settings.gpio.button_gpio << " ([button] gpio): " << error
					  << std::endl;
			return 2;
		}
	}
	// The status LED is optional: without it the recordings still work.
	std::unique_ptr<dlidar::gpio::GpioLine> led_line = dlidar::gpio::GpioLine::OpenLed(settings.gpio.led_gpio, error);
	if (!led_line)
		std::cerr << "WARNING: no status LED on GPIO " << settings.gpio.led_gpio << " ([led] gpio): " << error << std::endl;

	std::unique_ptr<dlidar::LidarSensor> sensor = OpenSensor(settings);
	if (!sensor)
		exit(-1);

	// Output channels: the console, the status LED (own thread, never delays
	// a recording). Later: the phone link via hub.AddAsync(...).
	dlidar::ConsoleEvents console;
	dlidar::EventHub hub;
	hub.AddSync(console);
	if (led_line)
		hub.AddAsync(std::make_shared<dlidar::gpio::StatusLed>(std::move(led_line)));

	// A stop during recovery or the sensor set-up: no recording (it would
	// leave an empty file pair that dMapper reads).
	if (StopRequested())
	{
		sensor->Stop();
		return 0;
	}

	{
		// Input channels: SIGINT/SIGTERM/SIGHUP (the stop flag, which also
		// ends a running recording), the push button. Later: the phone.
		dlidar::ScannerController controller(*sensor, storage, hub, config, &StopRequested());
		if (once)
		{
			controller.StartRecording();
			controller.WaitIdle();
		}
		else
		{
			ButtonCommands commands(controller, PowerOff);
			dlidar::gpio::ButtonInput button(
				std::move(button_line), [&] { commands.OnShort(); }, [&] { commands.OnLong(); });
			std::cout << "Ready: press the button (GPIO " << settings.gpio.button_gpio
					  << ") to start a recording, again to save it; hold it 5 s to power down." << std::endl;
			while (!StopRequested())
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			controller.StopRecording();
			controller.WaitIdle();
		}
	}

	sensor->Stop();

	return 0;
}
