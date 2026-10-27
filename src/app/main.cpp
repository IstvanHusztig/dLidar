// bin/dLidar: the composition root of the scanning device. Reads the device
// settings, builds the adapters (the chosen sensor, recording files, console),
// wires them to the core (ScannerController) and runs one recording (CLI
// one-shot mode).
//
//   dLidar [--config PATH] [--finalize] [--imu-time=auto|sensor|host]
#include "adapters/console/console_events.h"
#include "adapters/storage/file_recording_storage.h"
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

namespace
{
// Added to every IMU timestamp. Leave 0 unless the field check
// (docs/imu_lidar_time_alignment.md) measures a constant residual offset.
const int64_t kImuOffsetNs = 0;

const char *const kUsage = "usage: dLidar [--config PATH] [--finalize] [--imu-time=auto|sensor|host]";

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
	bool finalize = false;
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
	config.target_chunks = settings.target_chunks;

	// A recording that was killed or lost power is finalized into its own
	// subdirectory first, so the new recording does not overwrite it.
	if (!storage.RecoverLeftovers(true))
	{
		std::cerr << "Could not finalize the previous recording; fix that first (or run with --finalize)." << std::endl;
		return 1;
	}

	std::unique_ptr<dlidar::LidarSensor> sensor = OpenSensor(settings);
	if (!sensor)
		exit(-1);

	// Output channels. Later: LEDs and the phone link via hub.AddAsync(...).
	dlidar::ConsoleEvents console;
	dlidar::EventHub hub;
	hub.AddSync(console);

	// A stop during recovery or the sensor set-up: no recording (it would
	// leave an empty file pair that dMapper reads).
	if (StopRequested())
	{
		sensor->Stop();
		return 0;
	}

	{
		// Input channels: SIGINT/SIGTERM/SIGHUP (the stop flag). Later: buttons
		// and the phone send StartRecording/StopRecording.
		dlidar::ScannerController controller(*sensor, storage, hub, config, &StopRequested());
		controller.StartRecording();
		controller.WaitIdle();
	}

	sensor->Stop();

	return 0;
}
