// bin/dLidar: the composition root of the scanning device. Builds the
// adapters (Unitree L2, recording files, console), wires them to the core
// (ScannerController) and runs one recording (CLI one-shot mode).
//
//   dLidar [--finalize] [--imu-time=auto|sensor|host]
#include "adapters/console/console_events.h"
#include "adapters/storage/file_recording_storage.h"
#include "adapters/unitree_l2/live_l2_source.h"
#include "adapters/unitree_l2/unitree_l2_sensor.h"
#include "app/device_config.h"
#include "app/signals.h"
#include "application/device/event_hub.h"
#include "application/device/scanner_controller.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace
{
// Added to every IMU timestamp. Leave 0 unless the field check
// (docs/imu_lidar_time_alignment.md) measures a constant residual offset.
const int64_t kImuOffsetNs = 0;

std::string OutputDirectory()
{
	const char *linuxUser = getenv("USER");
	return std::string("/home/") + (linuxUser ? linuxUser : "") + "/PointCloudDump/";
}
} // namespace

int main(int argc, char *argv[])
{
	InstallSignalHandlers();

	// Recordings go to numbered files in OutputDirectory(); the L2's packet
	// state rows are its diagnostics (packetstateNNNN.tsv).
	dlidar::FileRecordingStorage storage(OutputDirectory(), dlidar::l2::kPacketStateHeader);

	dlidar::SessionConfig config;
	config.timing.imu_offset_ns = kImuOffsetNs;
	for (int i = 1; i < argc; i++)
	{
		std::string arg = argv[i];
		if (arg == "--finalize")
		{
			// Only convert an interrupted recording in place, no capture.
			return storage.RecoverLeftovers(false) ? 0 : 1;
		}
		else if (arg == "--imu-time=auto")
			config.timing.mode = dlidar::ImuTimeline::Mode::Auto;
		else if (arg == "--imu-time=sensor")
			config.timing.mode = dlidar::ImuTimeline::Mode::Sensor;
		else if (arg == "--imu-time=host")
			config.timing.mode = dlidar::ImuTimeline::Mode::HostAligned;
		else
		{
			std::cerr << "usage: dLidar [--finalize] [--imu-time=auto|sensor|host]" << std::endl;
			return 2;
		}
	}

	if (!kMount.IsProperRotation())
	{
		std::cerr << "The configured mount (" << kMount.Describe() << ") is not a rotation; fix kMount in src/app/device_config.h." << std::endl;
		return 2;
	}
	config.setup.mount = kMount;
	config.target_chunks = kTargetChunks;

	// A recording that was killed or lost power is finalized into its own
	// subdirectory first, so the new recording does not overwrite it.
	if (!storage.RecoverLeftovers(true))
	{
		std::cerr << "Could not finalize the previous recording; fix that first (or run with --finalize)." << std::endl;
		return 1;
	}

	dlidar::LiveL2Config l2;
	l2.lidar_ip = kLidarIp;
	l2.lidar_port = kLidarPort;
	l2.local_ip = kLocalIp;
	l2.local_port = kLocalPort;
	l2.work_mode = kLidarWorkMode;
	std::unique_ptr<dlidar::LiveL2Source> source = dlidar::LiveL2Source::Open(l2);
	if (!source)
		exit(-1);
	dlidar::UnitreeL2Sensor sensor(std::move(source));

	// Output channels. Later: LEDs and the phone link via hub.AddAsync(...).
	dlidar::ConsoleEvents console;
	dlidar::EventHub hub;
	hub.AddSync(console);

	{
		// Input channels: SIGINT/SIGTERM/SIGHUP (the stop flag). Later: buttons
		// and the phone send StartRecording/StopRecording.
		dlidar::ScannerController controller(sensor, storage, hub, config, &StopRequested());
		controller.StartRecording();
		controller.WaitIdle();
	}

	sensor.Stop();

	return 0;
}
