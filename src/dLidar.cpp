#include "pcd_manager.h"
#include "app/device_config.h"
#include "adapters/unitree_l2/live_l2_source.h"
#include "adapters/unitree_l2/unitree_l2_sensor.h"
#include "adapters/storage/file_recording_storage.h"

// Added to every IMU timestamp. Leave 0 unless the field check
// (docs/imu_lidar_time_alignment.md) measures a constant residual offset.
const int64_t kImuOffsetNs = 0;

int main(int argc, char *argv[])
{
	InstallSignalHandlers();

	// Recordings go to numbered files in OutputDirectory(); the L2's packet
	// state rows are its diagnostics (packetstateNNNN.tsv).
	dlidar::FileRecordingStorage storage(OutputDirectory(), dlidar::l2::kPacketStateHeader);

	ImuTimeline::Config timing_config;
	timing_config.imu_offset_ns = kImuOffsetNs;
	for (int i = 1; i < argc; i++)
	{
		std::string arg = argv[i];
		if (arg == "--finalize")
		{
			// Only convert an interrupted recording in place, no capture.
			return storage.RecoverLeftovers(false) ? 0 : 1;
		}
		else if (arg == "--imu-time=auto")
			timing_config.mode = ImuTimeline::Mode::Auto;
		else if (arg == "--imu-time=sensor")
			timing_config.mode = ImuTimeline::Mode::Sensor;
		else if (arg == "--imu-time=host")
			timing_config.mode = ImuTimeline::Mode::HostAligned;
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
	CaptureSetup setup;
	setup.mount = kMount;

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

	ProcessSensorData(sensor, storage, timing_config, setup, StopToken(&StopRequested()));

	sensor.Stop();

	return 0;
}