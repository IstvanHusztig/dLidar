#include "pcd_manager.h"

// Sent with setLidarWorkMode at start-up; recorded in timingNNNN.json.
const uint32_t kLidarWorkMode = 0;

void SetLidarWorkMode(UnitreeLidarReader *lidarReader)
{
	unitree_lidar_sdk::GetSensorOrientation() = unitree_lidar_sdk::SensorOrientation::VERTICAL;

	std::cout << "set Lidar work mode to: " << kLidarWorkMode << std::endl;
	lidarReader->setLidarWorkMode(kLidarWorkMode);
	sleep(1);
}

// false: the SDK leaves every packet with the L2's own stamp. Point packets
// keep it either way; with true the SDK would overwrite only the IMU stamp
// with the host arrival time, a different clock from the points'. IMU times
// are put on the points' clock by ImuTimeline (include/time_alignment.h).
const bool kUseSystemTimestamp = false;
const uint16_t kCloudScanNum = 18; // SDK default

// Added to every IMU timestamp. Leave 0 unless the field check
// (docs/imu_lidar_time_alignment.md) measures a constant residual offset.
const int64_t kImuOffsetNs = 0;

UnitreeLidarReader *InitializeLidar()
{
	UnitreeLidarReader *lreader = createUnitreeLidarReader();

	std::string lidar_ip = "192.168.0.62";
	std::string local_ip = "192.168.0.2";

	unsigned short lidar_port = 6101;
	unsigned short local_port = 6201;

	if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip, kCloudScanNum, kUseSystemTimestamp))
	{
		printf("Unilidar initialization failed! Exit here!\n");
		exit(-1);
	}
	else
	{
		printf("Unilidar initialization succeed!\n");
	}

	SetLidarWorkMode(lreader);

	return lreader;
}

int main(int argc, char *argv[])
{
	InstallSignalHandlers();

	ImuTimeline::Config timing_config;
	timing_config.imu_offset_ns = kImuOffsetNs;
	for (int i = 1; i < argc; i++)
	{
		std::string arg = argv[i];
		if (arg == "--finalize")
		{
			// Only convert an interrupted recording in place, no capture.
			return RecoverInterruptedRecording(OutputDirectory(), false) ? 0 : 1;
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

	// A recording that was killed or lost power is finalized into its own
	// subdirectory first, so the new recording does not overwrite it.
	if (!RecoverInterruptedRecording(OutputDirectory(), true))
	{
		std::cerr << "Could not finalize the previous recording; fix that first (or run with --finalize)." << std::endl;
		return 1;
	}

	UnitreeLidarReader *lidarReader = InitializeLidar();

	ProcessSensorData(lidarReader, timing_config, kLidarWorkMode);

	lidarReader->stopLidarRotation();

	return 0;
}