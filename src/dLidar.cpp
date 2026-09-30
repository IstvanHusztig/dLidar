#include "pcd_manager.h"

void SetLidarWorkMode(UnitreeLidarReader *lidarReader)
{
	unitree_lidar_sdk::GetSensorOrientation() = unitree_lidar_sdk::SensorOrientation::VERTICAL;

	std::cout << "set Lidar work mode to: " << 0 << std::endl;
	lidarReader->setLidarWorkMode(0);
	sleep(1);
}

// true: the SDK overwrites every packet stamp (points and IMU) with the host
// arrival time, so batched IMU samples need ImuTimestamper to rebuild their
// spacing. false: packets keep the LiDAR's own clock, synced once to the
// host clock at startup with syncLidarTimeStamp(). Both streams always share
// whichever clock is chosen.
const bool kUseSystemTimestamp = true;
const uint16_t kCloudScanNum = 18; // SDK default

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

	if (!kUseSystemTimestamp)
	{
		lreader->syncLidarTimeStamp();
	}

	SetLidarWorkMode(lreader);

	return lreader;
}

int main(int argc, char *argv[])
{
	UnitreeLidarReader *lidarReader = InitializeLidar();

	ProcessSensorData(lidarReader);

	lidarReader->stopLidarRotation();

	return 0;
}