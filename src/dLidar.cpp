#include "pcd_manager.h"

void SetLidarWorkMode(UnitreeLidarReader *lidarReader)
{
	std::cout << "set Lidar work mode to: " << 0 << std::endl;
	lidarReader->setLidarWorkMode(0);
	sleep(1);
}

UnitreeLidarReader* InitializeLidar() 
{
	UnitreeLidarReader *lreader = createUnitreeLidarReader();

	std::string lidar_ip = "192.168.0.62";
	std::string local_ip = "192.168.0.2";

	unsigned short lidar_port = 6101;
	unsigned short local_port = 6201;

	if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip))
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
	UnitreeLidarReader *lidarReader = InitializeLidar();

	ProcessSensorData(lidarReader);
	
	lidarReader->stopLidarRotation();
	
	return 0;
}