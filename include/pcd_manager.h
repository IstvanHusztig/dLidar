#pragma once

#include "unitree_lidar_sdk.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

using namespace unitree_lidar_sdk;

void WriteToFile(std::string fileName, const std::string& content)
{
	const std::string directoryPath = "/home/vs2022/PointCloudDump";
	const std::string filePath = directoryPath + "/" + fileName;
	
	struct stat info;
	
	if (stat(directoryPath.c_str(), &info) != 0) {
		if (mkdir(directoryPath.c_str(), 0777) == -1) {
			std::cerr << "Error: Could not create directory at " << directoryPath << std::endl;
			return;
		}
	}

	std::ofstream outFile(filePath, std::ios::out);

	if (!outFile) {
		std::cerr << "Error: Could not open file at " << filePath << " for writing." << std::endl;
		return;
	}

	outFile << content;

	outFile.close();

	if (outFile.fail()) {
		std::cerr << "Error: Failed to write to file at " << filePath << "." << std::endl;
	} else {
		std::cout << "Content successfully written to " << filePath << "." << std::endl;
	}
}

void PrintDirtyPercentage(UnitreeLidarReader *lreader) {
	float dirtyPercentage;
	while(!lreader->getDirtyPercentage(dirtyPercentage)) {
		lreader->runParse();
	}
	printf("dirty percentage = %f %%\n", dirtyPercentage);
	sleep(1);
}

void PrintTimeDelay(UnitreeLidarReader *lreader) {
	double timeDelay;
	while(!lreader->getTimeDelay(timeDelay)) {
		lreader->runParse();
	}
	printf("time delay (second) = %f\n", timeDelay);
	sleep(1);
}

void RestartLidar(UnitreeLidarReader *lreader) {
	// Stop and start lidar again
	std::cout << "stop lidar rotation ..." << std::endl;
	lreader->stopLidarRotation();
	sleep(3);

	std::cout << "start lidar rotation ..." << std::endl;
	lreader->startLidarRotation();
	sleep(3);
}

void GetImuData(UnitreeLidarReader *lreader) 
{
	LidarImuData imu;
	
	if(lreader->getImuData(imu)) 
	{
		printf("An IMU msg is parsed!\n");
		
		std::cout << std::setprecision(20) << "\tsystem stamp = " << getSystemTimeStamp() << std::endl;
		
		printf("\tseq = %d, stamp = %d.%d\n", imu.info.seq, imu.info.stamp.sec, imu.info.stamp.nsec);

		printf("\tquaternion (x, y, z, w) = [%.4f, %.4f, %.4f, %.4f]\n",
			   imu.quaternion[0],
			   imu.quaternion[1],
			   imu.quaternion[2],
			   imu.quaternion[3]);

		printf("\tangular_velocity (x, y, z) = [%.4f, %.4f, %.4f]\n",
			   imu.angular_velocity[0],
			   imu.angular_velocity[1],
			   imu.angular_velocity[2]);
		printf("\tlinear_acceleration (x, y, z) = [%.4f, %.4f, %.4f]\n",
			   imu.linear_acceleration[0],
			   imu.linear_acceleration[1],
			   imu.linear_acceleration[2]);
	}
}

std::vector<PointUnitree> GetPointCloud(UnitreeLidarReader *lreader) 
{
	PointCloudUnitree cloud;
	
	if(lreader->getPointCloud(cloud)) 
	{
		printf("A Cloud msg is parsed! \n");
		//FilterPointCloud(cloud);
	}
	
	//TransformUnitreeCloudToPCL(cloud, cloudOut);
	return cloud.points;
}

void PrintPointCloudToFile(const std::vector<PointUnitree>& points, int suffix = 0) 
{
	std::string fileName = "point_cloud_" + std::to_string(suffix) + ".dl";
	std::string content;

	for (const auto& point : points) 
	{
		content += std::to_string(point.x) + " " +
			std::to_string(point.y) + " " +
			std::to_string(point.z) + " " +
			std::to_string(point.intensity) + "\n";
	}

	WriteToFile(fileName, content);
}


void ReadSensorData(UnitreeLidarReader *lreader) 
{
	int result;
	int max_points = 2000000;
	int current_points = 0;
	int cycleCount = 0;
	//LidarImuData imu;
	std::vector<PointUnitree> ptCloudResult= std::vector<PointUnitree>();
		
	RestartLidar(lreader);

	PrintDirtyPercentage(lreader);

	PrintTimeDelay(lreader);
	
	while(cycleCount<5)
	{
		while(current_points< max_points) 
		{
			result = lreader->runParse();

			switch(result) 
			{
				/*case LIDAR_IMU_DATA_PACKET_TYPE: 
					GetImuData(lreader); 
					
					break;*/
				case LIDAR_POINT_DATA_PACKET_TYPE:
					std::vector<PointUnitree> ptCloud = GetPointCloud(lreader); 
					if(ptCloud.size()>0)
					{
						ptCloudResult.insert(ptCloudResult.end(), ptCloud.begin(), ptCloud.end());
						current_points += ptCloud.size();
					}
				break;
			}
		}
		cycleCount++;
		PrintPointCloudToFile(ptCloudResult, cycleCount);
	}
	
	lreader->stopLidarRotation();
}