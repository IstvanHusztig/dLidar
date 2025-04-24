#pragma once

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_utilities.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <dll/laszip_api.h>

using namespace unitree_lidar_sdk;

struct OutputImuData
{
	public:
		float Timestamp;

		int ImuId;
		
		int GyroX;
		int GyroY;
		int GyroZ;
		
		int AccelerationX;
		int AccelerationY;
		int AccelerationZ;
};

bool SaveToLaz(const std::string& filename, std::vector<PointUnitree>& points)
{

	constexpr float scale = 0.0001f; // one tenth of milimeter
	
	// find max
	double max_x{std::numeric_limits<double>::lowest()};
	double max_y{std::numeric_limits<double>::lowest()};
	double max_z{std::numeric_limits<double>::lowest()};

	double min_x{std::numeric_limits<double>::max()};
	double min_y{std::numeric_limits<double>::max()};
	double min_z{std::numeric_limits<double>::max()};

	for (const auto& point : points) 
	{
		double x = 0.001 * point.x;
		double y = 0.001 * point.y;
		double z = 0.001 * point.z;

		max_x = std::max(max_x, x);
		max_y = std::max(max_y, y);
		max_z = std::max(max_z, z);

		min_x = std::min(min_x, x);
		min_y = std::min(min_y, y);
		min_z = std::min(min_z, z);
	}

	std::cout << "processing: " << filename << "points " << points.size() << std::endl;

	laszip_POINTER laszip_writer;
	if(laszip_create(&laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: creating laszip writer\n");
		return false;
	}

	// get a pointer to the header of the writer so we can populate it

	laszip_header* header;

	if(laszip_get_header_pointer(laszip_writer, &header))
	{
		fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
		return false;
	}

	// populate the header
	int step = 1;
	if(points.size() > 4000000){
		step = ceil((double)points.size() / 2000000.0);
	}

	if(step < 1){
		step = 1;
	}

	int num_points = 0;
	for(uint i = 0; i < points.size(); i += step){
		num_points ++;
	}

	header->file_source_ID = 4711;
	header->global_encoding = (1 << 0); // see LAS specification for details
	header->version_major = 1;
	header->version_minor = 2;
	header->point_data_format = 1;
	header->point_data_record_length = 0;
	header->number_of_point_records = num_points;//points.size();
	header->number_of_points_by_return[0] = num_points;//buffer.size();
	header->number_of_points_by_return[1] = 0;
	header->point_data_record_length = 28;
	header->x_scale_factor = scale;
	header->y_scale_factor = scale;
	header->z_scale_factor = scale;

	header->max_x = max_x;
	header->min_x = min_x;
	header->max_y = max_y;
	header->min_y = min_y;
	header->max_z = max_z;
	header->min_z = min_z;

	// open the writer
	laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

	if(laszip_open_writer(laszip_writer, filename.c_str(), compress))
	{
		fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
		return false;
	}

	fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

	// get a pointer to the point of the writer that we will populate and write
	laszip_point* laszipPoint;
	
	if(laszip_get_point_pointer(laszip_writer, &laszipPoint))
	{
		fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
		return false;
	}

	laszip_I64 p_count = 0;
	laszip_F64 coordinates[3];

	for(uint i = 0; i < points.size(); i += step)
	{
		laszipPoint->intensity = points[i].intensity;
		laszipPoint->gps_time = points[i].time;
		laszipPoint->user_data = points[i].ring;

		p_count++;

		coordinates[0] = 0.001 * points[i].x;
		coordinates[1] = 0.001 * points[i].y;
		coordinates[2] = 0.001 * points[i].z;
		
		if(laszip_set_coordinates(laszip_writer, coordinates))
		{
			fprintf(stderr, "DLL ERROR: setting coordinates for point %ld\n", p_count);
			return false;
		}

		if(laszip_write_point(laszip_writer))
		{
			fprintf(stderr, "DLL ERROR: writing point %ld\n", p_count);
			return false;
		}
	}

	if(laszip_get_point_count(laszip_writer, &p_count))
	{
		fprintf(stderr, "DLL ERROR: getting point count\n");
		return false;
	}

	fprintf(stderr, "successfully written %ld points\n", p_count);

	// close the writer

	if(laszip_close_writer(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: closing laszip writer\n");
		return false;
	}

	// destroy the writer

	if(laszip_destroy(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
		return false;
	}

	std::cout << "exportLaz DONE" << std::endl;
	return true;
}

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

OutputImuData GetOutputImuData(UnitreeLidarReader *lreader) 
{
	OutputImuData imuData;
	LidarImuData  imu;
	
	if(lreader->getImuData(imu)) 
	{
		std::cout << std::setprecision(20) << "\tIMU system stamp = " << getSystemTimeStamp() << std::endl;
		
		printf("\tseq = %d, stamp = %d.%d\n", imu.info.seq, imu.info.stamp.sec, imu.info.stamp.nsec);

		imuData.AccelerationX = imu.linear_acceleration[0];
		imuData.AccelerationY = imu.linear_acceleration[1];
		imuData.AccelerationZ = imu.linear_acceleration[2];

		imuData.GyroX = imu.angular_velocity[0];
		imuData.GyroY = imu.angular_velocity[1];
		imuData.GyroZ = imu.angular_velocity[2];

		imuData.Timestamp = imu.info.stamp.sec  + imu.info.stamp.nsec / 1.0e9;

		imuData.ImuId = imu.info.seq; 
	}
	
	return imuData;
}

std::vector<PointUnitree> GetPointCloud(UnitreeLidarReader *lreader) 
{
	LidarPointDataPacket lidarDataPacket = reader->getLidarPointDataPacket();
	PointCloudUnitree cloud;
	
	if(lidarDataPacket) 
	{
		printf("A Cloud msg is parsed! \n");
		parseFromPacketToPointCloud(cloud, lidarDataPacket, false);
		//FilterPointCloud(cloud);
	}
	
	return cloud.points;
}

void PrintImuDatasToFile(const std::vector<OutputImuData>& imuVector, int suffix = 0) 
{
	std::string fileName = printf("imu_%d.txt", suffix);
	std::string content;

	for (const auto& imu : imuVector) 
	{
		content += std::to_string(imu.Timestamp) +" " +
				   std::to_string(imu.GyroX) + " " +
				   std::to_string(imu.GyroY) + " " +
				   std::to_string(imu.GyroZ)+ " " +
				   std::to_string(imu.AccelerationX) + " " +
				   std::to_string(imu.AccelerationY) + " " +
				   std::to_string(imu.AccelerationZ) + "\n";
				   
	}

	WriteToFile(fileName, content);
}

void ProcessSensorData(UnitreeLidarReader *lreader) 
{
	int result;
	int max_points = 1500000;
	int current_points = 0;
	int cycleCount = 0;
	std::vector<PointUnitree> ptCloudResult= std::vector<PointUnitree>();
	std::vector<LidarImuData> imuResult= std::vector<LidarImuData>();
		
	RestartLidar(lreader);

	PrintDirtyPercentage(lreader);

	PrintTimeDelay(lreader);
	
	while(cycleCount<1)
	{
		while(current_points< max_points) 
		{
			result = lreader->runParse();

			switch(result) 
			{
				case LIDAR_IMU_DATA_PACKET_TYPE: 
					{
						OutputImuData imuDt = GetOutputImuData(lreader); 
					}
				break;
				case LIDAR_POINT_DATA_PACKET_TYPE:
					{
						std::vector<PointUnitree> ptCloud = GetPointCloud(lreader); 
						if(ptCloud.size()>0)
						{
							ptCloudResult.insert(ptCloudResult.end(), ptCloud.begin(), ptCloud.end());
							current_points += ptCloud.size();
						}
					}
				break;
			}
		}
		
		std::string fileName = "point_cloud_" + std::to_string(cycleCount++) + ".laz";
		SaveToLaz(fileName, ptCloudResult);
		//PrintPointCloudToFile(ptCloudResult, cycleCount);
	}
	
	lreader->stopLidarRotation();
}