#pragma once

#include "unitree_lidar_sdk.h"
#include "d_lidar_util.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <dll/laszip_api.h>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

using namespace unitree_lidar_sdk;

struct OutputImuData
{
public:
	uint64_t LidarTimestamp;
	uint64_t EpochTimestamp;
	int ImuId;

	float GyroX;
	float GyroY;
	float GyroZ;

	float AccelerationX;
	float AccelerationY;
	float AccelerationZ;
};

struct SensorChunk
{
	int chunk_index;
	std::vector<PointDLidar> points;
	std::vector<OutputImuData> imu_data;
};

bool SaveToLazOrLas(const std::string &filename, std::vector<PointDLidar> &points)
{

	constexpr float scale = 0.0001f; // one tenth of milimeter

	// find max
	double max_x{std::numeric_limits<double>::lowest()};
	double max_y{std::numeric_limits<double>::lowest()};
	double max_z{std::numeric_limits<double>::lowest()};

	double min_x{std::numeric_limits<double>::max()};
	double min_y{std::numeric_limits<double>::max()};
	double min_z{std::numeric_limits<double>::max()};

	for (const auto &point : points)
	{
		double x = point.x;
		double y = point.y;
		double z = point.z;

		max_x = std::max(max_x, x);
		max_y = std::max(max_y, y);
		max_z = std::max(max_z, z);

		min_x = std::min(min_x, x);
		min_y = std::min(min_y, y);
		min_z = std::min(min_z, z);
	}

	std::cout << "processing: " << filename << "points " << points.size() << std::endl;

	laszip_POINTER laszip_writer;
	if (laszip_create(&laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: creating laszip writer\n");
		return false;
	}

	// get a pointer to the header of the writer so we can populate it

	laszip_header *header;

	if (laszip_get_header_pointer(laszip_writer, &header))
	{
		fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
		return false;
	}

	// populate the header
	int step = 1;
	if (points.size() > 4000000)
	{
		step = ceil((double)points.size() / 2000000.0);
	}

	if (step < 1)
	{
		step = 1;
	}

	int num_points = 0;
	for (uint i = 0; i < points.size(); i += step)
	{
		num_points++;
	}

	header->file_source_ID = 4711;
	header->global_encoding = (1 << 0); // see LAS specification for details
	header->version_major = 1;
	header->version_minor = 2;
	header->point_data_format = 1;
	header->point_data_record_length = 0;
	header->number_of_point_records = num_points;		// points.size();
	header->number_of_points_by_return[0] = num_points; // buffer.size();
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

	if (laszip_open_writer(laszip_writer, filename.c_str(), compress))
	{
		fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
		return false;
	}

	fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

	// get a pointer to the point of the writer that we will populate and write
	laszip_point *laszipPoint;

	if (laszip_get_point_pointer(laszip_writer, &laszipPoint))
	{
		fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
		return false;
	}

	laszip_I64 p_count = 0;
	laszip_F64 coordinates[3];

	for (uint i = 0; i < points.size(); i += step)
	{
		laszipPoint->intensity = points[i].intensity;
		laszipPoint->gps_time = points[i].time;
		laszipPoint->user_data = points[i].ring;

		p_count++;

		coordinates[0] = points[i].x;
		coordinates[1] = points[i].y;
		coordinates[2] = points[i].z;

		if (laszip_set_coordinates(laszip_writer, coordinates))
		{
			fprintf(stderr, "DLL ERROR: setting coordinates for point %ld\n", p_count);
			return false;
		}

		if (laszip_write_point(laszip_writer))
		{
			fprintf(stderr, "DLL ERROR: writing point %ld\n", p_count);
			return false;
		}
	}

	if (laszip_get_point_count(laszip_writer, &p_count))
	{
		fprintf(stderr, "DLL ERROR: getting point count\n");
		return false;
	}

	fprintf(stderr, "successfully written %ld points\n", p_count);

	// close the writer

	if (laszip_close_writer(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: closing laszip writer\n");
		return false;
	}

	// destroy the writer

	if (laszip_destroy(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
		return false;
	}

	std::cout << "exportLaz DONE" << std::endl;
	return true;
}

void WriteToFile(std::string filePath, const std::string &content)
{
	std::ofstream outFile(filePath, std::ios::out);

	if (!outFile)
	{
		std::cerr << "Error: Could not open file at " << filePath << " for writing." << std::endl;
		return;
	}

	outFile << content;

	outFile.close();

	if (outFile.fail())
	{
		std::cerr << "Error: Failed to write to file at " << filePath << "." << std::endl;
	}
	else
	{
		std::cout << "Content successfully written to " << filePath << "." << std::endl;
	}
}

void PrintDirtyPercentage(UnitreeLidarReader *lreader)
{
	float dirtyPercentage;
	while (!lreader->getDirtyPercentage(dirtyPercentage))
	{
		lreader->runParse();
	}
	printf("dirty percentage = %f %%\n", dirtyPercentage);
	sleep(1);
}

void PrintTimeDelay(UnitreeLidarReader *lreader)
{
	double timeDelay;
	while (!lreader->getTimeDelay(timeDelay))
	{
		lreader->runParse();
	}
	printf("time delay (second) = %f\n", timeDelay);
	sleep(1);
}

void RestartLidar(UnitreeLidarReader *lreader)
{
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
	LidarImuData imu;

	auto now = std::chrono::system_clock::now();
	auto duration = now.time_since_epoch();
	auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration);

	static uint64_t last_packet_base_time = 0;
	static uint64_t current_epoch_base_time = 0;
	static int buffer_idx = 0;

	if (lreader->getImuData(imu))
	{
		imuData.AccelerationX = imu.linear_acceleration[0];
		imuData.AccelerationY = imu.linear_acceleration[1];
		imuData.AccelerationZ = imu.linear_acceleration[2];

		imuData.GyroX = imu.angular_velocity[0];
		imuData.GyroY = imu.angular_velocity[1];
		imuData.GyroZ = imu.angular_velocity[2];

		// 1. Get the true hardware time to stay perfectly synchronized with the .laz Point Cloud
		uint64_t raw_time = ((uint64_t)imu.info.stamp.sec * 1000000000ULL) + (uint64_t)imu.info.stamp.nsec;
		uint64_t hardware_epoch_time = (uint64_t)millis.count() * 1000000ULL;

		// 2. Threshold check: > 1.5ms (1500000 ns) difference means a fresh UDP packet arrived
		if (raw_time > last_packet_base_time + 1500000ULL || last_packet_base_time == 0)
		{
			last_packet_base_time = raw_time;
			current_epoch_base_time = hardware_epoch_time;
			buffer_idx = 0;
		}
		else
		{
			// The difference is tiny (microseconds). It is a buffered sample from the SAME packet.
			buffer_idx++;
		}

		// 3. Anchor timestamp to the true hardware time, but space the 4 readings perfectly by 1.66ms
		imuData.LidarTimestamp = last_packet_base_time + (buffer_idx * 1666666ULL);
		imuData.EpochTimestamp = current_epoch_base_time + (buffer_idx * 1666666ULL);
		imuData.ImuId = 0;
	}
	else
	{
		imuData.ImuId = -1;
	}

	return imuData;
}

std::vector<PointDLidar> GetPointCloud(UnitreeLidarReader *lreader)
{
	LidarPointDataPacket lidarDataPacket = lreader->getLidarPointDataPacket();
	PointCloudDLidar cloudOut;

	if (lidarDataPacket.data.point_num == 0)
	{
		std::cout << "No point data in this packet!" << std::endl;
		return {};
	}

	parseFromPacketToPointCloud(cloudOut, lidarDataPacket);

	return cloudOut.points;
}

void PrintImuDatasToFile(std::string fileName, const std::vector<OutputImuData> &imuVector)
{
	std::ostringstream content_stream;
	content_stream << "gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix\n";

	for (const auto &imu : imuVector)
	{
		content_stream << std::fixed << std::setprecision(17)
					   << imu.GyroX << " " << imu.GyroY << " " << imu.GyroZ << " "
					   << imu.AccelerationX << " " << imu.AccelerationY << " " << imu.AccelerationZ << " "
					   << imu.ImuId << " " << imu.LidarTimestamp << " " << imu.EpochTimestamp << "\n";
	}

	WriteToFile(fileName, content_stream.str());
}

void ProcessSensorData(UnitreeLidarReader *lreader)
{
	// Tuning parameters
	const int max_points_per_chunk = 14000; // As requested, ~14k points per chunk
	const int target_chunks = 15;			// Adjust based on your scanning needs

	// Concurrency primitives
	std::queue<SensorChunk> chunk_queue;
	std::mutex queue_mutex;
	std::condition_variable queue_cv;
	std::atomic<bool> is_capturing{true};

	std::string linuxUser = getenv("USER");
	std::string base_path = "/home/" + linuxUser + "/PointCloudDump/";

	// ==========================================
	// CONSUMER THREAD: Disk I/O (.laz & .csv)
	// ==========================================
	std::thread consumer_thread([&]()
								{
        while (true)
        {
            SensorChunk current_chunk;

            // 1. Wait for data or shutdown signal
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [&]{ return !chunk_queue.empty() || !is_capturing; });

                // If capturing stopped and queue is drained, exit thread
                if (chunk_queue.empty() && !is_capturing)
                {
                    break;
                }

                // 2. Extract the chunk and immediately release the lock
                current_chunk = std::move(chunk_queue.front());
                chunk_queue.pop();
            } // lock goes out of scope here, unblocking the Producer

            // 3. Heavy I/O Operations (Safe to run slowly here)
            std::string fileName = base_path + "lidar000" + std::to_string(current_chunk.chunk_index) + ".laz";
            std::string imuFileName = base_path + "imu000" + std::to_string(current_chunk.chunk_index) + ".csv";

            SaveToLazOrLas(fileName, current_chunk.points);
            PrintImuDatasToFile(imuFileName, current_chunk.imu_data);
        } });

	// ==========================================
	// PRODUCER THREAD (Main): Network & UDP Parsing
	// ==========================================
	RestartLidar(lreader);
	PrintDirtyPercentage(lreader);
	PrintTimeDelay(lreader);

	std::cout << ">>> STARTING MULTITHREADED DATA CAPTURE <<<" << std::endl;
	sleep(2);

	int chunks_produced = 0;
	std::vector<PointDLidar> current_ptCloudResult;
	std::vector<OutputImuData> current_imuResult;

	// Pre-reserve memory to prevent reallocation overhead during the UDP loop
	current_ptCloudResult.reserve(max_points_per_chunk + 2000);
	current_imuResult.reserve((max_points_per_chunk / 10) + 100);

	while (chunks_produced < target_chunks)
	{
		// Run as fast as possible. NO std::cout or printf inside this loop!
		int result = lreader->runParse();

		switch (result)
		{
		case LIDAR_IMU_DATA_PACKET_TYPE:
		{
			current_imuResult.push_back(GetOutputImuData(lreader));
			break;
		}
		case LIDAR_POINT_DATA_PACKET_TYPE:
		{
			std::vector<PointDLidar> ptCloud = GetPointCloud(lreader);
			if (!ptCloud.empty())
			{
				current_ptCloudResult.insert(current_ptCloudResult.end(), ptCloud.begin(), ptCloud.end());
			}
			break;
		}
		}

		// Check if we have accumulated enough points for a chunk
		if (current_ptCloudResult.size() >= max_points_per_chunk)
		{
			chunks_produced++;

			SensorChunk new_chunk;
			new_chunk.chunk_index = chunks_produced;

			// std::move transfers ownership of the underlying memory buffers instantly.
			// This avoids a massive copy operation of the 14,000 points.
			new_chunk.points = std::move(current_ptCloudResult);
			new_chunk.imu_data = std::move(current_imuResult);

			// Lock, push, and notify Consumer
			{
				std::lock_guard<std::mutex> lock(queue_mutex);
				chunk_queue.push(std::move(new_chunk));
			}
			queue_cv.notify_one();

			// Re-initialize and pre-reserve vectors for the next UDP packets
			current_ptCloudResult = std::vector<PointDLidar>();
			current_ptCloudResult.reserve(max_points_per_chunk + 2000);

			current_imuResult = std::vector<OutputImuData>();
			current_imuResult.reserve((max_points_per_chunk / 10) + 100);
		}
	}

	// ==========================================
	// GRACEFUL SHUTDOWN
	// ==========================================
	lreader->stopLidarRotation();
	std::cout << "LiDAR stopped. Waiting for the Consumer thread to finish writing remaining chunks..." << std::endl;

	// Signal the Consumer to drain the queue and exit
	is_capturing = false;
	queue_cv.notify_one();

	// Block the main thread until the Consumer is completely finished
	consumer_thread.join();

	std::cout << "All data successfully saved. Multithreaded capture complete." << std::endl;
}