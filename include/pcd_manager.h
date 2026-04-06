#pragma once

#include "datawriter.h"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>
#include <unistd.h>

using namespace unitree_lidar_sdk;

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
	std::cout << "stop lidar rotation ..." << std::endl;
	lreader->stopLidarRotation();
	sleep(3);

	std::cout << "start lidar rotation ..." << std::endl;
	lreader->startLidarRotation();
	sleep(3);
}

// Unitree L2 Imu acceleration is in m/s^2 and angular velocity is in rad/s
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
		imuData.AccelerationX = imu.linear_acceleration[0] / 9.80665f;
		imuData.AccelerationY = imu.linear_acceleration[1] / 9.80665f;
		imuData.AccelerationZ = imu.linear_acceleration[2] / 9.80665f;

		imuData.GyroX = imu.angular_velocity[0];
		imuData.GyroY = imu.angular_velocity[1];
		imuData.GyroZ = imu.angular_velocity[2];

		uint64_t raw_time = ((uint64_t)imu.info.stamp.sec * 1000000000ULL) + (uint64_t)imu.info.stamp.nsec;
		uint64_t hardware_epoch_time = (uint64_t)millis.count() * 1000000ULL;

		if (raw_time > last_packet_base_time + 1500000ULL || last_packet_base_time == 0)
		{
			last_packet_base_time = raw_time;
			current_epoch_base_time = hardware_epoch_time;
			buffer_idx = 0;
		}
		else
		{
			buffer_idx++;
		}

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
		return {};
	}

	parseFromPacketToPointCloud(cloudOut, lidarDataPacket);
	return cloudOut.points;
}

void ProcessSensorData(UnitreeLidarReader *lreader)
{
	const int max_points_per_chunk = 14000;
	const int target_chunks = 200;

	std::queue<SensorChunk> chunk_queue;
	std::mutex queue_mutex;
	std::condition_variable queue_cv;
	std::atomic<bool> is_capturing{true};

	std::string linuxUser = getenv("USER");
	std::string base_path = "/home/" + linuxUser + "/PointCloudDump/";

	std::thread consumer_thread([&]()
								{
        ContinuousDataWriter stream_writer;
        std::string laz_path = base_path + "lidar0001.laz";
        std::string csv_path = base_path + "imu0001.csv";

        if (!stream_writer.Open(laz_path, csv_path)) {
            std::cerr << "Fatal Error: Failed to open output streams!" << std::endl;
            return;
        }

        while (true)
        {
            SensorChunk current_chunk;

            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                queue_cv.wait(lock, [&]{ return !chunk_queue.empty() || !is_capturing; });

                if (chunk_queue.empty() && !is_capturing)
                {
                    break; 
                }

                current_chunk = std::move(chunk_queue.front());
                chunk_queue.pop();
            }

            stream_writer.AppendChunk(current_chunk);
        }

        stream_writer.CloseAndPackageLAZ(); });

	RestartLidar(lreader);
	PrintDirtyPercentage(lreader);
	PrintTimeDelay(lreader);

	std::cout << ">>> STARTING CONTINUOUS DATA CAPTURE <<<" << std::endl;
	sleep(2);

	int chunks_produced = 0;
	std::vector<PointDLidar> current_ptCloudResult;
	std::vector<OutputImuData> current_imuResult;

	current_ptCloudResult.reserve(max_points_per_chunk + 2000);
	current_imuResult.reserve((max_points_per_chunk / 10) + 100);

	while (chunks_produced < target_chunks)
	{
		int result = lreader->runParse();

		switch (result)
		{
		case LIDAR_IMU_DATA_PACKET_TYPE:
			current_imuResult.push_back(GetOutputImuData(lreader));
			break;
		case LIDAR_POINT_DATA_PACKET_TYPE:
			std::vector<PointDLidar> ptCloud = GetPointCloud(lreader);
			if (!ptCloud.empty())
			{
				current_ptCloudResult.insert(current_ptCloudResult.end(), ptCloud.begin(), ptCloud.end());
			}
			break;
		}

		if (current_ptCloudResult.size() >= max_points_per_chunk)
		{
			chunks_produced++;

			SensorChunk new_chunk;
			new_chunk.chunk_index = chunks_produced;
			new_chunk.points = std::move(current_ptCloudResult);
			new_chunk.imu_data = std::move(current_imuResult);

			{
				std::lock_guard<std::mutex> lock(queue_mutex);
				chunk_queue.push(std::move(new_chunk));
			}
			queue_cv.notify_one();

			current_ptCloudResult = std::vector<PointDLidar>();
			current_ptCloudResult.reserve(max_points_per_chunk + 2000);

			current_imuResult = std::vector<OutputImuData>();
			current_imuResult.reserve((max_points_per_chunk / 10) + 100);
		}
	}

	lreader->stopLidarRotation();
	std::cout << "LiDAR stopped. Flushing final data to continuous streams..." << std::endl;

	is_capturing = false;
	queue_cv.notify_one();

	consumer_thread.join();

	std::cout << "Capture complete. Feed continuous_lidar.laz to HDmapper to resolve bias lift-off." << std::endl;
}