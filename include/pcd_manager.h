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

// Lens dirtiness above which we warn during capture. A dirty lens
// attenuates/scatters returns and quietly degrades the whole run
// (fogging, dust - relevant for handheld cave/outdoor use).
const float kDirtyPercentageWarnThreshold = 5.0f;

// Non-blocking: reuses whatever dirty_index was cached from the most
// recently parsed packet, so it is safe to call every loop iteration
// without consuming extra data. Warns at most once per interval so a
// persistently dirty lens doesn't spam the console.
void CheckDirtyPercentage(UnitreeLidarReader *lreader)
{
	static auto last_check = std::chrono::steady_clock::now() - std::chrono::seconds(10);
	auto now = std::chrono::steady_clock::now();
	if (now - last_check < std::chrono::seconds(10))
	{
		return;
	}
	last_check = now;

	float dirtyPercentage;
	if (lreader->getDirtyPercentage(dirtyPercentage) && dirtyPercentage > kDirtyPercentageWarnThreshold)
	{
		std::cerr << "WARNING: lens dirty percentage = " << dirtyPercentage
				  << " % (threshold " << kDirtyPercentageWarnThreshold
				  << " %) - clean the sensor housing, points may be degraded." << std::endl;
	}
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

// Reads the IMU sample of the packet just parsed. Acceleration is in g and
// angular velocity in rad/s. Timestamps are assigned later by ImuTimestamper.
bool ReadImuSample(UnitreeLidarReader *lreader, OutputImuData &imuData, ParsedImuData &parsedImu)
{
	LidarImuData imu;
	if (!lreader->getImuData(imu))
	{
		return false;
	}

	parseFromImuPacket(parsedImu, imu);

	imuData.AccelerationX = parsedImu.accel[0];
	imuData.AccelerationY = parsedImu.accel[1];
	imuData.AccelerationZ = parsedImu.accel[2];

	imuData.GyroX = parsedImu.gyro[0];
	imuData.GyroY = parsedImu.gyro[1];
	imuData.GyroZ = parsedImu.gyro[2];

	imuData.ImuId = 0;
	return true;
}

int64_t HostUnixTimeNs()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
			   std::chrono::system_clock::now().time_since_epoch())
		.count();
}

// Reject points inside the housing blind zone (spec: 0.05 m) to filter out
// self-reflection off the sensor housing. Kept a bit past the blind zone
// since accuracy degrades near it.
const float kMinPointRangeM = 0.1f;

// Reject low-reflectivity returns (0-255 scale) that are usually noise/
// multipath rather than a real surface.
const uint8_t kMinPointIntensity = 5;

std::vector<PointDLidar> GetPointCloud(UnitreeLidarReader *lreader)
{
	LidarPointDataPacket lidarDataPacket = lreader->getLidarPointDataPacket();
	PointCloudDLidar cloudOut;

	if (lidarDataPacket.data.point_num == 0)
	{
		return {};
	}

	parseFromPacketToPointCloud(cloudOut, lidarDataPacket, kMinPointRangeM, 100, kMinPointIntensity);
	return cloudOut.points;
}

std::string OutputDirectory()
{
	const char *linuxUser = getenv("USER");
	return std::string("/home/") + (linuxUser ? linuxUser : "") + "/PointCloudDump/";
}

// The SDK buffers packets while nothing reads them (lidar restart, the
// status queries above). Reading those out now instead of sleeping keeps a
// burst of stale packets with compressed host stamps out of the recording.
void DrainPendingPackets(UnitreeLidarReader *lreader, std::chrono::milliseconds duration)
{
	auto until = std::chrono::steady_clock::now() + duration;
	while (std::chrono::steady_clock::now() < until && !StopRequested())
	{
		lreader->runParse();
	}
}

void ProcessSensorData(UnitreeLidarReader *lreader)
{
	const int max_points_per_chunk = 14000;
	const int target_chunks = 1000;

	std::queue<SensorChunk> chunk_queue;
	std::mutex queue_mutex;
	std::condition_variable queue_cv;
	std::atomic<bool> is_capturing{true};

	std::string base_path = OutputDirectory();

	std::thread consumer_thread([&]()
								{
        ContinuousDataWriter stream_writer;

        if (!stream_writer.Open(base_path)) {
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

	DrainPendingPackets(lreader, std::chrono::seconds(2));
	std::cout << ">>> STARTING CONTINUOUS DATA CAPTURE <<< (Ctrl-C to stop)" << std::endl;

	int chunks_produced = 0;
	ImuTimestamper imu_timestamper;
	std::vector<PointDLidar> current_ptCloudResult;
	std::vector<OutputImuData> current_imuResult;

	current_ptCloudResult.reserve(max_points_per_chunk + 2000);
	current_imuResult.reserve((max_points_per_chunk / 10) + 100);

	while (chunks_produced < target_chunks && !StopRequested())
	{
		int result = lreader->runParse();

		switch (result)
		{
		case LIDAR_IMU_DATA_PACKET_TYPE:
		{
			OutputImuData imuData;
			ParsedImuData parsedImu;
			if (ReadImuSample(lreader, imuData, parsedImu))
			{
				imu_timestamper.Push(imuData, parsedImu.timestamp, parsedImu.seq, HostUnixTimeNs(), current_imuResult);
			}
			break;
		}
		case LIDAR_POINT_DATA_PACKET_TYPE:
			std::vector<PointDLidar> ptCloud = GetPointCloud(lreader);
			if (!ptCloud.empty())
			{
				current_ptCloudResult.insert(current_ptCloudResult.end(), ptCloud.begin(), ptCloud.end());
			}
			CheckDirtyPercentage(lreader);
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

	// The timestamper holds back the last ~0.5 s of IMU samples (and the
	// first ~2 s until its clock fit settles); write them out with the
	// remaining points as a final chunk.
	imu_timestamper.Flush(current_imuResult);

	const ImuTimestamper::Stats &imuStats = imu_timestamper.GetStats();
	if (imuStats.samples_out > 0)
	{
		std::cout << "IMU timeline: " << imuStats.samples_out << " of " << imuStats.samples_in << " samples written, period "
				  << imuStats.period_ns / 1e6 << " ms (" << 1e9 / imuStats.period_ns << " Hz), seq "
				  << (imuStats.seq_used ? "used" : "not usable") << ", lost samples " << imuStats.lost_samples
				  << " (" << imuStats.drop_events << " detected drops), index corrections " << imuStats.index_corrections
				  << ", resyncs " << imuStats.resyncs << ", clock steps back " << imuStats.clock_steps_back
				  << ", slew violations " << imuStats.slew_violations
				  << ", dropped: " << imuStats.negative_dropped << " before t=0, " << imuStats.conflicts_dropped
				  << " conflicting, " << imuStats.duplicates_dropped << " duplicate"
				  << ", arrival lead mean " << imuStats.lead_sum_ns / imuStats.samples_out / 1e6
				  << " ms / max " << imuStats.max_lead_ns / 1e6 << " ms" << std::endl;
	}
	if (!current_imuResult.empty() || !current_ptCloudResult.empty())
	{
		SensorChunk final_chunk;
		final_chunk.chunk_index = chunks_produced + 1;
		final_chunk.points = std::move(current_ptCloudResult);
		final_chunk.imu_data = std::move(current_imuResult);

		std::lock_guard<std::mutex> lock(queue_mutex);
		chunk_queue.push(std::move(final_chunk));
	}

	is_capturing = false;
	queue_cv.notify_one();

	consumer_thread.join();

	std::cout << "Capture complete. Recording is in " << base_path << std::endl;
}