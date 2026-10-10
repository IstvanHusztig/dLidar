#pragma once

#include "datawriter.h"
#include "capture_timing.h"

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

std::string OutputDirectory()
{
	const char *linuxUser = getenv("USER");
	return std::string("/home/") + (linuxUser ? linuxUser : "") + "/PointCloudDump/";
}

// The SDK buffers packets while nothing reads them (lidar restart, the
// status queries above). Reading those out now instead of sleeping keeps a
// burst of stale packets (late arrivals) out of the recording.
void DrainPendingPackets(UnitreeLidarReader *lreader, std::chrono::milliseconds duration)
{
	auto until = std::chrono::steady_clock::now() + duration;
	while (std::chrono::steady_clock::now() < until && !StopRequested())
	{
		lreader->runParse();
	}
}

void PrintTimingSummary(const ImuTimeline &timeline)
{
	const ImuTimeline::Stats &st = timeline.GetStats();
	ImuTimeline::Clocks c = timeline.MeasureClocks();
	std::cout << "IMU time source: " << ImuTimeline::ModeName(timeline.ActiveMode()) << " (" << timeline.Decision() << ")\n"
			  << "IMU: " << st.imu_out << " of " << st.imu_in << " samples written, dropped " << st.non_increasing_dropped
			  << " non-increasing, " << st.duplicates_dropped << " duplicate, " << st.negative_dropped << " before t=0, "
			  << st.unmapped_dropped << " unmapped; lost " << st.lost_samples << ", stamp jumps " << st.stamp_jumps
			  << ", nudged " << st.nudged << std::endl;
	if (c.valid)
	{
		std::cout << "Clocks over " << c.span_s << " s: IMU period " << c.imu_period_ns / 1e6 << " ms on the L2 clock ("
				  << 1e9 / c.imu_period_ns << " Hz), L2 clock vs host " << c.lidar_rate_ppm << " ppm, IMU stamps vs host "
				  << c.imu_rate_ppm << " ppm; IMU - point min latency " << c.min_latency_diff_start_ms << " -> "
				  << c.min_latency_diff_end_ms << " ms; mean extra latency IMU " << c.imu_extra_latency_ms << " ms, points "
				  << c.lidar_extra_latency_ms << " ms; the old host-stamped IMU would have lagged by "
				  << c.old_lag_start_ms << " -> " << c.old_lag_end_ms << " ms" << std::endl;
	}
}

void ProcessSensorData(UnitreeLidarReader *lreader, const ImuTimeline::Config &timing_config)
{
	const int max_points_per_chunk = 14000;
	const int target_chunks = 800; // ~1 min of data at 20 Hz, 18 scans per chunk

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
	CaptureTiming timing(timing_config);
	bool decision_reported = false;
	std::vector<PointDLidar> current_ptCloudResult;
	std::vector<OutputImuData> current_imuResult;

	current_ptCloudResult.reserve(max_points_per_chunk + 2000);
	current_imuResult.reserve((max_points_per_chunk / 10) + 100);

	while (chunks_produced < target_chunks && !StopRequested())
	{
		int result = lreader->runParse();

		// Both packet types keep the L2's own stamp (use_system_timestamp is
		// off); the host arrival time is taken here for diagnostics and the
		// fallback clock map.
		switch (result)
		{
		case LIDAR_IMU_DATA_PACKET_TYPE:
			timing.OnImu(lreader->getLidarImuDataPacket().data, HostNow(), current_imuResult);
			break;
		case LIDAR_POINT_DATA_PACKET_TYPE:
			timing.OnPoints(lreader->getLidarPointDataPacket(), HostNow(), current_ptCloudResult, current_imuResult);
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
			timing.TakePacketLog(new_chunk.packets);
			if (!decision_reported && timing.Timeline().Decided())
			{
				// Written early as well, so a killed recording still says how its times were made.
				new_chunk.timing_report = timing.Timeline().Report();
				decision_reported = true;
			}

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

	// Auto mode holds the first ~2 s of IMU samples until it has checked the
	// L2 IMU stamps (the fallback holds ~0.5 s more); write them out with the
	// remaining points as a final chunk.
	timing.Flush(current_imuResult);
	PrintTimingSummary(timing.Timeline());

	{
		SensorChunk final_chunk;
		final_chunk.chunk_index = chunks_produced + 1;
		final_chunk.points = std::move(current_ptCloudResult);
		final_chunk.imu_data = std::move(current_imuResult);
		timing.TakePacketLog(final_chunk.packets);
		final_chunk.timing_report = timing.Timeline().Report();

		std::lock_guard<std::mutex> lock(queue_mutex);
		chunk_queue.push(std::move(final_chunk));
	}

	is_capturing = false;
	queue_cv.notify_one();

	consumer_thread.join();

	std::cout << "Capture complete. Recording is in " << base_path << std::endl;
}