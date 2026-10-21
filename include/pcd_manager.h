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

// Calls `get` (a getVersionOf... of the reader) while parsing packets, for at
// most `timeout`. The SDK fills the version from the L2's version packet.
template <typename Get>
std::string QueryVersion(UnitreeLidarReader *lreader, Get get, std::chrono::milliseconds timeout)
{
	std::string version;
	auto until = std::chrono::steady_clock::now() + timeout;
	while (!get(version))
	{
		if (std::chrono::steady_clock::now() >= until || StopRequested())
			return "unknown";
		lreader->runParse();
	}
	return version;
}

RecordingInfo QueryRecordingInfo(UnitreeLidarReader *lreader, uint32_t work_mode)
{
	RecordingInfo info;
	info.work_mode = work_mode;
	info.lidar_firmware_version = QueryVersion(lreader, [&](std::string &v) { return lreader->getVersionOfLidarFirmware(v); },
											   std::chrono::seconds(3));
	info.lidar_hardware_version = QueryVersion(lreader, [&](std::string &v) { return lreader->getVersionOfLidarHardware(v); },
											   std::chrono::seconds(1));
	if (!lreader->getVersionOfSDK(info.sdk_version))
		info.sdk_version = unitree_lidar_sdk_VERSION;
	std::cout << "L2 firmware " << info.lidar_firmware_version << ", hardware " << info.lidar_hardware_version
			  << ", SDK " << info.sdk_version << ", work mode " << work_mode << std::endl;
	return info;
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

// Collects the parsed packets of a recording into SensorChunks for the writer:
// a chunk is ready once it holds kMaxPointsPerChunk points. Used by the
// capture loop and by replays of synthetic packets.
class ChunkAssembler
{
public:
	static const int kMaxPointsPerChunk = 14000;

	ChunkAssembler(const ImuTimeline::Config &config, const RecordingInfo &info) : timing_(config), info_(info)
	{
		Reserve();
	}

	void OnImu(const LidarImuData &imu, HostTime now) { timing_.OnImu(imu, now, imu_); }
	void OnPoints(const LidarPointDataPacket &packet, HostTime now) { timing_.OnPoints(packet, now, points_, imu_); }

	bool ChunkReady() const { return points_.size() >= kMaxPointsPerChunk; }
	int ChunksProduced() const { return chunks_produced_; }
	const ImuTimeline &Timeline() const { return timing_.Timeline(); }

	SensorChunk TakeChunk()
	{
		chunks_produced_++;
		SensorChunk chunk = Collect(chunks_produced_);
		if (!decision_reported_ && timing_.Timeline().Decided())
		{
			// Written early as well, so a killed recording still says how its times were made.
			chunk.timing_report = AddRecordingInfo(timing_.Timeline().Report(), info_);
			decision_reported_ = true;
		}
		Reserve();
		return chunk;
	}

	// Auto mode holds the first ~2 s of IMU samples until it has checked the
	// L2 IMU stamps (the fallback holds ~0.5 s more); they go out with the
	// remaining points as the final chunk.
	SensorChunk Finish()
	{
		timing_.Flush(imu_);
		SensorChunk chunk = Collect(chunks_produced_ + 1);
		chunk.timing_report = AddRecordingInfo(timing_.Timeline().Report(), info_);
		return chunk;
	}

private:
	CaptureTiming timing_;
	RecordingInfo info_;
	std::vector<PointDLidar> points_;
	std::vector<OutputImuData> imu_;
	int chunks_produced_ = 0;
	bool decision_reported_ = false;

	void Reserve()
	{
		points_ = std::vector<PointDLidar>();
		points_.reserve(kMaxPointsPerChunk + 2000);
		imu_ = std::vector<OutputImuData>();
		imu_.reserve((kMaxPointsPerChunk / 10) + 100);
	}

	SensorChunk Collect(int index)
	{
		SensorChunk chunk;
		chunk.chunk_index = index;
		chunk.points = std::move(points_);
		chunk.imu_data = std::move(imu_);
		timing_.TakePacketLog(chunk.packets);
		timing_.TakePacketStateRows(GetGlobalTimeOffsetNs(), chunk.packet_state_rows);
		return chunk;
	}
};

void ProcessSensorData(UnitreeLidarReader *lreader, const ImuTimeline::Config &timing_config, uint32_t work_mode)
{
	const int target_chunks = 200; // ~1 min of data at 20 Hz, 18 scans per chunk

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
	RecordingInfo info = QueryRecordingInfo(lreader, work_mode);

	DrainPendingPackets(lreader, std::chrono::seconds(2));
	std::cout << ">>> STARTING CONTINUOUS DATA CAPTURE <<< (Ctrl-C to stop)" << std::endl;

	ChunkAssembler assembler(timing_config, info);

	while (assembler.ChunksProduced() < target_chunks && !StopRequested())
	{
		int result = lreader->runParse();

		// Both packet types keep the L2's own stamp (use_system_timestamp is
		// off); the host arrival time is taken here for diagnostics and the
		// fallback clock map.
		switch (result)
		{
		case LIDAR_IMU_DATA_PACKET_TYPE:
			assembler.OnImu(lreader->getLidarImuDataPacket().data, HostNow());
			break;
		case LIDAR_POINT_DATA_PACKET_TYPE:
			assembler.OnPoints(lreader->getLidarPointDataPacket(), HostNow());
			CheckDirtyPercentage(lreader);
			break;
		}

		if (assembler.ChunkReady())
		{
			SensorChunk new_chunk = assembler.TakeChunk();
			{
				std::lock_guard<std::mutex> lock(queue_mutex);
				chunk_queue.push(std::move(new_chunk));
			}
			queue_cv.notify_one();
		}
	}

	lreader->stopLidarRotation();
	std::cout << "LiDAR stopped. Flushing final data to continuous streams..." << std::endl;

	{
		SensorChunk final_chunk = assembler.Finish();
		PrintTimingSummary(assembler.Timeline());

		std::lock_guard<std::mutex> lock(queue_mutex);
		chunk_queue.push(std::move(final_chunk));
	}

	is_capturing = false;
	queue_cv.notify_one();

	consumer_thread.join();

	std::cout << "Capture complete. Recording is in " << base_path << std::endl;
}
