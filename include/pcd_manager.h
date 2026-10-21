#pragma once

#include "datawriter.h"
#include "application/capture/chunk_assembler.h"
#include "application/ports/lidar_sensor.h"
#include "application/ports/stop_token.h"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>
#include <unistd.h>

using dlidar::CaptureSetup;
using dlidar::ChunkAssembler;
using dlidar::LidarSensor;
using dlidar::PacketKind;
using dlidar::Polled;
using dlidar::StopToken;

// Lens dirtiness above which we warn during capture. A dirty lens
// attenuates/scatters returns and quietly degrades the whole run
// (fogging, dust - relevant for handheld cave/outdoor use).
const float kDirtyPercentageWarnThreshold = 5.0f;

// Non-blocking: reuses whatever dirtiness the sensor cached from the most
// recently parsed packet, so it is safe to call every loop iteration
// without consuming extra data. Warns at most once per interval so a
// persistently dirty lens doesn't spam the console.
void CheckDirtyPercentage(LidarSensor &sensor)
{
	static auto last_check = std::chrono::steady_clock::now() - std::chrono::seconds(10);
	auto now = std::chrono::steady_clock::now();
	if (now - last_check < std::chrono::seconds(10))
	{
		return;
	}
	last_check = now;

	std::optional<float> dirtyPercentage = sensor.LensDirtiness();
	if (dirtyPercentage && *dirtyPercentage > kDirtyPercentageWarnThreshold)
	{
		std::cerr << "WARNING: lens dirty percentage = " << *dirtyPercentage
				  << " % (threshold " << kDirtyPercentageWarnThreshold
				  << " %) - clean the sensor housing, points may be degraded." << std::endl;
	}
}

std::string OutputDirectory()
{
	const char *linuxUser = getenv("USER");
	return std::string("/home/") + (linuxUser ? linuxUser : "") + "/PointCloudDump/";
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

void ProcessSensorData(LidarSensor &sensor, const ImuTimeline::Config &timing_config, const CaptureSetup &setup,
					   const StopToken &stop, int target_chunks = 200)
{
	// target_chunks: 200 is ~1 min of data at 20 Hz, 18 scans per chunk.

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

	sensor.Prepare(stop);
	dlidar::SensorInfo sensor_info = sensor.Info();
	std::cout << ">>> STARTING CONTINUOUS DATA CAPTURE <<< (Ctrl-C to stop)" << std::endl;

	ChunkAssembler assembler(sensor, timing_config, sensor_info.versions, setup.mount,
							 setup.filter.value_or(sensor_info.default_filter));

	while (assembler.ChunksProduced() < target_chunks && !stop.StopRequested())
	{
		Polled p = sensor.Poll();
		if (p.kind == PacketKind::EndOfStream)
			break;
		if (p.kind == PacketKind::Imu || p.kind == PacketKind::Points)
			assembler.OnPacket(p);
		if (p.kind == PacketKind::Points)
			CheckDirtyPercentage(sensor);

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

	sensor.Stop();
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
