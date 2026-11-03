#pragma once

#include "application/capture/capture_timing.h"
#include "application/ports/lidar_sensor.h"
#include "domain/recording/recording_info.h"
#include "domain/recording/sensor_chunk.h"

#include <utility>
#include <vector>

namespace dlidar
{
    // Collects the packets of a recording into SensorChunks for the storage: a
    // chunk is ready once it holds kMaxPointsPerChunk points. Check
    // ChunkReady() after every packet.
    class ChunkAssembler
    {
    public:
        static const int kMaxPointsPerChunk = 14000;

        ChunkAssembler(LidarSensor &sensor, const ImuTimeline::Config &config, const RecordingInfo &info,
                       const AxisMap &mount, const PointFilter &filter)
            : timing_(sensor, config, mount, filter), info_(info)
        {
            Reserve();
        }

        void OnPacket(const Polled &p) { timing_.OnPacket(p, points_, imu_); }

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
        // sensor's IMU stamps (the fallback holds ~0.5 s more); they go out with
        // the remaining points as the final chunk.
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
        std::vector<PointRecord> points_;
        std::vector<ImuSample> imu_;
        int chunks_produced_ = 0;
        bool decision_reported_ = false;

        void Reserve()
        {
            points_ = std::vector<PointRecord>();
            points_.reserve(kMaxPointsPerChunk + 2000);
            imu_ = std::vector<ImuSample>();
            imu_.reserve((kMaxPointsPerChunk / 10) + 100);
        }

        SensorChunk Collect(int index)
        {
            SensorChunk chunk;
            chunk.chunk_index = index;
            chunk.points = std::move(points_);
            chunk.imu_data = std::move(imu_);
            timing_.TakePacketLog(chunk.packets);
            timing_.TakePacketStateRows(timing_.ZeroNs(), chunk.packet_state_rows);
            return chunk;
        }
    };
}
