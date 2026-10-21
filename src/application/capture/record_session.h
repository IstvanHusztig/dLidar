#pragma once

#include "application/capture/chunk_assembler.h"
#include "application/capture/lens_monitor.h"
#include "application/ports/device_events.h"
#include "application/ports/lidar_sensor.h"
#include "application/ports/recording_storage.h"
#include "application/ports/stop_token.h"

#include <atomic>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

namespace dlidar
{
    struct SessionConfig
    {
        ImuTimeline::Config timing;
        CaptureSetup setup;
        // A recording ends after this many chunks (200: ~1 min at 20 Hz, 18
        // scans per chunk) or on a stop. Temporary until a button starts and
        // stops recordings.
        int target_chunks = 200;
    };

    // One recording: sensor start-up, capture loop, finishing the files.
    // Producer/consumer: this thread polls the sensor and assembles chunks, a
    // writer thread hands them to the storage, so slow storage (SD card)
    // never stalls packet reading. States and messages go to `events`.
    class RecordSession
    {
    public:
        RecordSession(LidarSensor &sensor, RecordingStorage &storage, DeviceEvents &events, const SessionConfig &config)
            : sensor_(sensor), storage_(storage), events_(events), config_(config) {}

        // Runs the whole recording; true if it was written completely.
        bool Run(const StopToken &stop)
        {
            status_.location = storage_.Location();
            status_.chunks = 0;
            status_.points = 0;
            Publish(DeviceState::Preparing);

            std::thread writer([this] { WriterLoop(); });

            bool sensor_ok = sensor_.Prepare(stop);
            SensorInfo info = sensor_.Info();
            assembler_ = std::make_unique<ChunkAssembler>(sensor_, config_.timing, info.versions, config_.setup.mount,
                                                          config_.setup.filter.value_or(info.default_filter));
            if (!sensor_ok)
                events_.OnWarning({WarningKind::SensorFailed, "ERROR: the sensor did not start."});
            else
            {
                Publish(DeviceState::Recording);
                Capture(stop);
            }

            if (sensor_ok)
                sensor_.Stop();
            Publish(DeviceState::Finalizing);
            {
                SensorChunk final_chunk = assembler_->Finish();
                events_.OnSummary(Summary(assembler_->Timeline()));
                Push(std::move(final_chunk));
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                capturing_ = false;
            }
            cv_.notify_one();
            writer.join();

            bool ok = sensor_ok && writer_ok_;
            status_.last_ok = ok;
            Publish(sensor_ok ? DeviceState::Idle : DeviceState::Error);
            return ok;
        }

        const DeviceStatus &Status() const { return status_; }

    private:
        LidarSensor &sensor_;
        RecordingStorage &storage_;
        DeviceEvents &events_;
        SessionConfig config_;
        DeviceStatus status_;
        std::unique_ptr<ChunkAssembler> assembler_;

        std::queue<SensorChunk> queue_;
        std::mutex mutex_;
        std::condition_variable cv_;
        bool capturing_ = true;
        std::atomic<bool> writer_ok_{false};

        void Publish(DeviceState s)
        {
            status_.state = s;
            events_.OnState(status_);
        }

        void Capture(const StopToken &stop)
        {
            LensMonitor lens;
            while (assembler_->ChunksProduced() < config_.target_chunks && !stop.StopRequested())
            {
                Polled p = sensor_.Poll();
                if (p.kind == PacketKind::EndOfStream)
                    break;
                if (p.kind == PacketKind::Imu || p.kind == PacketKind::Points)
                    assembler_->OnPacket(p);
                if (p.kind == PacketKind::Points)
                    lens.Check(sensor_, p.record.host_mono_ns, events_);

                if (assembler_->ChunkReady())
                {
                    SensorChunk chunk = assembler_->TakeChunk();
                    status_.chunks = assembler_->ChunksProduced();
                    status_.points += chunk.points.size();
                    Push(std::move(chunk));
                    events_.OnProgress(status_);
                }
            }
        }

        void Push(SensorChunk chunk)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                queue_.push(std::move(chunk));
            }
            cv_.notify_one();
        }

        void WriterLoop()
        {
            if (!storage_.Open())
            {
                events_.OnWarning({WarningKind::StorageFailed, "Fatal Error: Failed to open output streams!"});
                return;
            }
            while (true)
            {
                SensorChunk chunk;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    cv_.wait(lock, [&] { return !queue_.empty() || !capturing_; });
                    if (queue_.empty() && !capturing_)
                        break;
                    chunk = std::move(queue_.front());
                    queue_.pop();
                }
                storage_.Append(chunk);
            }
            writer_ok_ = storage_.Close();
        }

        static RecordingSummary Summary(const ImuTimeline &t)
        {
            RecordingSummary s;
            s.mode = t.ActiveMode();
            s.decision = t.Decision();
            s.stats = t.GetStats();
            s.clocks = t.MeasureClocks();
            return s;
        }
    };
}
