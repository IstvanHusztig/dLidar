#pragma once

#include "application/capture/chunk_assembler.h"
#include "application/capture/lens_monitor.h"
#include "application/ports/device_events.h"
#include "application/ports/lidar_sensor.h"
#include "application/ports/recording_storage.h"
#include "application/ports/stop_token.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <queue>
#include <sstream>
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
        // No point from the sensor for this long (host time) ends the
        // recording: it lost power or its connection, or it restarted and
        // sends only IMU packets.
        std::chrono::milliseconds sensor_silence_timeout{10000};
    };

    // One recording: sensor start-up, capture loop, finishing the files.
    // Producer/consumer: this thread polls the sensor and assembles chunks, a
    // writer thread hands them to the storage, so slow storage (SD card)
    // never stalls packet reading. States and messages go to `events`, from
    // this thread and (storage warnings) from the writer thread.
    //
    // The storage is opened once the sensor has started, so a recording that
    // never started leaves no files. It ends in Error unless the sensor
    // started and the storage wrote everything. A storage failure stops the capture. An exception (sensor,
    // storage, out of memory) or a silent sensor ends the capture too; the
    // chunks already queued are still written and the files closed.
    class RecordSession
    {
    public:
        RecordSession(LidarSensor &sensor, RecordingStorage &storage, DeviceEvents &events, const SessionConfig &config)
            : sensor_(sensor), storage_(storage), events_(events), config_(config) {}

        // Runs the whole recording; true if it was written completely (or,
        // stopped during the start-up, nothing had to be written).
        bool Run(const StopToken &stop)
        {
            status_.location = storage_.Location();
            status_.chunks = 0;
            status_.points = 0;
            Publish(DeviceState::Preparing);

            // The storage opens only once the sensor runs: a failed start or
            // a stop during the start-up leaves no files behind (dMapper reads
            // every .csv/.laz in the folder).
            std::thread writer;
            bool sensor_ok = false, recorded = false;
            std::string failure;
            try
            {
                sensor_ok = sensor_.Prepare(stop);
                if (!sensor_ok)
                    events_.OnWarning({WarningKind::SensorFailed, "ERROR: the sensor did not start."});
                else if (!stop.StopRequested())
                {
                    SensorInfo info = sensor_.Info();
                    assembler_ = std::make_unique<ChunkAssembler>(sensor_, config_.timing, info.versions, config_.setup.mount,
                                                                  config_.setup.filter.value_or(info.default_filter));
                    writer = std::thread([this] { WriterLoop(); });
                    recorded = true;
                    Publish(DeviceState::Recording);
                    Capture(stop);
                }

                if (sensor_ok)
                    sensor_.Stop();
                if (recorded)
                {
                    Publish(DeviceState::Finalizing);
                    SensorChunk final_chunk = assembler_->Finish();
                    events_.OnSummary(Summary(assembler_->Timeline()));
                    Push(std::move(final_chunk));
                }
            }
            catch (const std::exception &e)
            {
                failure = e.what();
            }
            catch (...)
            {
                failure = "unknown exception";
            }
            bool failed = !failure.empty();
            if (writer.joinable())
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    capturing_ = false;
                }
                cv_.notify_one();
                writer.join();
            }

            if (failed)
            {
                StopSensorQuietly();
                events_.OnWarning({WarningKind::RecordingFailed, "ERROR: the recording stopped on an internal error: " + failure});
            }
            bool ok = sensor_ok && !failed && !sensor_silent_ && (!recorded || writer_ok_);
            status_.last_ok = ok;
            Publish(ok ? DeviceState::Idle : DeviceState::Error);
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

        // Chunks waiting for the writer. Not bounded: dropping chunks would
        // break the recording, so a slow storage only gets a warning.
        std::queue<SensorChunk> queue_;
        std::mutex mutex_;
        std::condition_variable cv_;
        bool capturing_ = true;
        bool backlog_warned_ = false;
        std::atomic<bool> writer_ok_{false};
        std::atomic<bool> writer_failed_{false}; // stops the capture
        bool sensor_silent_ = false;               // the watchdog stopped the capture

        // ~40 MB of points waiting: the storage is far slower than the sensor.
        static const size_t kBacklogWarnChunks = 50;

        void Publish(DeviceState s)
        {
            status_.state = s;
            events_.OnState(status_);
        }

        void Capture(const StopToken &stop)
        {
            LensMonitor lens;
            auto last_points = std::chrono::steady_clock::now();
            while (assembler_->ChunksProduced() < config_.target_chunks && !stop.StopRequested() && !writer_failed_)
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

                // Host time, not the packets' arrival stamps: a silent sensor
                // delivers no packet to stamp (Poll() returns None), and a
                // replay delivers its log as fast as it can.
                auto now = std::chrono::steady_clock::now();
                if (p.kind == PacketKind::Points && sensor_.Points().count > 0)
                    last_points = now;
                else if (now - last_points >= config_.sensor_silence_timeout)
                {
                    sensor_silent_ = true;
                    std::ostringstream text;
                    text << "ERROR: no points from the sensor for " << config_.sensor_silence_timeout.count() / 1000.0
                         << " s (power or connection lost, or the sensor restarted); the recording was stopped.";
                    events_.OnWarning({WarningKind::SensorSilent, text.str()});
                    break;
                }
            }
        }

        void Push(SensorChunk chunk)
        {
            size_t waiting = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                queue_.push(std::move(chunk));
                if (queue_.size() >= kBacklogWarnChunks && !backlog_warned_)
                {
                    backlog_warned_ = true;
                    waiting = queue_.size();
                }
            }
            cv_.notify_one();
            if (waiting)
                events_.OnWarning({WarningKind::StorageSlow, "WARNING: the storage is " + std::to_string(waiting) +
                                                                 " chunks behind the sensor; the recording is held in memory."});
        }

        void StopSensorQuietly()
        {
            try
            {
                sensor_.Stop();
            }
            catch (...)
            {
            }
        }

        void WriterLoop()
        {
            bool opened = false;
            try
            {
                if (!storage_.Open())
                {
                    writer_failed_ = true;
                    events_.OnWarning({WarningKind::StorageFailed, "Fatal Error: Failed to open output streams!"});
                    return;
                }
                opened = true;
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
            catch (const std::exception &e)
            {
                WriterFailed(opened, e.what());
            }
            catch (...)
            {
                WriterFailed(opened, "unknown exception");
            }
        }

        // An exception in the storage: stop the capture, still try to finish
        // what was written (whatever is left behind is recovered on the next start).
        void WriterFailed(bool opened, const std::string &what)
        {
            writer_failed_ = true;
            writer_ok_ = false;
            events_.OnWarning({WarningKind::StorageFailed, "ERROR: writing the recording failed: " + what});
            if (!opened)
                return;
            try
            {
                storage_.Close();
            }
            catch (...)
            {
            }
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
