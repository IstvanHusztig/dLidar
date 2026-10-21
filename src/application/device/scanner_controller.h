#pragma once

#include "application/capture/record_session.h"
#include "application/ports/device_events.h"
#include "application/ports/lidar_sensor.h"
#include "application/ports/recording_storage.h"
#include "application/ports/stop_token.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace dlidar
{
    // The scanning device's control interface (device-control spec): every
    // input channel - signals and the CLI today, buttons and the phone later -
    // starts and stops recordings through it. Thread-safe. One recording at a
    // time, each on a worker thread; its state changes go to `events` (an
    // EventHub for several output channels).
    //
    // Idle -> Preparing -> Recording -> Finalizing -> Idle (or Error if the
    // sensor did not start; Error accepts a new start like Idle).
    class ScannerController : private DeviceEvents
    {
    public:
        // `process_stop` (may be null) also stops a recording: the signal flag.
        ScannerController(LidarSensor &sensor, RecordingStorage &storage, DeviceEvents &events,
                          const SessionConfig &config, const std::atomic<bool> *process_stop = nullptr)
            : sensor_(sensor), storage_(storage), events_(events), config_(config), process_stop_(process_stop)
        {
            status_.location = storage_.Location();
        }

        ScannerController(const ScannerController &) = delete;
        ScannerController &operator=(const ScannerController &) = delete;

        ~ScannerController()
        {
            StopRecording();
            WaitIdle();
            if (worker_.joinable())
                worker_.join();
        }

        // Starts a recording. False (and nothing changes) unless Idle or Error;
        // `state_out` gets the state the command saw.
        bool StartRecording(DeviceState *state_out = nullptr)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (state_out)
                *state_out = status_.state;
            if (busy_)
                return false;
            if (worker_.joinable())
                worker_.join(); // the previous recording's thread has finished
            busy_ = true;
            stop_ = false;
            worker_ = std::thread([this] { Run(); });
            return true;
        }

        // Ends the running recording normally (its files are finished). No-op when idle.
        void StopRecording() { stop_ = true; }

        DeviceStatus Status() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return status_;
        }

        // Blocks until no recording is running.
        void WaitIdle()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return !busy_; });
        }

    private:
        LidarSensor &sensor_;
        RecordingStorage &storage_;
        DeviceEvents &events_;
        SessionConfig config_;
        const std::atomic<bool> *process_stop_;

        mutable std::mutex mutex_;
        std::condition_variable cv_;
        DeviceStatus status_;
        bool busy_ = false;
        std::atomic<bool> stop_{false};
        std::thread worker_;

        void Run()
        {
            RecordSession session(sensor_, storage_, *this, config_);
            session.Run(StopToken(&stop_, process_stop_));
            {
                std::lock_guard<std::mutex> lock(mutex_);
                busy_ = false;
            }
            cv_.notify_all();
        }

        // The session's events: keep the status, then pass them on.
        void OnState(const DeviceStatus &s) override
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_ = s;
            }
            events_.OnState(s);
        }
        void OnWarning(const DeviceWarning &w) override { events_.OnWarning(w); }
        void OnProgress(const DeviceStatus &s) override
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_ = s;
            }
            events_.OnProgress(s);
        }
        void OnSummary(const RecordingSummary &r) override { events_.OnSummary(r); }
    };
}
