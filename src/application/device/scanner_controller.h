#pragma once

#include "application/capture/record_session.h"
#include "application/ports/device_events.h"
#include "application/ports/lidar_sensor.h"
#include "application/ports/recording_storage.h"
#include "application/ports/stop_token.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <string>
#include <thread>

namespace dlidar
{
    // The scanning device's control interface (device-control spec): every
    // input channel - signals and the CLI today, buttons and the phone later -
    // starts and stops recordings through it. Thread-safe. One recording at a
    // time, on the controller's worker thread; its state changes go to
    // `events` (an EventHub for several output channels).
    //
    // Idle -> Preparing -> Recording -> Finalizing -> Idle, or Error if the
    // recording failed (sensor did not start, storage failed, an exception).
    // Error accepts a new start like Idle.
    //
    // The state seen by StartRecording and Status() is the one that decides
    // whether a start is accepted: an accepted start is Preparing at once, and
    // the final Idle/Error is stored before it is sent to the channels, so a
    // channel may start the next recording from its OnState(Idle).
    class ScannerController : private DeviceEvents
    {
    public:
        // `process_stop` (may be null) also stops a recording: the signal flag.
        ScannerController(LidarSensor &sensor, RecordingStorage &storage, DeviceEvents &events,
                          const SessionConfig &config, const std::atomic<bool> *process_stop = nullptr)
            : sensor_(sensor), storage_(storage), events_(events), config_(config), process_stop_(process_stop)
        {
            status_.location = storage_.Location();
            worker_ = std::thread([this] { WorkerLoop(); });
        }

        ScannerController(const ScannerController &) = delete;
        ScannerController &operator=(const ScannerController &) = delete;

        // Stops a running recording, waits until its files are finished; a
        // start that has not begun yet is dropped.
        ~ScannerController()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                shutdown_ = true;
            }
            stop_ = true;
            cv_.notify_all();
            worker_.join();
        }

        // Starts a recording. False (and nothing changes) unless Idle or Error;
        // `state_out` gets the state the command saw.
        bool StartRecording(DeviceState *state_out = nullptr)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (state_out)
                    *state_out = status_.state;
                if (shutdown_ || (status_.state != DeviceState::Idle && status_.state != DeviceState::Error))
                    return false;
                stop_ = false;
                start_pending_ = true;
                accepted_++;
                status_.state = DeviceState::Preparing;
                status_.chunks = 0;
                status_.points = 0;
            }
            cv_.notify_all();
            return true;
        }

        // Ends the running recording normally (its files are finished). No-op when idle.
        void StopRecording() { stop_ = true; }

        DeviceStatus Status() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return status_;
        }

        // Blocks until every accepted recording has ended and its final state
        // has been sent to the channels. Returns at once when called from an
        // event callback (the worker cannot wait for itself).
        void WaitIdle()
        {
            if (std::this_thread::get_id() == worker_.get_id())
                return;
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return finished_ == accepted_; });
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
        bool shutdown_ = false;
        bool start_pending_ = false;
        bool ended_ = false;     // the running recording published Idle/Error
        uint64_t accepted_ = 0;  // starts accepted
        uint64_t finished_ = 0;  // recordings whose final state was delivered
        std::atomic<bool> stop_{false};
        std::thread worker_;     // last: started once the members above exist

        void WorkerLoop()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            while (true)
            {
                cv_.wait(lock, [&] { return start_pending_ || shutdown_; });
                if (shutdown_)
                    return;
                start_pending_ = false;
                ended_ = false;
                lock.unlock();
                RunOne();
                lock.lock();
            }
        }

        void RunOne()
        {
            std::string failure;
            try
            {
                RecordSession session(sensor_, storage_, *this, config_);
                session.Run(StopToken(&stop_, process_stop_));
            }
            catch (const std::exception &e)
            {
                failure = e.what();
            }
            catch (...)
            {
                failure = "unknown exception";
            }
            bool ended;
            DeviceStatus s;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ended = ended_;
                s = status_;
            }
            if (ended)
                return;
            // The session did not get to its final state.
            OnWarning({WarningKind::RecordingFailed, "ERROR: the recording stopped on an internal error: " + failure});
            s.state = DeviceState::Error;
            s.last_ok = false;
            OnState(s);
        }

        // Channels never affect the controller (the EventHub also catches;
        // this covers a single channel passed directly).
        template <typename F>
        void Forward(F f)
        {
            try
            {
                f();
            }
            catch (...)
            {
            }
        }

        // The session's events: keep the status, then pass them on.
        void OnState(const DeviceStatus &s) override
        {
            bool final_state = s.state == DeviceState::Idle || s.state == DeviceState::Error;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_ = s;
                if (final_state)
                    ended_ = true;
            }
            Forward([&] { events_.OnState(s); });
            if (final_state)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    finished_++;
                }
                cv_.notify_all();
            }
        }
        void OnWarning(const DeviceWarning &w) override
        {
            Forward([&] { events_.OnWarning(w); });
        }
        void OnProgress(const DeviceStatus &s) override
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                status_ = s;
            }
            Forward([&] { events_.OnProgress(s); });
        }
        void OnSummary(const RecordingSummary &r) override
        {
            Forward([&] { events_.OnSummary(r); });
        }
    };
}
