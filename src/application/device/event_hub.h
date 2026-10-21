#pragma once

#include "application/ports/device_events.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace dlidar
{
    // Fans the device's events out to every registered output channel.
    //
    // AddSync: called on the publishing thread, in order with other console
    // output - only for trusted, fast channels (the console). A recording
    // publishes from two threads (capture and writer); the hub delivers to
    // the sync channels one event at a time, so they need no locking.
    // AddAsync: the default for anything else (LEDs, Bluetooth). Each channel
    // gets its own thread and a bounded queue; when the queue is full the
    // oldest event is dropped, so a slow or blocked channel never delays a
    // recording. Exceptions from any channel are caught and ignored.
    //
    // Register every channel before events start. The hub must outlive its
    // publishers (destroy the ScannerController first).
    class EventHub : public DeviceEvents
    {
    public:
        static const size_t kQueueSize = 64;

        EventHub() = default;
        EventHub(const EventHub &) = delete;
        EventHub &operator=(const EventHub &) = delete;
        // Delivers what is queued, giving all async channels together at
        // most 1 s; a channel still stuck after that is left behind on its
        // thread, which keeps the channel alive until the call returns.
        ~EventHub()
        {
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            for (auto &a : async_)
                a->RequestStop();
            for (auto &a : async_)
                a->Finish(deadline);
        }

        // `channel` must outlive the hub.
        void AddSync(DeviceEvents &channel) { sync_.push_back(&channel); }
        // Shared with the channel's thread, which may outlive the hub.
        void AddAsync(std::shared_ptr<DeviceEvents> channel)
        {
            auto a = std::make_shared<Async>(std::move(channel));
            a->Start();
            async_.push_back(std::move(a));
        }

        void OnState(const DeviceStatus &s) override
        {
            Deliver([s](DeviceEvents &c) { c.OnState(s); });
        }
        void OnWarning(const DeviceWarning &w) override
        {
            Deliver([w](DeviceEvents &c) { c.OnWarning(w); });
        }
        void OnProgress(const DeviceStatus &s) override
        {
            Deliver([s](DeviceEvents &c) { c.OnProgress(s); });
        }
        void OnSummary(const RecordingSummary &r) override
        {
            Deliver([r](DeviceEvents &c) { c.OnSummary(r); });
        }

    private:
        using Event = std::function<void(DeviceEvents &)>;

        // Shared with its worker thread, so a worker stuck in its channel at
        // shutdown can be detached without touching freed state.
        struct Async : std::enable_shared_from_this<Async>
        {
            explicit Async(std::shared_ptr<DeviceEvents> c) : channel(std::move(c)) {}

            std::shared_ptr<DeviceEvents> channel;
            std::mutex mutex;
            std::condition_variable cv;
            std::deque<Event> queue;
            bool stop = false, done = false;
            uint64_t dropped = 0;
            std::thread worker;

            void Push(Event e)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (queue.size() >= kQueueSize)
                    {
                        queue.pop_front();
                        dropped++;
                    }
                    queue.push_back(std::move(e));
                }
                cv.notify_one();
            }

            void Start()
            {
                std::shared_ptr<Async> self = shared_from_this();
                worker = std::thread([self] { self->Run(); });
            }

            void Run()
            {
                std::unique_lock<std::mutex> lock(mutex);
                while (true)
                {
                    cv.wait(lock, [&] { return stop || !queue.empty(); });
                    if (queue.empty())
                        break; // stop requested and nothing left
                    Event e = std::move(queue.front());
                    queue.pop_front();
                    lock.unlock();
                    try
                    {
                        e(*channel);
                    }
                    catch (...)
                    {
                    }
                    lock.lock();
                }
                done = true;
                cv.notify_all();
            }

            void RequestStop()
            {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    stop = true;
                }
                cv.notify_all();
            }

            // Waits until `deadline` for the queue to be delivered; a stuck
            // channel's thread is then left behind.
            void Finish(std::chrono::steady_clock::time_point deadline)
            {
                std::unique_lock<std::mutex> lock(mutex);
                bool finished = cv.wait_until(lock, deadline, [&] { return done; });
                if (!finished)
                    queue.clear(); // the stuck channel gets nothing more once it returns
                lock.unlock();
                if (finished)
                    worker.join();
                else
                    worker.detach();
            }
        };

        std::vector<DeviceEvents *> sync_;
        std::vector<std::shared_ptr<Async>> async_;
        // Recursive: a sync channel may publish from inside its callback.
        std::recursive_mutex sync_mutex_;

        void Deliver(const Event &e)
        {
            {
                std::lock_guard<std::recursive_mutex> lock(sync_mutex_);
                for (DeviceEvents *c : sync_)
                {
                    try
                    {
                        e(*c);
                    }
                    catch (...)
                    {
                    }
                }
            }
            for (auto &a : async_)
                a->Push(e);
        }
    };
}
