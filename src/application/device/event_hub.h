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
    // output - only for trusted, fast channels (the console).
    // AddAsync: the default for anything else (LEDs, Bluetooth). Each channel
    // gets its own thread and a bounded queue; when the queue is full the
    // oldest event is dropped, so a slow or blocked channel never delays a
    // recording. Exceptions from any channel are caught and ignored.
    class EventHub : public DeviceEvents
    {
    public:
        static const size_t kQueueSize = 64;

        EventHub() = default;
        EventHub(const EventHub &) = delete;
        EventHub &operator=(const EventHub &) = delete;
        ~EventHub()
        {
            for (auto &a : async_)
                a->Shutdown();
        }

        // Register before events start; `channel` must outlive the hub.
        void AddSync(DeviceEvents &channel) { sync_.push_back(&channel); }
        void AddAsync(DeviceEvents &channel) { async_.push_back(std::make_shared<Async>(channel)); }

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
            explicit Async(DeviceEvents &c) : channel(c) {}

            DeviceEvents &channel;
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
                    if (!worker.joinable())
                        Start();
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
                        e(channel);
                    }
                    catch (...)
                    {
                    }
                    lock.lock();
                }
                done = true;
                cv.notify_all();
            }

            // Delivers what is queued, waiting at most 1 s for a channel that
            // is stuck; then leaves its thread behind.
            void Shutdown()
            {
                std::unique_lock<std::mutex> lock(mutex);
                if (!worker.joinable())
                    return;
                stop = true;
                cv.notify_all();
                bool finished = cv.wait_for(lock, std::chrono::seconds(1), [&] { return done; });
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

        void Deliver(const Event &e)
        {
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
            for (auto &a : async_)
                a->Push(e);
        }
    };
}
