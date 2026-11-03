#pragma once

#include "application/ports/device_events.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

namespace dlidar::phone
{
    // Device events on their way from the hub's thread to the phone loop
    // (design D5). Pushing never blocks for long: the queue is bounded and a
    // progress replaces a progress at its end. On overflow the oldest event is
    // dropped and counted, but never the latest state. `wake` runs (on the
    // pushing thread, under the lock) when the first event of a batch
    // arrives; the loop then takes the whole batch.
    class Mailbox
    {
    public:
        static const size_t kMaxEvents = 256;

        struct Event
        {
            enum Kind
            {
                State,
                Progress,
                Warning,
                Summary
            } kind;
            DeviceStatus status; // State, Progress
            DeviceWarning warning{};
            RecordingSummary summary;
        };

        explicit Mailbox(std::function<void()> wake) : wake_(std::move(wake)) {}

        void Push(Event e)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_)
                return;
            if (e.kind == Event::Progress && !events_.empty() && events_.back().kind == Event::Progress)
                events_.back() = std::move(e);
            else
            {
                if (events_.size() >= kMaxEvents)
                    DropOldest();
                events_.push_back(std::move(e));
            }
            // Under the lock: Close() cannot complete while a wake runs, so
            // the loop it wakes still exists. It must not block.
            if (!wake_pending_ && wake_)
                wake_();
            wake_pending_ = true;
        }

        // The batch, and how many events were dropped before it.
        std::deque<Event> Take(uint64_t &dropped)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::deque<Event> out;
            out.swap(events_);
            dropped = dropped_;
            dropped_ = 0;
            wake_pending_ = false;
            return out;
        }

        bool Empty() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return events_.empty();
        }

        // Ignores every later Push (the loop is gone; the hub may still deliver).
        void Close()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
            events_.clear();
        }

    private:
        mutable std::mutex mutex_;
        std::function<void()> wake_;
        std::deque<Event> events_;
        uint64_t dropped_ = 0;
        bool wake_pending_ = false;
        bool closed_ = false;

        void DropOldest()
        {
            size_t last_state = events_.size();
            for (size_t i = events_.size(); i-- > 0;)
                if (events_[i].kind == Event::State)
                {
                    last_state = i;
                    break;
                }
            size_t victim = last_state == 0 ? 1 : 0;
            events_.erase(events_.begin() + victim);
            dropped_++;
        }
    };

    // The phone link as an output channel (hub.AddAsync): every event goes to
    // the mailbox; it holds the mailbox by shared_ptr, so the loop may stop
    // first.
    class PhoneEvents : public DeviceEvents
    {
    public:
        explicit PhoneEvents(std::shared_ptr<Mailbox> mailbox) : mailbox_(std::move(mailbox)) {}

        void OnState(const DeviceStatus &s) override { Push(Mailbox::Event::State, s); }
        void OnProgress(const DeviceStatus &s) override { Push(Mailbox::Event::Progress, s); }
        void OnWarning(const DeviceWarning &w) override
        {
            Mailbox::Event e{Mailbox::Event::Warning, {}, w, {}};
            mailbox_->Push(std::move(e));
        }
        void OnSummary(const RecordingSummary &r) override
        {
            Mailbox::Event e{Mailbox::Event::Summary, {}, {}, r};
            mailbox_->Push(std::move(e));
        }

    private:
        std::shared_ptr<Mailbox> mailbox_;

        void Push(Mailbox::Event::Kind kind, const DeviceStatus &s)
        {
            Mailbox::Event e{kind, s, {}, {}};
            mailbox_->Push(std::move(e));
        }
    };
}
