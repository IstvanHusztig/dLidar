#pragma once

#include "adapters/livox_mid360s/livox_packet_source.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

namespace dlidar::livox
{
    // Hands packets from the SDK's data thread (Push, in its callbacks) to the
    // capture thread (Pop), in arrival order. Push copies the packet into a
    // free slot and never waits for the consumer; packets are never dropped:
    // when every slot is in use the pool grows by another block (dropping
    // would break the recording, as for the chunk queue). The first growth
    // is reported once (TakeGrowWarning), so the capture thread can warn.
    // Between recordings the pool is set not to accept: packets are then
    // dropped (counted), so it cannot grow while nobody reads it.
    class LivoxPacketPool
    {
    public:
        static const size_t kSlotBytes = 1536; // a Mid-360 packet is at most ~1400 bytes

        // 8192 slots: ~12 MB, ~4 s of Mid-360S packets.
        explicit LivoxPacketPool(size_t initial_slots = 8192, size_t grow_slots = 1024) : grow_slots_(grow_slots)
        {
            Grow(initial_slots);
            initial_ = slots_;
        }

        // Producer. A packet larger than a slot is not a Mid-360 packet: it is
        // counted and not queued.
        void Push(const HostTime &arrival, const uint8_t *data, size_t size)
        {
            if (!accepting_.load(std::memory_order_relaxed))
            {
                dropped_idle_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            if (size > kSlotBytes)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                oversized_++;
                return;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (free_.empty())
                {
                    Grow(grow_slots_);
                    if (!grew_)
                        grew_ = warn_ = true;
                }
                Slot *s = free_.back();
                free_.pop_back();
                s->arrival = arrival;
                s->size = size;
                memcpy(s->bytes, data, size);
                queue_.push_back(s);
            }
            ready_.notify_one();
        }

        // Consumer: waits up to `timeout` for the oldest packet; true with it
        // in Data()/Size()/Arrival() until the next Pop or Clear.
        bool Pop(std::chrono::milliseconds timeout)
        {
            std::unique_lock<std::mutex> lock(mutex_);
            Release();
            if (!ready_.wait_for(lock, timeout, [this] { return !queue_.empty(); }))
                return false;
            current_ = queue_.front();
            queue_.pop_front();
            return true;
        }

        const uint8_t *Data() const { return current_ ? current_->bytes : nullptr; }
        size_t Size() const { return current_ ? current_->size : 0; }
        HostTime Arrival() const { return current_ ? current_->arrival : HostTime{}; }

        // Drops every queued packet (the backlog before a recording).
        size_t Clear()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Release();
            size_t n = queue_.size();
            free_.insert(free_.end(), queue_.begin(), queue_.end());
            queue_.clear();
            return n;
        }

        // Packets are queued only while accepting; turning it off also drops
        // what is queued.
        void SetAccepting(bool accepting)
        {
            accepting_ = accepting;
            if (!accepting)
                Clear();
        }
        bool Accepting() const { return accepting_; }
        uint64_t DroppedIdle() const { return dropped_idle_; }

        // True once, after the pool first had to grow.
        bool TakeGrowWarning()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            bool w = warn_;
            warn_ = false;
            return w;
        }

        size_t Slots() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return slots_;
        }
        size_t InitialSlots() const { return initial_; }
        size_t Queued() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return queue_.size();
        }
        uint64_t Oversized() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return oversized_;
        }

    private:
        struct Slot
        {
            HostTime arrival;
            size_t size;
            uint8_t bytes[kSlotBytes];
        };

        mutable std::mutex mutex_;
        std::condition_variable ready_;
        std::vector<std::unique_ptr<Slot[]>> blocks_;
        std::vector<Slot *> free_;
        std::deque<Slot *> queue_;
        Slot *current_ = nullptr; // held by the consumer
        size_t grow_slots_;
        size_t slots_ = 0, initial_ = 0;
        bool grew_ = false, warn_ = false;
        uint64_t oversized_ = 0;
        std::atomic<bool> accepting_{true};
        std::atomic<uint64_t> dropped_idle_{0};

        void Grow(size_t n) // with mutex_ held (or in the constructor)
        {
            blocks_.emplace_back(new Slot[n]);
            free_.reserve(slots_ + n);
            for (size_t i = 0; i < n; i++)
                free_.push_back(&blocks_.back()[i]);
            slots_ += n;
        }

        void Release()
        {
            if (current_)
                free_.push_back(current_);
            current_ = nullptr;
        }
    };
}
