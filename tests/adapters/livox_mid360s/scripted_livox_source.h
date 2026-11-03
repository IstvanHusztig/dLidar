#pragma once

// Test helper: a LivoxPacketSource that hands out queued packets in order,
// then ends the stream.
#include "adapters/livox_mid360s/livox_packet_source.h"

#include <deque>
#include <utility>
#include <vector>

namespace dlidar::test
{
    class ScriptedLivoxSource : public livox::LivoxPacketSource
    {
    public:
        void Add(std::vector<uint8_t> packet, livox::HostTime arrival) { queue_.push_back({std::move(packet), arrival}); }

        int Next(livox::HostTime &arrival) override
        {
            if (queue_.empty())
                return kEndOfStream;
            current_ = std::move(queue_.front());
            queue_.pop_front();
            arrival = current_.second;
            return kPacket;
        }
        const uint8_t *Data() const override { return current_.first.data(); }
        size_t Size() const override { return current_.first.size(); }
        RecordingInfo Versions() override
        {
            RecordingInfo info;
            info.lidar_firmware_version = "scripted";
            return info;
        }

    private:
        std::deque<std::pair<std::vector<uint8_t>, livox::HostTime>> queue_;
        std::pair<std::vector<uint8_t>, livox::HostTime> current_;
    };
}
