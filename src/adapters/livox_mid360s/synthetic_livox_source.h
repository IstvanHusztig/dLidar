#pragma once

#include "adapters/livox_mid360s/livox_packet_source.h"
#include "domain/recording/packet_timing.h"

#include <vector>

namespace dlidar::livox
{
    // Replays the packet sequence of a packetsNNNN.bin timing log as Mid-360S
    // packets. The log holds only timing, so packet contents are synthetic and
    // deterministic: point packets of 96 points whose rays (a 360 deg x
    // -7..52 deg pattern) hit a box room, and an IMU at rest. The sensor
    // stamps, seq and host times come from the log, so two builds fed the
    // same log write the same recording. A log of any sensor can be replayed
    // (an L2 log gives Mid-360S packets with the L2's timing).
    class SyntheticLivoxSource : public LivoxPacketSource
    {
    public:
        // Replays `log` up to `seconds` of host time after its first packet.
        SyntheticLivoxSource(std::vector<PacketTiming> log, double seconds);

        int Next(HostTime &arrival) override;
        const uint8_t *Data() const override { return packet_.data(); }
        size_t Size() const override { return packet_.size(); }
        RecordingInfo Versions() override;

    private:
        std::vector<PacketTiming> log_;
        double seconds_;
        size_t next_ = 0;
        uint64_t points_made_ = 0; // rays so far: the scan pattern position
        std::vector<uint8_t> packet_;
    };
}
