#pragma once

#include "application/ports/stop_token.h"
#include "domain/recording/recording_info.h"

#include <stddef.h>
#include <stdint.h>

namespace dlidar::livox
{
    // Host clocks when a packet was received.
    struct HostTime
    {
        int64_t mono_ns; // CLOCK_MONOTONIC: arrival timing (never stepped)
        int64_t unix_ns; // CLOCK_REALTIME: Unix anchor
    };

    // Where LivoxMid360sSensor gets Mid-360S packets from: the live SDK
    // (LiveLivoxSource), a replay of a packet log (SyntheticLivoxSource) or a
    // test. A packet is the bytes of one LivoxLidarEthernetPacket (header and
    // data) as the sensor sent it.
    class LivoxPacketSource
    {
    public:
        static const int kNothing = 0;      // nothing this time (timeout)
        static const int kPacket = 1;       // Data()/Size() hold a packet
        static const int kEndOfStream = -1; // a finite source is exhausted

        virtual ~LivoxPacketSource() = default;

        // Reads the next packet (with `arrival` set); valid until the next call.
        virtual int Next(HostTime &arrival) = 0;
        virtual const uint8_t *Data() const = 0;
        virtual size_t Size() const = 0;

        virtual bool Prepare(const StopToken &) { return true; }
        virtual RecordingInfo Versions() = 0;
        virtual void Stop() {}
    };
}
