#pragma once

#include "adapters/unitree_l2/vendor/unitree_lidar_protocol.h"
#include "application/ports/stop_token.h"
#include "domain/recording/recording_info.h"

#include <optional>
#include <stdint.h>

namespace dlidar
{
    // Host clocks when a packet was received.
    struct HostTime
    {
        int64_t mono_ns; // CLOCK_MONOTONIC: arrival timing (never stepped)
        int64_t unix_ns; // CLOCK_REALTIME: Unix anchor
    };

    // Where UnitreeL2Sensor gets L2 packets from: the live SDK reader
    // (LiveL2Source), a replay of a packet log (SyntheticL2Source) or a test.
    class L2PacketSource
    {
    public:
        static const int kEndOfStream = -1;

        virtual ~L2PacketSource() = default;

        // Reads the next packet: LIDAR_POINT_DATA_PACKET_TYPE or
        // LIDAR_IMU_DATA_PACKET_TYPE (with `arrival` set), 0 for nothing usable,
        // kEndOfStream when a finite source is exhausted.
        virtual int Next(HostTime &arrival) = 0;
        virtual const unitree_lidar_sdk::LidarPointDataPacket &PointPacket() const = 0;
        virtual const unitree_lidar_sdk::LidarImuData &ImuPacket() const = 0;

        virtual bool Prepare(const StopToken &) { return true; }
        virtual RecordingInfo Versions() = 0;
        virtual std::optional<float> LensDirtiness() { return std::nullopt; }
        virtual void Stop() {}
    };
}
