#pragma once

#include <stddef.h>
#include <stdint.h>
#include <cmath>

namespace dlidar
{
    // Timing of one received packet; also the record of packetsNNNN.bin, so the
    // layout is fixed. The fields follow the L2 (its packet type codes and
    // point packet fields); another sensor fills them with its equivalents.
    struct PacketTiming
    {
        int64_t host_mono_ns; // CLOCK_MONOTONIC when the packet was received
        int64_t host_unix_ns; // CLOCK_REALTIME at the same moment
        int64_t sensor_ns;    // the packet's sensor stamp, absolute
        uint32_t seq;         // the packet's sequence number
        uint32_t type;        // kPacketTypePoints or kPacketTypeImu
        float time_increment; // points: seconds between points; IMU: 0
        uint32_t point_num;   // points: points in the packet; IMU: 0
    };
    static_assert(sizeof(PacketTiming) == 40, "packet log record layout");
    static_assert(offsetof(PacketTiming, host_mono_ns) == 0 && offsetof(PacketTiming, host_unix_ns) == 8 &&
                      offsetof(PacketTiming, sensor_ns) == 16 && offsetof(PacketTiming, seq) == 24 &&
                      offsetof(PacketTiming, type) == 28 && offsetof(PacketTiming, time_increment) == 32 &&
                      offsetof(PacketTiming, point_num) == 36,
                  "packet log record field offsets");

    const uint32_t kPacketTypePoints = 102; // L2 LIDAR_POINT_DATA_PACKET_TYPE
    const uint32_t kPacketTypeImu = 104;    // L2 LIDAR_IMU_DATA_PACKET_TYPE
    const char kPacketLogMagic[8] = {'d', 'L', 'p', 'k', 't', 'v', '1', '\n'};

    // Sensor time of a point packet's last point, relative to its first.
    inline int64_t PacketDurationNs(uint32_t point_num, float time_increment)
    {
        return point_num > 1 ? (int64_t)std::llround((double)(point_num - 1) * (double)time_increment * 1e9) : 0;
    }
}
