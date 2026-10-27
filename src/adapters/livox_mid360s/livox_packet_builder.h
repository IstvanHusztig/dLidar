#pragma once

#include "adapters/livox_mid360s/livox_conversion.h"

#include <cstring>
#include <stdint.h>
#include <vector>

// Builds Mid-360S packets byte for byte as the sensor sends them (packed
// LivoxLidarEthernetPacket, little-endian), for the synthetic source and the
// tests. The CRC is left 0: dLidar does not check it (the SDK does).
namespace dlidar::livox
{
    inline void PutHeader(std::vector<uint8_t> &out, uint8_t data_type, uint16_t dot_num, uint16_t udp_cnt,
                          uint8_t frame_cnt, uint16_t time_interval, int64_t stamp_ns, uint8_t time_type)
    {
        LivoxLidarEthernetPacket h{};
        h.version = 0;
        h.length = (uint16_t)out.size();
        h.time_interval = time_interval;
        h.dot_num = dot_num;
        h.udp_cnt = udp_cnt;
        h.frame_cnt = frame_cnt;
        h.data_type = data_type;
        h.time_type = time_type;
        for (int i = 0; i < 8; i++)
            h.timestamp[i] = (uint8_t)((uint64_t)stamp_ns >> (8 * i));
        memcpy(out.data(), &h, kHeaderSize);
    }

    // A Cartesian high point packet; time_interval in 0.1 us (first to last point).
    inline std::vector<uint8_t> MakePointPacket(int64_t stamp_ns, uint8_t frame_cnt, uint16_t udp_cnt,
                                                uint16_t time_interval,
                                                const std::vector<LivoxLidarCartesianHighRawPoint> &points,
                                                uint8_t time_type = 0)
    {
        std::vector<uint8_t> out(kHeaderSize + points.size() * sizeof(LivoxLidarCartesianHighRawPoint));
        if (!points.empty())
            memcpy(out.data() + kHeaderSize, points.data(), points.size() * sizeof(LivoxLidarCartesianHighRawPoint));
        PutHeader(out, kLivoxLidarCartesianCoordinateHighData, (uint16_t)points.size(), udp_cnt, frame_cnt,
                  time_interval, stamp_ns, time_type);
        return out;
    }

    inline std::vector<uint8_t> MakeImuPacket(int64_t stamp_ns, uint16_t udp_cnt, const LivoxLidarImuRawPoint &sample,
                                              uint8_t time_type = 0)
    {
        std::vector<uint8_t> out(kHeaderSize + sizeof sample);
        memcpy(out.data() + kHeaderSize, &sample, sizeof sample);
        PutHeader(out, kLivoxLidarImuData, 1, udp_cnt, 0, 0, stamp_ns, time_type);
        return out;
    }
}
