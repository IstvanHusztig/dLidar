#pragma once

#include "adapters/livox_mid360s/livox_conversion.h"
#include "domain/recording/recording_profile.h"

#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>

namespace dlidar::livox
{
    // One row of packetstateNNNN.tsv: the header fields of a point packet, as
    // received (docs/livox_mid360s.md). time_type shows whether the stamps
    // are since power-on (0), PTP (1) or GPS (2) time.
    struct PointPacketState
    {
        int64_t stamp_ns; // packet stamp, absolute sensor clock
        uint8_t time_type;
        uint8_t data_type;
        uint8_t frame_cnt;
        uint16_t udp_cnt;
        uint16_t dot_num;
        uint16_t time_interval; // 0.1 us, first to last point
        uint16_t low_confidence; // points the sensor tags low confidence (LowConfidence)
    };

    // `data` is a checked point packet.
    inline PointPacketState MakePointPacketState(const uint8_t *data)
    {
        LivoxLidarEthernetPacket h = Header(data);
        PointPacketState s{StampNs(h), h.time_type, h.data_type, h.frame_cnt, h.udp_cnt, h.dot_num, h.time_interval, 0};
        if (h.data_type == kLivoxLidarCartesianCoordinateHighData)
            for (int i = 0; i < h.dot_num; i++)
            {
                LivoxLidarCartesianHighRawPoint q;
                memcpy(&q, data + kHeaderSize + (size_t)i * sizeof q, sizeof q);
                s.low_confidence += LowConfidence(q.tag);
            }
        return s;
    }

    constexpr const char *kPacketStateHeader = "stamp_ns\tlivox_stamp_ns\ttime_type\tdata_type\tframe_cnt\tudp_cnt\t"
                                               "dot_num\ttime_interval_100ns\tlow_confidence\n";

    // One tab-separated row; stamp_ns is relative to `zero_ns` (the zero of
    // the LAZ gps_time and the IMU timestamp).
    inline void AppendPacketStateRow(std::string &out, const PointPacketState &s, int64_t zero_ns)
    {
        char row[256];
        int n = snprintf(row, sizeof row, "%" PRId64 "\t%" PRId64 "\t%u\t%u\t%u\t%u\t%u\t%u\t%u\n", s.stamp_ns - zero_ns,
                         s.stamp_ns, s.time_type, s.data_type, s.frame_cnt, s.udp_cnt, s.dot_num, s.time_interval,
                         s.low_confidence);
        if (n > 0 && n < (int)sizeof row)
            out.append(row, (size_t)n);
    }

    // What the Mid-360S's recordings say about it (src/domain/recording/recording_profile.h).
    constexpr RecordingProfile kRecordingProfile{
        2,
        "livox_mid360s",
        "Livox Mid-360S",
        {"|xyz| in sensor frame [m]", "distance, sat. at 65535 [mm]", "index of point in packet",
         "frame_cnt<<16 | udp_cnt", "azimuth atan2(y,x), rad", "elevation in sensor frame, rad"},
        kPacketStateHeader,
    };
    static_assert(FitsLas(kRecordingProfile), "LAS texts are at most 31 characters");

    // The IMU's position in the point frame, m (Mid-360S user manual: IMU at
    // x = 11.0, y = 23.29, z = -44.12 mm; same axes as the points).
    const std::array<float, 3> kImuPositionM = {0.011f, 0.02329f, -0.04412f};
}
