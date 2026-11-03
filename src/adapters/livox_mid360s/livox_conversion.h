#pragma once

#include "application/ports/lidar_sensor.h"
#include "domain/geometry/point_filter.h"
#include "domain/recording/point_record.h"

#include <livox_lidar_def.h>

#include <stddef.h>
#include <stdint.h>
#include <vector>

// The Mid-360S packet -> sensor-frame data conversion (docs/livox_mid360s.md,
// docs/recording-format.md). Only the packed packet layout of the SDK's
// livox_lidar_def.h is used, not the SDK library. Mount and units are
// applied by the core (CaptureTiming).
namespace dlidar::livox
{
    // The Mid-360S's default point filter: its 0.1 m blind zone and 100 m
    // cut-off (spec sheet). No intensity floor: its reflectivity scale differs
    // from the L2's; tune at bring-up.
    const PointFilter kDefaultFilter{0.1f, 100.0f, 0.0f};

    // Bytes before LivoxLidarEthernetPacket::data.
    const size_t kHeaderSize = offsetof(LivoxLidarEthernetPacket, data);
    static_assert(kHeaderSize == 36, "Mid-360 packet header (Livox protocol)");
    static_assert(sizeof(LivoxLidarCartesianHighRawPoint) == 14 && sizeof(LivoxLidarImuRawPoint) == 24,
                  "Mid-360 point and IMU records (Livox protocol)");

    enum class PacketStatus
    {
        Ok,
        TooShort,   // fewer bytes than the header and its dot_num records
        UnknownType // data_type other than IMU, Cartesian high/low, spherical
    };

    // Checks that `size` bytes hold a whole packet of a known data type.
    PacketStatus CheckPacket(const uint8_t *data, size_t size);

    // The packet's header, by copy (the bytes may be unaligned).
    LivoxLidarEthernetPacket Header(const uint8_t *data);

    // The packet stamp: the time of its first sample, ns on the sensor clock.
    int64_t StampNs(const LivoxLidarEthernetPacket &header);

    // Point i of a packet is at stamp + i * step: its time_interval (0.1 us,
    // first to last point) spread evenly over its dot_num points.
    double PointStepS(const LivoxLidarEthernetPacket &header);
    int64_t DurationNs(const LivoxLidarEthernetPacket &header);

    // packet_seq of a point packet: frame_cnt << 16 | udp_cnt, as sent.
    inline uint32_t PacketSeq(const LivoxLidarEthernetPacket &h) { return (uint32_t)h.frame_cnt << 16 | h.udp_cnt; }

    // Tag bits 0-1 (glued objects) or 2-3 (rain, fog, dust) say "low confidence".
    inline bool LowConfidence(uint8_t tag) { return (tag & 3) == 2 || ((tag >> 2) & 3) == 2; }

    // IMU packet (data type 0, checked) -> its sample in the sensor frame, SI
    // units, absolute sensor stamp; `seq` is left to the caller.
    void ParseImu(const uint8_t *data, ImuReading &out);

    // Cartesian high point packet (data type 1, checked) -> points in the
    // sensor frame appended to `out`, times in s relative to `zero_ns`. Points
    // `filter` rejects are skipped, and with `drop_low_confidence` the ones
    // the sensor flags as low confidence.
    void ParsePoints(const uint8_t *data, int64_t zero_ns, const PointFilter &filter, bool drop_low_confidence,
                     std::vector<PointRecord> &out);
}
