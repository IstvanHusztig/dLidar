#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dlidar
{
    // One recorded point (recording format 2, docs/l2-recording-format.md).
    // Written as-is to points_tempNNNN.bin, so the layout is fixed. The fields
    // from `range` on go into the LAZ extra bytes; they follow the L2's
    // conventions, and a sensor that has no such value sets it to 0.
    struct PointRecord
    {
        float x; // m, output frame (+X front, +Y left, +Z up)
        float y;
        float z;
        float intensity;
        double time; // s, relative to the recording's time zero
        uint32_t ring;
        float range;           // m, measured range (L2: range_scale * (ranges[j] + range_bias))
        float alpha;           // rad, the alpha angle used for the point (bias included)
        float theta;           // rad, the theta angle used for the point (bias included)
        uint32_t packet_seq;   // the packet's sequence number
        uint16_t raw_range;    // mm, range as sent by the sensor
        uint16_t sample_index; // index of the point in its packet
    };

    static_assert(sizeof(PointRecord) == 48, "points temp record layout (format 2)");
    static_assert(offsetof(PointRecord, x) == 0 && offsetof(PointRecord, y) == 4 && offsetof(PointRecord, z) == 8 &&
                      offsetof(PointRecord, intensity) == 12 && offsetof(PointRecord, time) == 16 &&
                      offsetof(PointRecord, ring) == 24 && offsetof(PointRecord, range) == 28 &&
                      offsetof(PointRecord, alpha) == 32 && offsetof(PointRecord, theta) == 36 &&
                      offsetof(PointRecord, packet_seq) == 40 && offsetof(PointRecord, raw_range) == 44 &&
                      offsetof(PointRecord, sample_index) == 46,
                  "points temp record field offsets (format 2)");
}
