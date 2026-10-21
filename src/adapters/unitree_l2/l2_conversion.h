#pragma once

#include "adapters/unitree_l2/vendor/unitree_lidar_protocol.h"
#include "application/ports/lidar_sensor.h"
#include "domain/geometry/point_filter.h"
#include "domain/recording/point_record.h"

#include <stdint.h>
#include <vector>

// The L2 packet -> sensor-frame data conversion (docs/l2_point_conversion.md).
// Unitree's point math from its SDK header, with dLidar's changes: double
// point times relative to the recording zero, the filter, the extra fields.
// Mount and units are applied by the core (CaptureTiming).
namespace dlidar::l2
{
    // The L2's default point filter: rejects points inside the housing blind
    // zone (spec: 0.05 m, kept a bit past it since accuracy degrades near it:
    // 0.1 m), beyond 100 m, and low-reflectivity returns (intensity below 5 of
    // 255), which are usually noise or multipath rather than a real surface.
    const PointFilter kDefaultFilter{0.1f, 100.0f, 5.0f};

    inline int64_t StampToNs(const unitree_lidar_sdk::TimeStamp &stamp)
    {
        return (int64_t)stamp.sec * 1000000000LL + (int64_t)stamp.nsec;
    }

    // IMU packet -> sample in the sensor frame, SI units, absolute L2 stamp.
    void ParseImu(const unitree_lidar_sdk::LidarImuData &imu, ImuReading &out);

    // Point packet -> points in the sensor frame appended to `out`, times in s
    // relative to `zero_ns`. Samples without a return or outside the packet's
    // range limits are skipped, and so are the ones `filter` rejects (before
    // the trig, to save CPU).
    void ParsePoints(const unitree_lidar_sdk::LidarPointDataPacket &packet, int64_t zero_ns, const PointFilter &filter,
                     std::vector<PointRecord> &out);
}
