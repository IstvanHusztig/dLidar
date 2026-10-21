#pragma once

#include <stdint.h>

namespace dlidar
{
    // One row of imuNNNN.csv. Acceleration in g, angular velocity in rad/s,
    // output frame (+X front, +Y left, +Z up).
    struct ImuSample
    {
        int64_t LidarTimestamp; // ns, relative to the recording's time zero (same base as point times)
        int64_t EpochTimestamp; // ns, Unix epoch; LidarTimestamp + a fixed offset
        int ImuId;

        float GyroX;
        float GyroY;
        float GyroZ;

        float AccelerationX;
        float AccelerationY;
        float AccelerationZ;
    };
}
