#pragma once

#include <livox_lidar_def.h>

// The Mid-360S IMU setting (SetLivoxLidarImuRange) from settings values to
// the SDK's codes.
namespace dlidar::livox
{
    struct ImuRangeCodes
    {
        LivoxLidarImuOutRate rate;
        LivoxLidarAccelRange accel;
        LivoxLidarGyroRange gyro;
    };

    // False if a value is not one the sensor supports.
    inline bool ToImuRangeCodes(int rate_hz, int accel_g, double gyro_dps, ImuRangeCodes &out)
    {
        switch (rate_hz)
        {
        case 200: out.rate = kLivoxLidarImuOutRate200Hz; break;
        case 500: out.rate = kLivoxLidarImuOutRate500Hz; break;
        case 100: out.rate = kLivoxLidarImuOutRate100Hz; break;
        case 50: out.rate = kLivoxLidarImuOutRate50Hz; break;
        default: return false;
        }
        switch (accel_g)
        {
        case 4: out.accel = kLivoxLidarAccelRange4G; break;
        case 8: out.accel = kLivoxLidarAccelRange8G; break;
        case 16: out.accel = kLivoxLidarAccelRange16G; break;
        case 32: out.accel = kLivoxLidarAccelRange32G; break;
        default: return false;
        }
        const double gyro[] = {2000, 1000, 500, 250, 125, 62.5, 31.25, 15.625}; // codes 0..7
        for (int i = 0; i < 8; i++)
            if (gyro_dps == gyro[i])
            {
                out.gyro = (LivoxLidarGyroRange)i;
                return true;
            }
        return false;
    }
}
