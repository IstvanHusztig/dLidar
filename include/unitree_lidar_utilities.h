/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#if defined(_WIN32) && !defined(__MINGW32__)
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;           // NOLINT
typedef unsigned short uint16_t; // NOLINT
typedef int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
// intptr_t and friends are defined in crtdefs.h through stdio.h.
#else
#include <stdint.h>
#endif

#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <deque>
#include <vector>
#include <memory>
#include <math.h>
#include <chrono>

#include "unitree_lidar_sdk_config.h"
#include "unitree_lidar_protocol.h"

namespace unitree_lidar_sdk
{
    /**
     * @brief Shared zero-point for all sensor streams to preserve nanosecond precision
     * when using 64-bit doubles in HDmapper.
     */
    inline int64_t &GetGlobalTimeOffsetNs()
    {
        static int64_t offset = 0;
        return offset;
    }

    /**
     * @brief Defines the physical mounting orientation of the sensor
     */
    enum class SensorOrientation
    {
        STANDARD, // +X Front, +Y Left, +Z Up
        VERTICAL  // Physical: +Z Front, +X Down, +Y Left
    };

    /**
     * @brief Global property to toggle the sensor orientation mapping.
     * Defaults to SensorOrientation::STANDARD.
     */
    inline SensorOrientation &GetSensorOrientation()
    {
        static SensorOrientation orientation = SensorOrientation::STANDARD;
        return orientation;
    }

    ///////////////////////////////////////////////////////////////////////////////
    // CONSTANTS
    ///////////////////////////////////////////////////////////////////////////////
    const float DEGREE_TO_RADIAN = M_PI / 180.0;
    const float RADIAN_TO_DEGREE = 180.0 / M_PI;

    ///////////////////////////////////////////////////////////////////////////////
    // TYPES
    ///////////////////////////////////////////////////////////////////////////////

    /**
     * @brief Point Type
     */
    typedef struct
    {
        float x;
        float y;
        float z;
        float intensity;
        float time;
        uint32_t ring;
    } PointUnitree;

    /*
     * @brief Point Type DLidar
     */
    typedef struct
    {
        float x;
        float y;
        float z;
        float intensity;
        double time;
        uint32_t ring;
    } PointDLidar;

    /**
     * @brief Parsed IMU Data container for modular axis mapping.
     */
    typedef struct
    {
        float gyro[3];
        float accel[3];
        int64_t timestamp; // sensor stamp, ns relative to GetGlobalTimeOffsetNs() (may be < 0)
        uint32_t seq;      // packet sequence id, one sample per packet
    } ParsedImuData;

    /**
     * @brief Point Cloud Type
     */
    typedef struct
    {
        double stamp;
        uint32_t id;
        uint32_t ringNum;
        std::vector<PointUnitree> points;
    } PointCloudUnitree;

    /**
     * @brief Point Cloud Type
     */
    typedef struct
    {
        uint32_t id;
        uint32_t ringNum;
        std::vector<PointDLidar> points;
    } PointCloudDLidar;

    ///////////////////////////////////////////////////////////////////////////////
    // FUNCTIONS
    ///////////////////////////////////////////////////////////////////////////////

    /**
     * @brief Get system timestamp
     */
    inline void getSystemTimeStamp(TimeStamp &timestamp)
    {
        struct timespec time1 = {0, 0};
        clock_gettime(CLOCK_REALTIME, &time1);
        timestamp.sec = time1.tv_sec;
        timestamp.nsec = time1.tv_nsec;
    }

    /**
     * @brief crc32 check
     */
    inline uint32_t crc32(const uint8_t *buf, uint32_t len)
    {
        uint8_t i;
        uint32_t crc = 0xFFFFFFFF;
        while (len--)
        {
            crc ^= *buf++;
            for (i = 0; i < 8; ++i)
            {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0xEDB88320;
                else
                    crc = (crc >> 1);
            }
        }
        return ~crc;
    }

    /**
     * @brief Centralized IMU parsing and coordinate mapping.
     */
    inline void parseFromImuPacket(ParsedImuData &out, const LidarImuData &imu)
    {
        switch (GetSensorOrientation())
        {
        case SensorOrientation::VERTICAL:
            // -------------------------------------------------------------
            // IMU AXIS MAPPING (Vertical Mount)
            // Physical: +Z Front, +X Down, +Y Left
            // Target: +X Front, +Y Left, +Z Up
            // Math: X_out = Z_phys, Y_out = Y_phys, Z_out = -X_phys
            // -------------------------------------------------------------
            out.accel[0] = imu.linear_acceleration[2] / 9.80665f;
            out.accel[1] = imu.linear_acceleration[1] / 9.80665f;
            out.accel[2] = -imu.linear_acceleration[0] / 9.80665f;

            out.gyro[0] = imu.angular_velocity[2];
            out.gyro[1] = imu.angular_velocity[1];
            out.gyro[2] = -imu.angular_velocity[0];
            break;

        case SensorOrientation::STANDARD:
        default:
            // -------------------------------------------------------------
            // IMU AXIS MAPPING (Standard NWU)
            // -------------------------------------------------------------
            out.accel[0] = imu.linear_acceleration[0] / 9.80665f;
            out.accel[1] = imu.linear_acceleration[1] / 9.80665f;
            out.accel[2] = imu.linear_acceleration[2] / 9.80665f;

            out.gyro[0] = imu.angular_velocity[0];
            out.gyro[1] = imu.angular_velocity[1];
            out.gyro[2] = imu.angular_velocity[2];
            break;
        }

        int64_t absolute_raw_time = ((int64_t)imu.info.stamp.sec * 1000000000LL) + (int64_t)imu.info.stamp.nsec;

        if (GetGlobalTimeOffsetNs() == 0)
        {
            GetGlobalTimeOffsetNs() = absolute_raw_time;
        }

        out.timestamp = absolute_raw_time - GetGlobalTimeOffsetNs();
        out.seq = imu.info.seq;
    }

    /**
     * @brief Parse from a point packet to a 3D point cloud
     */
    inline void parseFromPacketToPointCloud(
        PointCloudDLidar &cloudOut,
        const LidarPointDataPacket &packet,
        float range_min = 0,
        float range_max = 100,
        uint8_t min_intensity = 0)
    {
        // ... [Matrix math remains unchanged] ...
        const float sin_beta = sin(packet.data.param.beta_angle);
        const float cos_beta = cos(packet.data.param.beta_angle);
        const float sin_xi = sin(packet.data.param.xi_angle);
        const float cos_xi = cos(packet.data.param.xi_angle);
        const float cos_beta_sin_xi = cos_beta * sin_xi;
        const float sin_beta_cos_xi = sin_beta * cos_xi;
        const float sin_beta_sin_xi = sin_beta * sin_xi;
        const float cos_beta_cos_xi = cos_beta * cos_xi;

        const int num_of_points = packet.data.point_num;
        const float time_step = packet.data.time_increment;

        cloudOut.id = 1;
        cloudOut.ringNum = 1;
        cloudOut.points.clear();
        cloudOut.points.reserve(num_of_points);

        auto &ranges = packet.data.ranges;
        auto &intensities = packet.data.intensities;

        float time_relative = 0;
        float alpha_cur = packet.data.angle_min + packet.data.param.alpha_angle_bias;
        float alpha_step = packet.data.angle_increment;
        float theta_cur = packet.data.com_horizontal_angle_start + packet.data.param.theta_angle_bias;
        float theta_step = packet.data.com_horizontal_angle_step;

        PointDLidar point3d;
        point3d.ring = 1;

        for (int j = 0; j < num_of_points; j += 1, alpha_cur += alpha_step,
                 theta_cur += theta_step, time_relative += time_step)
        {
            if (ranges[j] < 1)
                continue;

            float range_float = packet.data.param.range_scale * ((float)ranges[j] + packet.data.param.range_bias);

            if (range_float < packet.data.range_min || range_float > packet.data.range_max)
                continue;
            if (range_float < range_min || range_float > range_max)
                continue;
            if (intensities[j] < min_intensity)
                continue;

            float sin_alpha = sin(alpha_cur);
            float cos_alpha = cos(alpha_cur);
            float sin_theta = sin(theta_cur);
            float cos_theta = cos(theta_cur);

            float A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_float + packet.data.param.b_axis_dist;
            float B = cos_alpha * cos_xi * range_float;
            float C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_float;

            float orig_x = cos_theta * A - sin_theta * B;
            float orig_y = sin_theta * A + cos_theta * B;
            float orig_z = C + packet.data.param.a_axis_dist;

            switch (GetSensorOrientation())
            {
            case SensorOrientation::VERTICAL:
                // -------------------------------------------------------------
                // LIDAR AXIS MAPPING (Vertical Mount)
                // Physical: +Z Front, +X Down, +Y Left
                // Target: +X Front, +Y Left, +Z Up
                // Math: X_out = Z_phys, Y_out = Y_phys, Z_out = -X_phys
                // -------------------------------------------------------------
                point3d.x = orig_z;
                point3d.y = orig_y;
                point3d.z = -orig_x;
                break;

            case SensorOrientation::STANDARD:
            default:
                // -------------------------------------------------------------
                // LIDAR AXIS MAPPING (Standard)
                // -------------------------------------------------------------
                point3d.x = orig_x;
                point3d.y = orig_y;
                point3d.z = orig_z;
                break;
            }

            point3d.intensity = intensities[j];

            int64_t packet_time_ns = ((int64_t)packet.data.info.stamp.sec * 1000000000LL) + (int64_t)packet.data.info.stamp.nsec;

            if (GetGlobalTimeOffsetNs() == 0)
            {
                GetGlobalTimeOffsetNs() = packet_time_ns;
            }

            int64_t relative_packet_time_ns = packet_time_ns - GetGlobalTimeOffsetNs();
            double packet_base_time = (double)relative_packet_time_ns / 1.0e9;

            point3d.time = packet_base_time + time_relative;

            cloudOut.points.push_back(point3d);
        }
    }
}