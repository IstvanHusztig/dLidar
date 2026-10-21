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
#include <cmath>
#include <chrono>

#include "unitree_lidar_sdk_config.h"
#include "unitree_lidar_protocol.h"
#include "domain/geometry/point_filter.h"
#include "domain/recording/point_record.h"

namespace unitree_lidar_sdk
{

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

    // dLidar point record (recording format 2), see domain/recording/point_record.h.
    using PointDLidar = dlidar::PointRecord;

    /**
     * @brief One L2 IMU sample in the sensor frame, SI units (m/s^2, rad/s).
     */
    typedef struct
    {
        float gyro[3];
        float accel[3];
        int64_t sensor_stamp_ns; // the packet's L2 stamp (DataInfo.stamp), absolute ns
        uint32_t seq;            // packet sequence id, one sample per packet
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

    inline int64_t StampToNs(const TimeStamp &stamp)
    {
        return (int64_t)stamp.sec * 1000000000LL + (int64_t)stamp.nsec;
    }

    /**
     * @brief IMU packet -> sample in the sensor frame, SI units. The mount and
     * the conversion to g are applied by the core (CaptureTiming).
     */
    inline void parseFromImuPacket(ParsedImuData &out, const LidarImuData &imu)
    {
        for (int i = 0; i < 3; i++)
        {
            out.accel[i] = imu.linear_acceleration[i];
            out.gyro[i] = imu.angular_velocity[i];
        }
        out.sensor_stamp_ns = StampToNs(imu.info.stamp);
        out.seq = imu.info.seq;
    }

    /**
     * @brief Point packet -> points in the sensor frame, times in s relative to
     * `zero_ns`. Samples without a return or outside the packet's range limits
     * are skipped, and so are the ones `filter` rejects (before the trig, to
     * save CPU; the core applies the same filter again).
     */
    inline void parseFromPacketToPointCloud(
        PointCloudDLidar &cloudOut,
        const LidarPointDataPacket &packet,
        int64_t zero_ns,
        const dlidar::PointFilter &filter)
    {
        // ... [Matrix math remains unchanged] ...
        // Float overloads (sinf/cosf), as with <math.h>; unqualified sin() with
        // only <cmath> would compute in double and change the points.
        const float sin_beta = std::sin(packet.data.param.beta_angle);
        const float cos_beta = std::cos(packet.data.param.beta_angle);
        const float sin_xi = std::sin(packet.data.param.xi_angle);
        const float cos_xi = std::cos(packet.data.param.xi_angle);
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

        float alpha_cur = packet.data.angle_min + packet.data.param.alpha_angle_bias;
        float alpha_step = packet.data.angle_increment;
        float theta_cur = packet.data.com_horizontal_angle_start + packet.data.param.theta_angle_bias;
        float theta_step = packet.data.com_horizontal_angle_step;

        // L2 stamp of the packet = time of its first point; point j adds
        // j * time_increment. Never the host arrival time (the SDK's
        // use_system_timestamp does not touch this packet).
        const int64_t packet_time_ns = StampToNs(packet.data.info.stamp);
        const double packet_base_time = (double)(packet_time_ns - zero_ns) / 1.0e9;

        PointDLidar point3d;
        point3d.ring = 1;

        for (int j = 0; j < num_of_points; j += 1, alpha_cur += alpha_step,
                 theta_cur += theta_step)
        {
            if (ranges[j] < 1)
                continue;

            float range_float = packet.data.param.range_scale * ((float)ranges[j] + packet.data.param.range_bias);

            if (range_float < packet.data.range_min || range_float > packet.data.range_max)
                continue;
            if (!filter.Accepts(range_float, (float)intensities[j]))
                continue;

            float sin_alpha = std::sin(alpha_cur);
            float cos_alpha = std::cos(alpha_cur);
            float sin_theta = std::sin(theta_cur);
            float cos_theta = std::cos(theta_cur);

            float A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_float + packet.data.param.b_axis_dist;
            float B = cos_alpha * cos_xi * range_float;
            float C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_float;

            float orig_x = cos_theta * A - sin_theta * B;
            float orig_y = sin_theta * A + cos_theta * B;
            float orig_z = C + packet.data.param.a_axis_dist;

            point3d.x = orig_x;
            point3d.y = orig_y;
            point3d.z = orig_z;

            point3d.intensity = intensities[j];

            point3d.time = packet_base_time + (double)j * (double)time_step;

            point3d.range = range_float;
            point3d.alpha = alpha_cur;
            point3d.theta = theta_cur;
            point3d.packet_seq = packet.data.info.seq;
            point3d.raw_range = ranges[j];
            point3d.sample_index = (uint16_t)j;

            cloudOut.points.push_back(point3d);
        }
    }
}