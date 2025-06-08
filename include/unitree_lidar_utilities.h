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
#include <iostream>
#include <chrono>

#include "unitree_lidar_sdk_config.h"
#include "unitree_lidar_protocol.h"

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
    uint32_t ring; // the ring number indicates which channel of the sensor that this point belongs to
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
    uint32_t ring; // the ring number indicates which channel of the sensor that this point belongs to
} PointDLidar;

/**
 * @brief Point Cloud Type
 */
typedef struct
{
    double stamp;     // cloud start timestamp, the point timestamp is relative to this
    uint32_t id;      // sequence id
    uint32_t ringNum; // number of rings
    std::vector<PointUnitree> points;
} PointCloudUnitree;

/**
 * @brief Point Cloud Type
 */
typedef struct
{
    uint32_t id;      // sequence id
    uint32_t ringNum; // number of rings
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
 * @param buf
 * @param len
 * @return uint32_t
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
 * @brief Parse from a point packet to a 3D point cloud
 * @param[out] cloud
 * @param[in] packet lidar point data packet
 * @param[in] use_system_timestamp use system timestamp, otherwise use lidar hardware timestamp
 * @param[in] range_min allowed minimum point range in meters
 * @param[in] range_max allowed maximum point range in meters
 */
inline void parseFromPacketToPointCloud(
    PointCloudDLidar &cloudOut,
    const LidarPointDataPacket &packet,
    float range_min = 0,
    float range_max = 100)
{
    

    // intermediate variables
    const float sin_beta = sin(packet.data.param.beta_angle);
    const float cos_beta = cos(packet.data.param.beta_angle);
    const float sin_xi = sin(packet.data.param.xi_angle);
    const float cos_xi = cos(packet.data.param.xi_angle);
    const float cos_beta_sin_xi = cos_beta * sin_xi;
    const float sin_beta_cos_xi = sin_beta * cos_xi;
    const float sin_beta_sin_xi = sin_beta * sin_xi;
    const float cos_beta_cos_xi = cos_beta * cos_xi;
 
    // scan info
    const int num_of_points = packet.data.point_num;
    const float time_step = packet.data.time_increment;

    cloudOut.id = 1;
    cloudOut.ringNum = 1;
    cloudOut.points.clear();
    cloudOut.points.reserve(num_of_points);

    // transform raw data to a pointcloud
    auto &ranges = packet.data.ranges;
    auto &intensities = packet.data.intensities;

    float time_relative = 0;
    float alpha_cur = packet.data.angle_min + packet.data.param.alpha_angle_bias;
    float alpha_step = packet.data.angle_increment;
    float theta_cur = packet.data.com_horizontal_angle_start + packet.data.param.theta_angle_bias;
    float theta_step = packet.data.com_horizontal_angle_step;

    float range_float;
    float sin_alpha, cos_alpha, sin_theta, cos_theta;
    float A, B, C;

    PointDLidar point3d;
    point3d.ring = 1;
    // std::cout << "packet.data.param.range_scale = " << packet.data.param.range_scale << std::endl;

    for (int j = 0; j < num_of_points; j += 1, alpha_cur += alpha_step,
             theta_cur += theta_step, time_relative += time_step)
    {
        // jump invalid points
        if (ranges[j] < 1)
        {
            continue;
        }

        // calculate point range in float type
        range_float = packet.data.param.range_scale * ((float)ranges[j] + packet.data.param.range_bias);

        // jump points beyond range limit
        if ( range_float < packet.data.range_min || range_float > packet.data.range_max)
        {
            continue;
        }

        // jump points beyond range limit
        if (range_float < range_min || range_float > range_max)
        {
            continue;
        }

        // transform to XYZ coordinate
        sin_alpha = sin(alpha_cur);
        cos_alpha = cos(alpha_cur);
        sin_theta = sin(theta_cur);
        cos_theta = cos(theta_cur);

        A = (-cos_beta_sin_xi + sin_beta_cos_xi * sin_alpha) * range_float + packet.data.param.b_axis_dist;
        B = cos_alpha * cos_xi * range_float;
        C = (sin_beta_sin_xi + cos_beta_cos_xi * sin_alpha) * range_float;

        point3d.x = cos_theta * A - sin_theta * B;
        point3d.y = sin_theta * A + cos_theta * B;
        point3d.z = C + packet.data.param.a_axis_dist;

        // push back this point to cloud
        point3d.intensity = intensities[j];
        point3d.time = (packet.data.info.stamp.sec + packet.data.info.stamp.nsec/1.0e6)/1.0e9;
        cloudOut.points.push_back(point3d);
    }
}

}