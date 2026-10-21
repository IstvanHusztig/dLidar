#include "adapters/unitree_l2/l2_conversion.h"

#include <cmath>

namespace dlidar::l2
{
    using unitree_lidar_sdk::LidarImuData;
    using unitree_lidar_sdk::LidarPointDataPacket;

    void ParseImu(const LidarImuData &imu, ImuReading &out)
    {
        for (int i = 0; i < 3; i++)
        {
            out.accel_mps2[i] = imu.linear_acceleration[i];
            out.gyro_rads[i] = imu.angular_velocity[i];
        }
        out.stamp_ns = StampToNs(imu.info.stamp);
        out.seq = imu.info.seq;
    }

    void ParsePoints(const LidarPointDataPacket &packet, int64_t zero_ns, const PointFilter &filter,
                     std::vector<PointRecord> &out)
    {
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

        out.reserve(out.size() + num_of_points);

        auto &ranges = packet.data.ranges;
        auto &intensities = packet.data.intensities;

        // Accumulated step by step over every sample, filtered or not (never
        // angle_min + j * step, which rounds differently).
        float alpha_cur = packet.data.angle_min + packet.data.param.alpha_angle_bias;
        float alpha_step = packet.data.angle_increment;
        float theta_cur = packet.data.com_horizontal_angle_start + packet.data.param.theta_angle_bias;
        float theta_step = packet.data.com_horizontal_angle_step;

        // L2 stamp of the packet = time of its first point; point j adds
        // j * time_increment. Never the host arrival time (the SDK's
        // use_system_timestamp does not touch this packet).
        const int64_t packet_time_ns = StampToNs(packet.data.info.stamp);
        const double packet_base_time = (double)(packet_time_ns - zero_ns) / 1.0e9;

        PointRecord point3d;
        point3d.ring = 1;

        for (int j = 0; j < num_of_points; j += 1, alpha_cur += alpha_step, theta_cur += theta_step)
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

            point3d.x = cos_theta * A - sin_theta * B;
            point3d.y = sin_theta * A + cos_theta * B;
            point3d.z = C + packet.data.param.a_axis_dist;

            point3d.intensity = intensities[j];

            point3d.time = packet_base_time + (double)j * (double)time_step;

            point3d.range = range_float;
            point3d.alpha = alpha_cur;
            point3d.theta = theta_cur;
            point3d.packet_seq = packet.data.info.seq;
            point3d.raw_range = ranges[j];
            point3d.sample_index = (uint16_t)j;

            out.push_back(point3d);
        }
    }
}
