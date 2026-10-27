#pragma once

#include "adapters/unitree_l2/vendor/unitree_lidar_protocol.h"
#include "domain/recording/recording_profile.h"

#include <cinttypes>
#include <cstdio>
#include <string>

namespace dlidar::l2
{
using unitree_lidar_sdk::LidarCalibParam;
using unitree_lidar_sdk::LidarInsideState;
using unitree_lidar_sdk::LidarPointData;

// One row of packetstateNNNN.tsv: the fields of a point packet other than its
// ranges and intensities, as received (recording format 2,
// docs/l2-recording-format.md).
struct PointPacketState
{
    int64_t l2_stamp_ns; // DataInfo.stamp, absolute L2 clock
    uint32_t seq;
    uint32_t point_num;
    float time_increment;
    float scan_period;
    float range_min;
    float range_max;
    LidarInsideState state;
    LidarCalibParam param;
    float angle_min;
    float angle_increment;
    float com_horizontal_angle_start;
    float com_horizontal_angle_step;
};

inline PointPacketState MakePointPacketState(const LidarPointData &d)
{
    PointPacketState s;
    s.l2_stamp_ns = (int64_t)d.info.stamp.sec * 1000000000LL + (int64_t)d.info.stamp.nsec;
    s.seq = d.info.seq;
    s.point_num = d.point_num;
    s.time_increment = d.time_increment;
    s.scan_period = d.scan_period;
    s.range_min = d.range_min;
    s.range_max = d.range_max;
    s.state = d.state;
    s.param = d.param;
    s.angle_min = d.angle_min;
    s.angle_increment = d.angle_increment;
    s.com_horizontal_angle_start = d.com_horizontal_angle_start;
    s.com_horizontal_angle_step = d.com_horizontal_angle_step;
    return s;
}

constexpr const char *kPacketStateHeader =
    "stamp_ns\tl2_stamp_ns\tseq\tpoint_num\ttime_increment\tscan_period\trange_min\trange_max\t"
    "apd_temperature\tapd_voltage\tlaser_voltage\timu_temperature\tdirty_index\tpacket_lost_up\tpacket_lost_down\t"
    "sys_rotation_period\tcom_rotation_period\t"
    "range_scale\trange_bias\ta_axis_dist\tb_axis_dist\ttheta_angle_bias\talpha_angle_bias\tbeta_angle\txi_angle\t"
    "angle_min\tangle_increment\tcom_horizontal_angle_start\tcom_horizontal_angle_step\n";

// What the L2's recordings say about it (src/domain/recording/recording_profile.h).
constexpr RecordingProfile kRecordingProfile{
    kPreFormat3SensorId,
    "unitree_l2",
    "Unitree L2",
    {"range_scale*(raw+bias) [m]", "range as sent by the L2 [mm]", "index of point in packet", "point packet info.seq",
     "alpha scan angle incl bias, rad", "theta scan angle incl bias, rad"},
    kPacketStateHeader,
};
static_assert(FitsLas(kRecordingProfile), "LAS texts are at most 31 characters");

// One tab-separated row. stamp_ns is relative to `zero_ns` (the zero of the
// LAZ gps_time and the IMU timestamp). Floats are printed with 9 significant
// digits, which reproduces the received float32 exactly.
inline void AppendPacketStateRow(std::string &out, const PointPacketState &s, int64_t zero_ns)
{
    char row[1024];
    int n = snprintf(row, sizeof row,
                     "%" PRId64 "\t%" PRId64 "\t%u\t%u\t%.9g\t%.9g\t%.9g\t%.9g\t"
                     "%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%u\t%u\t"
                     "%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t"
                     "%.9g\t%.9g\t%.9g\t%.9g\n",
                     s.l2_stamp_ns - zero_ns, s.l2_stamp_ns, s.seq, s.point_num, (double)s.time_increment,
                     (double)s.scan_period, (double)s.range_min, (double)s.range_max,
                     (double)s.state.apd_temperature, (double)s.state.apd_voltage, (double)s.state.laser_voltage,
                     (double)s.state.imu_temperature, (double)s.state.dirty_index, (double)s.state.packet_lost_up,
                     (double)s.state.packet_lost_down, s.state.sys_rotation_period, s.state.com_rotation_period,
                     (double)s.param.range_scale, (double)s.param.range_bias, (double)s.param.a_axis_dist,
                     (double)s.param.b_axis_dist, (double)s.param.theta_angle_bias, (double)s.param.alpha_angle_bias,
                     (double)s.param.beta_angle, (double)s.param.xi_angle,
                     (double)s.angle_min, (double)s.angle_increment, (double)s.com_horizontal_angle_start,
                     (double)s.com_horizontal_angle_step);
    if (n > 0 && n < (int)sizeof row)
        out.append(row, (size_t)n);
}

} // namespace dlidar::l2
