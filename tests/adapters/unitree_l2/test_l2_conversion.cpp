// The L2 conversion (l2::ParsePoints): the format 2 fields and the filters.
#include "adapters/unitree_l2/l2_conversion.h"
#include "../../check.h"

#include <cstdio>

using namespace dlidar;
using namespace unitree_lidar_sdk;

namespace
{
// The parser fills the format 2 fields with the values it used for the point,
// and keeps the index of filtered points out of the others.
void TestParseFillsExtraFields()
{
    printf("parse fills range, raw_range, sample_index, packet_seq, alpha, theta\n");
    LidarPointDataPacket pkt{};
    LidarPointData &d = pkt.data;
    d.info.seq = 777;
    d.info.stamp = TimeStamp{100, 0};
    d.param.range_scale = 0.001f;
    d.param.range_bias = 3.0f;
    d.param.alpha_angle_bias = 0.01f;
    d.param.theta_angle_bias = -0.02f;
    d.range_min = 0;
    d.range_max = 100;
    d.angle_min = -0.5f;
    d.angle_increment = 0.002f;
    d.com_horizontal_angle_start = 1.0f;
    d.com_horizontal_angle_step = 0.0003f;
    d.time_increment = 7.7e-6f;
    d.point_num = 300;
    for (int j = 0; j < 300; j++)
    {
        d.ranges[j] = (uint16_t)(j % 7 == 0 ? 0 : 1000 + 10 * j); // every 7th point has no return
        d.intensities[j] = (uint8_t)(j % 11 == 0 ? 2 : 100);      // every 11th is below the intensity floor
    }
    std::vector<PointRecord> points;
    l2::ParsePoints(pkt, l2::StampToNs(d.info.stamp), PointFilter{0.1f, 100, 5}, points);

    size_t expected = 0;
    for (int j = 0; j < 300; j++)
        expected += j % 7 != 0 && j % 11 != 0;
    CHECK(points.size() == expected, "%zu points, expected %zu", points.size(), expected);
    float alpha = d.angle_min + d.param.alpha_angle_bias, theta = d.com_horizontal_angle_start + d.param.theta_angle_bias;
    size_t k = 0;
    for (int j = 0; j < 300 && k < points.size(); j++, alpha += d.angle_increment, theta += d.com_horizontal_angle_step)
    {
        if (j % 7 == 0 || j % 11 == 0)
            continue;
        const PointRecord &p = points[k++];
        float range = d.param.range_scale * ((float)d.ranges[j] + d.param.range_bias);
        CHECK(p.sample_index == j && p.raw_range == d.ranges[j] && p.packet_seq == 777 && p.range == range &&
                  p.alpha == alpha && p.theta == theta && p.ring == 1,
              "point j=%d: index %u raw %u seq %u range %g alpha %g theta %g", j, p.sample_index, p.raw_range,
              p.packet_seq, p.range, p.alpha, p.theta);
        CHECK(p.time == (double)j * (double)d.time_increment, "point j=%d time %.12f", j, p.time);
    }
}

} // namespace

int main()
{
    TestParseFillsExtraFields();
    return FinishTests("L2 conversion");
}
