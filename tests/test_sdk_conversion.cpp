// Checks that dLidar's point path (CaptureTiming::OnPoints: VERTICAL remap,
// range/intensity filters) gives the same xyz as the vendor SDK's compiled
// conversion (UnitreeLidarReader::getPointCloud in libunitree_lidar_sdk.a),
// with the factory calibration applied. Packets with known contents are sent
// to the SDK over loopback UDP, so no sensor is needed. Also prints how far
// the calibration moves the points (docs/l2_point_conversion.md).
#include "unitree_lidar_sdk.h"
#include "capture_timing.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <cstring>
#include <map>
#include <random>

using namespace unitree_lidar_sdk;

static int failures = 0;
#define CHECK(cond, ...)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(cond))                                                                                                   \
        {                                                                                                              \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                                                                \
            printf(__VA_ARGS__);                                                                                       \
            printf("\n");                                                                                              \
            failures++;                                                                                                \
        }                                                                                                              \
    } while (0)

// float32 rounding differs by compiler flags (dLidar is built -O0, the SDK
// optimized); 10 um is far below the LAZ resolution of 0.1 mm.
const double kMaxDiffM = 1e-5;

static void FillPacket(LidarPointDataPacket &pk, const LidarCalibParam &cal, uint32_t seq, std::mt19937 &rng)
{
    memset(&pk, 0, sizeof pk);
    const uint8_t head[4] = {0x55, 0xAA, 0x05, 0x0A};
    memcpy(pk.header.header, head, 4);
    pk.header.packet_type = LIDAR_POINT_DATA_PACKET_TYPE;
    pk.header.packet_size = sizeof(LidarPointDataPacket);
    LidarPointData &d = pk.data;
    d.info.seq = seq;
    d.info.payload_size = sizeof(LidarPointData);
    d.info.stamp.sec = 1791547057;
    d.info.stamp.nsec = 721602864 + seq * 4509320;
    d.param = cal;
    std::uniform_real_distribution<float> u(0, 1);
    d.com_horizontal_angle_start = 6.283f * u(rng);
    d.com_horizontal_angle_step = 0.000267f;
    d.scan_period = 0.18f;
    d.range_min = 0.05f;
    d.range_max = 50.f;
    d.angle_min = -0.2f + 0.4f * u(rng);
    d.angle_increment = 0.0104719754f;
    d.time_increment = 7.67e-6f;
    d.point_num = 300;
    std::uniform_int_distribution<int> range_mm(0, 45000), intensity(0, 255);
    for (int j = 0; j < 300; j++)
    {
        d.ranges[j] = (j % 37 == 0) ? 0 : (uint16_t)range_mm(rng);
        d.intensities[j] = (uint8_t)intensity(rng);
    }
    pk.tail.tail[1] = 0xFF;
    pk.tail.crc32 = crc32((const uint8_t *)&pk.data, sizeof(LidarPointData)); // the SDK checks the data only
}

// The vendor model in double precision, STANDARD frame.
static void ModelDouble(const LidarPointData &d, const LidarCalibParam &c, int j, double out[3])
{
    double a = (double)d.angle_min + c.alpha_angle_bias + j * (double)d.angle_increment;
    double t = (double)d.com_horizontal_angle_start + c.theta_angle_bias + j * (double)d.com_horizontal_angle_step;
    double r = (double)c.range_scale * ((double)d.ranges[j] + c.range_bias);
    double sb = sin(c.beta_angle), cb = cos(c.beta_angle), sx = sin(c.xi_angle), cx = cos(c.xi_angle);
    double A = (-cb * sx + sb * cx * sin(a)) * r + c.b_axis_dist;
    double B = cos(a) * cx * r;
    double C = (sb * sx + cb * cx * sin(a)) * r;
    out[0] = cos(t) * A - sin(t) * B;
    out[1] = sin(t) * A + cos(t) * B;
    out[2] = C + c.a_axis_dist;
}

static double Dist(double ax, double ay, double az, double bx, double by, double bz)
{
    return std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (az - bz) * (az - bz));
}

int main()
{
    const unsigned short kLidarPort = 16101, kLocalPort = 16201;
    UnitreeLidarReader *reader = createUnitreeLidarReader();
    if (reader->initializeUDP(kLidarPort, "127.0.0.1", kLocalPort, "127.0.0.1", 1, false, 0, 100))
    {
        printf("FAIL: SDK initializeUDP on loopback\n");
        return 1;
    }
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in from{}, to{};
    from.sin_family = to.sin_family = AF_INET;
    from.sin_port = htons(kLidarPort);
    to.sin_port = htons(kLocalPort);
    inet_pton(AF_INET, "127.0.0.1", &from.sin_addr);
    inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    bind(sock, (sockaddr *)&from, sizeof from);

    GetSensorOrientation() = SensorOrientation::VERTICAL; // as src/dLidar.cpp
    struct Calib
    {
        const char *name;
        LidarCalibParam p; // a, b, theta, alpha, beta, xi, range_bias, range_scale
    };
    const Calib calibs[] = {
        {"none", {0, 0, 0, 0, 0, 0, 0, 0.001f}},
        {"small", {0.0076f, -0.0021f, 0.0015f, -0.0021f, 0.0035f, -0.0012f, 2.5f, 0.001f}},
        {"large", {0.05f, -0.03f, 0.02f, -0.03f, 0.04f, -0.025f, -40.f, 0.00102f}},
    };
    const LidarCalibParam kNone = calibs[0].p;

    std::mt19937 rng(42);
    uint32_t seq = 1;
    for (const Calib &cal : calibs)
    {
        double max_sdk_vs_dlidar = 0, max_float_vs_double = 0, max_cal_effect_per_m = 0;
        size_t sdk_points = 0, dlidar_points = 0;
        for (int n = 0; n < 4; n++, seq++)
        {
            LidarPointDataPacket sent;
            FillPacket(sent, cal.p, seq, rng);

            // runParse blocks up to the SDK's 1 s receive timeout.
            PointCloudUnitree sdk_cloud;
            bool got = false;
            for (int tries = 0; tries < 3 && !got; tries++)
            {
                sendto(sock, &sent, sizeof sent, 0, (sockaddr *)&to, sizeof to);
                for (int k = 0; k < 3 && !got; k++)
                    if (reader->runParse() == LIDAR_POINT_DATA_PACKET_TYPE)
                        got = reader->getPointCloud(sdk_cloud);
            }
            CHECK(got, "SDK returned no cloud for seq %u", seq);
            if (!got)
                continue;
            const LidarPointDataPacket &parsed = reader->getLidarPointDataPacket();
            CHECK(memcmp(&parsed.data, &sent.data, sizeof(LidarPointData)) == 0, "SDK packet differs from sent");

            CaptureTiming timing;
            std::vector<PointDLidar> dlidar;
            std::vector<OutputImuData> imu;
            timing.OnPoints(parsed, HostNow(), dlidar, imu);

            // The SDK keeps samples in order, skipping ranges[j] < 1 and
            // ranges outside the packet's and its own [0, 100] m limits.
            const LidarPointData &d = parsed.data;
            std::map<int, PointUnitree> sdk_by_j;
            size_t i = 0;
            for (int j = 0; j < (int)d.point_num && i < sdk_cloud.points.size(); j++)
            {
                float r = d.param.range_scale * ((float)d.ranges[j] + d.param.range_bias);
                if (d.ranges[j] < 1 || r < d.range_min || r > d.range_max || r < 0 || r > 100)
                    continue;
                sdk_by_j[j] = sdk_cloud.points[i++];
            }
            CHECK(i == sdk_cloud.points.size(), "SDK cloud has %zu points, expected %zu", sdk_cloud.points.size(), i);

            size_t matched = 0;
            for (const PointDLidar &p : dlidar)
            {
                auto it = sdk_by_j.find(p.sample_index);
                CHECK(it != sdk_by_j.end(), "dLidar point j=%u is not in the SDK cloud", p.sample_index);
                if (it == sdk_by_j.end())
                    continue;
                matched++;
                // Undo the VERTICAL remap: x = orig_z, y = orig_y, z = -orig_x.
                double ox = -p.z, oy = p.y, oz = p.x;
                const PointUnitree &s = it->second;
                max_sdk_vs_dlidar = std::max(max_sdk_vs_dlidar, Dist(s.x, s.y, s.z, ox, oy, oz));
                CHECK(p.intensity == s.intensity, "intensity differs at j=%u", p.sample_index);
                double m[3], z[3];
                ModelDouble(d, cal.p, p.sample_index, m);
                ModelDouble(d, kNone, p.sample_index, z);
                max_float_vs_double = std::max(max_float_vs_double, Dist(m[0], m[1], m[2], ox, oy, oz));
                max_cal_effect_per_m =
                    std::max(max_cal_effect_per_m, Dist(m[0], m[1], m[2], z[0], z[1], z[2]) / std::max(1.0, (double)p.range));
            }
            // Every SDK point dLidar lacks must be one of its own filters.
            for (auto &kv : sdk_by_j)
            {
                bool kept = false;
                for (const PointDLidar &p : dlidar)
                    kept |= p.sample_index == kv.first;
                float r = d.param.range_scale * ((float)d.ranges[kv.first] + d.param.range_bias);
                CHECK(kept || d.intensities[kv.first] < kMinPointIntensity || r < kMinPointRangeM,
                      "SDK point j=%d (I=%u, r=%.3f) dropped by dLidar", kv.first, d.intensities[kv.first], r);
            }
            sdk_points += sdk_cloud.points.size();
            dlidar_points += matched;
        }
        CHECK(max_sdk_vs_dlidar <= kMaxDiffM, "calibration %s: |SDK - dLidar| = %.3g m", cal.name, max_sdk_vs_dlidar);
        printf("calibration %-5s: SDK %zu points, dLidar %zu (rest removed by kMinPointIntensity/kMinPointRangeM); "
               "max |SDK - dLidar| = %.2g m, float32 vs double model %.2g m; calibration moves points up to %.1f mm/m\n",
               cal.name, sdk_points, dlidar_points, max_sdk_vs_dlidar, max_float_vs_double, 1000 * max_cal_effect_per_m);
    }
    // No reader->closeUDP(): in SDK 2.0.4 it recurses until the stack overflows.
    close(sock);
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
