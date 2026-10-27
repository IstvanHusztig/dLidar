#include "adapters/livox_mid360s/synthetic_livox_source.h"

#include "adapters/livox_mid360s/livox_packet_builder.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace dlidar::livox
{
    namespace
    {
        uint32_t Hash(uint64_t a)
        {
            a ^= a >> 33; a *= 0xff51afd7ed558ccdULL; a ^= a >> 33; a *= 0xc4ceb9fe1a85ec53ULL; a ^= a >> 33;
            return (uint32_t)a;
        }

        const int kPointsPerPacket = 96;

        // Ray g of the scan pattern hits a box room around the sensor (sensor
        // frame; mounted like the L2, its +X points down).
        LivoxLidarCartesianHighRawPoint Ray(uint64_t g)
        {
            const double kGolden = 0.6180339887498949;
            double az = std::fmod((double)g * 2 * M_PI * kGolden, 2 * M_PI);
            double el = (-7.0 + 59.0 * std::fmod((double)g * kGolden * kGolden + 0.5, 1.0)) * M_PI / 180.0;
            double D[3] = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
            const double lo[3] = {-1.6, -2.5, -4.0}, hi[3] = {1.2, 3.0, 6.0};
            double r = 1e9;
            int axis = 0;
            for (int k = 0; k < 3; k++)
            {
                if (std::fabs(D[k]) < 1e-12)
                    continue;
                double tk = (D[k] > 0 ? hi[k] : lo[k]) / D[k];
                if (tk > 0 && tk < r)
                    r = tk, axis = k;
            }
            uint32_t h = Hash(g);
            LivoxLidarCartesianHighRawPoint p{};
            if (r > 40 || h % 50 == 0) // no return: the sensor sends zeros
                return p;
            r += ((double)(h % 7) - 3.0) * 0.001;
            p.x = (int32_t)std::lround(D[0] * r * 1000);
            p.y = (int32_t)std::lround(D[1] * r * 1000);
            p.z = (int32_t)std::lround(D[2] * r * 1000);
            double reflect = axis == 0 ? (D[0] > 0 ? 30 : 80) : 60; // floor, ceiling, walls
            p.reflectivity = (uint8_t)std::min(255.0, reflect * std::fabs(D[axis]) + (double)(h >> 8 & 7));
            p.tag = h % 97 == 0 ? 0x02 : 0; // a few low-confidence points
            return p;
        }
    }

    SyntheticLivoxSource::SyntheticLivoxSource(std::vector<PacketTiming> log, double seconds)
        : log_(std::move(log)), seconds_(seconds)
    {
    }

    int SyntheticLivoxSource::Next(HostTime &arrival)
    {
        if (next_ >= log_.size())
            return kEndOfStream;
        const PacketTiming &t = log_[next_];
        if ((t.host_mono_ns - log_[0].host_mono_ns) / 1e9 > seconds_)
            return kEndOfStream;
        next_++;
        arrival = {t.host_mono_ns, t.host_unix_ns};
        if (t.type == kPacketTypeImu)
        {
            uint32_t h = Hash((uint64_t)t.sensor_ns);
            auto n = [&](int s) { return (float)(((double)((h >> s) & 0xff) - 127.5) / 127.5); };
            // At rest; +X down, so the accelerometer feels +1 g along -X.
            LivoxLidarImuRawPoint s{0.002f * n(0), 0.002f * n(8), 0.002f * n(16), -1.0f + 0.002f * n(24),
                                    0.002f * n(4), 0.002f * n(12)};
            packet_ = MakeImuPacket(t.sensor_ns, (uint16_t)t.seq, s);
            return kPacket;
        }
        std::vector<LivoxLidarCartesianHighRawPoint> pts(t.point_num ? kPointsPerPacket : 0);
        for (LivoxLidarCartesianHighRawPoint &p : pts)
            p = Ray(points_made_++);
        // The log's point spacing over 96 points, in 0.1 us.
        double interval = std::round((double)t.time_increment * (kPointsPerPacket - 1) * 1e7);
        packet_ = MakePointPacket(t.sensor_ns, (uint8_t)(t.seq >> 16), (uint16_t)t.seq,
                                  (uint16_t)std::min(65535.0, std::max(0.0, interval)), pts);
        return kPacket;
    }

    RecordingInfo SyntheticLivoxSource::Versions()
    {
        RecordingInfo info;
        info.lidar_firmware_version = "replay (synthetic packets)";
        info.lidar_hardware_version = "replay (synthetic packets)";
        info.sdk_version = std::to_string(LIVOX_LIDAR_SDK_MAJOR_VERSION) + "." + std::to_string(LIVOX_LIDAR_SDK_MINOR_VERSION) +
                           "." + std::to_string(LIVOX_LIDAR_SDK_PATCH_VERSION);
        info.work_mode = kLivoxLidarNormal;
        return info;
    }
}
