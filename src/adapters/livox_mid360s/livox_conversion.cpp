#include "adapters/livox_mid360s/livox_conversion.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace dlidar::livox
{
    namespace
    {
        size_t RecordSize(uint8_t data_type)
        {
            switch (data_type)
            {
            case kLivoxLidarImuData:
                return sizeof(LivoxLidarImuRawPoint);
            case kLivoxLidarCartesianCoordinateHighData:
                return sizeof(LivoxLidarCartesianHighRawPoint);
            case kLivoxLidarCartesianCoordinateLowData:
                return sizeof(LivoxLidarCartesianLowRawPoint);
            case kLivoxLidarSphericalCoordinateData:
                return sizeof(LivoxLidarSpherPoint);
            default:
                return 0;
            }
        }
    }

    PacketStatus CheckPacket(const uint8_t *data, size_t size)
    {
        if (size < kHeaderSize)
            return PacketStatus::TooShort;
        LivoxLidarEthernetPacket h = Header(data);
        size_t record = RecordSize(h.data_type);
        if (record == 0)
            return PacketStatus::UnknownType;
        // An IMU packet holds one sample.
        size_t records = h.data_type == kLivoxLidarImuData ? 1 : h.dot_num;
        return size >= kHeaderSize + records * record ? PacketStatus::Ok : PacketStatus::TooShort;
    }

    LivoxLidarEthernetPacket Header(const uint8_t *data)
    {
        LivoxLidarEthernetPacket h;
        memcpy(&h, data, kHeaderSize);
        h.data[0] = 0;
        return h;
    }

    int64_t StampNs(const LivoxLidarEthernetPacket &header)
    {
        uint64_t ns = 0;
        for (int i = 7; i >= 0; i--) // little-endian on the wire
            ns = ns << 8 | header.timestamp[i];
        return (int64_t)ns;
    }

    double PointStepS(const LivoxLidarEthernetPacket &header)
    {
        return header.dot_num > 1 ? (double)header.time_interval * 1e-7 / (double)(header.dot_num - 1) : 0.0;
    }

    int64_t DurationNs(const LivoxLidarEthernetPacket &header)
    {
        return header.dot_num > 1 ? (int64_t)header.time_interval * 100 : 0;
    }

    void ParseImu(const uint8_t *data, ImuReading &out)
    {
        LivoxLidarImuRawPoint s;
        memcpy(&s, data + kHeaderSize, sizeof s);
        out.gyro_rads[0] = s.gyro_x; // rad/s as sent
        out.gyro_rads[1] = s.gyro_y;
        out.gyro_rads[2] = s.gyro_z;
        out.accel_mps2[0] = s.acc_x * 9.80665f; // g as sent; the port is m/s^2
        out.accel_mps2[1] = s.acc_y * 9.80665f;
        out.accel_mps2[2] = s.acc_z * 9.80665f;
        out.stamp_ns = StampNs(Header(data));
    }

    void ParsePoints(const uint8_t *data, int64_t zero_ns, const PointFilter &filter, bool drop_low_confidence,
                     std::vector<PointRecord> &out)
    {
        const LivoxLidarEthernetPacket h = Header(data);
        // The packet stamp is the time of its first point; point i adds i * step.
        const double base_time = (double)(StampNs(h) - zero_ns) / 1.0e9;
        const double step = PointStepS(h);
        const uint32_t seq = PacketSeq(h);
        out.reserve(out.size() + h.dot_num);

        PointRecord p{};
        p.ring = 1;
        for (int i = 0; i < h.dot_num; i++)
        {
            LivoxLidarCartesianHighRawPoint q;
            memcpy(&q, data + kHeaderSize + (size_t)i * sizeof q, sizeof q);
            const double x = q.x, y = q.y, z = q.z; // mm
            const double range_mm = std::sqrt(x * x + y * y + z * z);
            const float range = (float)(range_mm * 0.001);
            if (!filter.Accepts(range, (float)q.reflectivity))
                continue;
            if (drop_low_confidence && LowConfidence(q.tag))
                continue;

            p.x = (float)(x * 0.001);
            p.y = (float)(y * 0.001);
            p.z = (float)(z * 0.001);
            p.intensity = q.reflectivity;
            p.time = base_time + (double)i * step;
            p.range = range;
            p.raw_range = (uint16_t)std::min(65535.0, std::round(range_mm));
            p.alpha = (float)std::atan2(y, x);
            p.theta = (float)std::atan2(z, std::hypot(x, y));
            p.packet_seq = seq;
            p.sample_index = (uint16_t)i;
            out.push_back(p);
        }
    }
}
