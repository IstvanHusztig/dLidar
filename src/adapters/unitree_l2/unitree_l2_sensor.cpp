#include "adapters/unitree_l2/unitree_l2_sensor.h"

#include "adapters/unitree_l2/l2_conversion.h"

namespace dlidar
{
    UnitreeL2Sensor::UnitreeL2Sensor(std::unique_ptr<L2PacketSource> source) : source_(std::move(source)) {}

    SensorInfo UnitreeL2Sensor::Info()
    {
        SensorInfo info;
        info.versions = source_->Versions();
        info.versions.sensor_model = l2::kRecordingProfile.sensor_model; // the L2's IMU position is not known
        info.default_filter = l2::kDefaultFilter;
        return info;
    }

    Polled UnitreeL2Sensor::Poll()
    {
        Polled p;
        HostTime now{};
        int type = source_->Next(now);
        if (type == L2PacketSource::kEndOfStream)
        {
            p.kind = PacketKind::EndOfStream;
            return p;
        }
        if (type == LIDAR_IMU_DATA_PACKET_TYPE)
        {
            const unitree_lidar_sdk::LidarImuData &d = source_->ImuPacket();
            l2::ParseImu(d, imu_);
            has_imu_ = true;
            p.kind = PacketKind::Imu;
            p.record = {now.mono_ns, now.unix_ns, imu_.stamp_ns, imu_.seq, kPacketTypeImu, 0.0f, 0};
        }
        else if (type == LIDAR_POINT_DATA_PACKET_TYPE)
        {
            const unitree_lidar_sdk::LidarPointData &d = source_->PointPacket().data;
            states_.push_back(l2::MakePointPacketState(d)); // empty packets too
            points_.first_ns = l2::StampToNs(d.info.stamp);
            points_.duration_ns = PacketDurationNs(d.point_num, d.time_increment);
            points_.seq = d.info.seq;
            points_.count = d.point_num;
            p.kind = PacketKind::Points;
            p.record = {now.mono_ns, now.unix_ns, points_.first_ns, d.info.seq, kPacketTypePoints, d.time_increment,
                        d.point_num};
        }
        return p;
    }

    void UnitreeL2Sensor::TakeImu(std::vector<ImuReading> &out)
    {
        if (has_imu_)
            out.push_back(imu_);
        has_imu_ = false;
    }

    void UnitreeL2Sensor::ConvertPoints(int64_t zero_ns, const PointFilter &filter, std::vector<PointRecord> &out)
    {
        l2::ParsePoints(source_->PointPacket(), zero_ns, filter, out);
    }

    void UnitreeL2Sensor::TakeRows(int64_t zero_ns, std::string &out)
    {
        out.reserve(out.size() + states_.size() * 320);
        for (const l2::PointPacketState &st : states_)
            l2::AppendPacketStateRow(out, st, zero_ns);
        states_.clear();
    }
}
