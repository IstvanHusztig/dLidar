#include "adapters/livox_mid360s/livox_mid360s_sensor.h"

#include "adapters/livox_mid360s/livox_conversion.h"

namespace dlidar
{
    LivoxMid360sSensor::LivoxMid360sSensor(std::unique_ptr<livox::LivoxPacketSource> source, bool drop_low_confidence)
        : source_(std::move(source)), drop_low_confidence_(drop_low_confidence)
    {
    }

    SensorInfo LivoxMid360sSensor::Info()
    {
        SensorInfo info;
        info.versions = source_->Versions();
        info.versions.sensor_model = livox::kRecordingProfile.sensor_model;
        info.versions.imu_position_m = livox::kImuPositionM;
        info.default_filter = livox::kDefaultFilter;
        return info;
    }

    Polled LivoxMid360sSensor::Poll()
    {
        Polled p;
        livox::HostTime now{};
        int r = source_->Next(now);
        if (r == livox::LivoxPacketSource::kEndOfStream)
        {
            p.kind = PacketKind::EndOfStream;
            return p;
        }
        if (r != livox::LivoxPacketSource::kPacket)
            return p;
        const uint8_t *data = source_->Data();
        if (livox::CheckPacket(data, source_->Size()) != livox::PacketStatus::Ok)
        {
            skipped_++;
            return p;
        }
        const LivoxLidarEthernetPacket h = livox::Header(data);
        if (h.data_type == kLivoxLidarImuData)
        {
            livox::ParseImu(data, imu_);
            if (!has_imu_cnt_)
                imu_seq_ = h.udp_cnt;
            else
                imu_seq_ += (uint16_t)(h.udp_cnt - last_imu_cnt_);
            has_imu_cnt_ = true;
            last_imu_cnt_ = h.udp_cnt;
            imu_.seq = imu_seq_;
            has_imu_ = true;
            p.kind = PacketKind::Imu;
            p.record = {now.mono_ns, now.unix_ns, imu_.stamp_ns, imu_.seq, kPacketTypeImu, 0.0f, 0};
            return p;
        }

        states_.push_back(livox::MakePointPacketState(data)); // every point packet, empty ones too
        bool cartesian = h.data_type == kLivoxLidarCartesianCoordinateHighData;
        if (!cartesian)
            other_points_++;
        points_.first_ns = livox::StampNs(h);
        points_.duration_ns = livox::DurationNs(h);
        points_.seq = livox::PacketSeq(h);
        points_.count = cartesian ? h.dot_num : 0;
        p.kind = PacketKind::Points;
        p.record = {now.mono_ns, now.unix_ns, points_.first_ns, points_.seq, kPacketTypePoints,
                    (float)livox::PointStepS(h), h.dot_num};
        return p;
    }

    void LivoxMid360sSensor::TakeImu(std::vector<ImuReading> &out)
    {
        if (has_imu_)
            out.push_back(imu_);
        has_imu_ = false;
    }

    void LivoxMid360sSensor::ConvertPoints(int64_t zero_ns, const PointFilter &filter, std::vector<PointRecord> &out)
    {
        livox::ParsePoints(source_->Data(), zero_ns, filter, drop_low_confidence_, out);
    }

    void LivoxMid360sSensor::TakeRows(int64_t zero_ns, std::string &out)
    {
        out.reserve(out.size() + states_.size() * 64);
        for (const livox::PointPacketState &st : states_)
            livox::AppendPacketStateRow(out, st, zero_ns);
        states_.clear();
    }
}
