#pragma once

#include "adapters/livox_mid360s/livox_packet_source.h"
#include "adapters/livox_mid360s/livox_packet_state.h"
#include "application/ports/lidar_sensor.h"

#include <memory>
#include <vector>

namespace dlidar
{
    // The Livox Mid-360S as a LidarSensor: packets from a LivoxPacketSource,
    // converted by livox_conversion, with packetstateNNNN.tsv rows as
    // diagnostics. Only Cartesian high point packets give points; point
    // packets of another data type are logged with no points, and packets
    // that are cut short or of an unknown type are skipped (both counted).
    class LivoxMid360sSensor : public LidarSensor, private PacketDiagnostics
    {
    public:
        explicit LivoxMid360sSensor(std::unique_ptr<livox::LivoxPacketSource> source, bool drop_low_confidence = false);

        bool Prepare(const StopToken &stop) override { return source_->Prepare(stop); }
        SensorInfo Info() override;
        Polled Poll() override;
        void TakeImu(std::vector<ImuReading> &out) override;
        const PointPacketInfo &Points() const override { return points_; }
        void ConvertPoints(int64_t zero_ns, const PointFilter &filter, std::vector<PointRecord> &out) override;
        PacketDiagnostics *Diagnostics() override { return this; }
        void Stop() override { source_->Stop(); }

        livox::LivoxPacketSource &Source() { return *source_; }
        uint64_t SkippedPackets() const { return skipped_; }         // cut short or unknown type
        uint64_t OtherPointPackets() const { return other_points_; } // point packets not Cartesian high

    private:
        std::unique_ptr<livox::LivoxPacketSource> source_;
        bool drop_low_confidence_;
        ImuReading imu_{};
        bool has_imu_ = false;
        // The IMU packets' 16-bit udp_cnt, widened to 32 bits.
        bool has_imu_cnt_ = false;
        uint16_t last_imu_cnt_ = 0;
        uint32_t imu_seq_ = 0;
        PointPacketInfo points_;
        std::vector<livox::PointPacketState> states_;
        uint64_t skipped_ = 0;
        uint64_t other_points_ = 0;

        const char *Header() const override { return livox::kPacketStateHeader; }
        void TakeRows(int64_t zero_ns, std::string &out) override;
    };
}
