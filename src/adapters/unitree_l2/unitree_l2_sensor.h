#pragma once

#include "adapters/unitree_l2/l2_packet_source.h"
#include "adapters/unitree_l2/l2_packet_state.h"
#include "application/ports/lidar_sensor.h"

#include <memory>
#include <vector>

namespace dlidar
{
    // The Unitree L2 as a LidarSensor: packets from an L2PacketSource,
    // converted by l2_conversion, with packetstateNNNN.tsv rows as diagnostics.
    class UnitreeL2Sensor : public LidarSensor, private PacketDiagnostics
    {
    public:
        explicit UnitreeL2Sensor(std::unique_ptr<L2PacketSource> source);

        bool Prepare(const StopToken &stop) override { return source_->Prepare(stop); }
        SensorInfo Info() override;
        Polled Poll() override;
        void TakeImu(std::vector<ImuReading> &out) override;
        const PointPacketInfo &Points() const override { return points_; }
        void ConvertPoints(int64_t zero_ns, const PointFilter &filter, std::vector<PointRecord> &out) override;
        std::optional<float> LensDirtiness() override { return source_->LensDirtiness(); }
        PacketDiagnostics *Diagnostics() override { return this; }
        void Stop() override { source_->Stop(); }

        L2PacketSource &Source() { return *source_; }

    private:
        std::unique_ptr<L2PacketSource> source_;
        ImuReading imu_{};
        bool has_imu_ = false;
        PointPacketInfo points_;
        std::vector<l2::PointPacketState> states_;

        const char *Header() const override { return l2::kPacketStateHeader; }
        void TakeRows(int64_t zero_ns, std::string &out) override;
    };
}
