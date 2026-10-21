#pragma once

// A scripted LidarSensor for core tests: no L2 code, no SDK. Packets are
// queued with their host arrival, Poll() hands them out in order and returns
// EndOfStream at the end. Points are generated (range `range_m` along +X,
// times j * dt_s after the packet stamp) and not pre-filtered, so the core's
// own filter is what removes them. Diagnostics (optional) are one row
// "<stamp - zero>\n" per point packet.
#include "application/ports/lidar_sensor.h"

#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace dlidar::test
{
    class FakeSensor : public LidarSensor, private PacketDiagnostics
    {
    public:
        struct Packet
        {
            PacketKind kind = PacketKind::None;
            PacketTiming record{};
            std::vector<ImuReading> imu;
            PointPacketInfo info;
            double dt_s = 1e-5;
            float range_m = 2.0f;
            float intensity = 100.0f;
        };

        bool with_diagnostics = false;
        bool prepared = false, stopped = false;
        bool prepare_ok = true;
        std::function<void()> on_prepare; // runs inside Prepare (e.g. to hold the start-up)
        // When the queue is empty: endless point packets of 300 points, 3 ms
        // apart, instead of EndOfStream.
        bool endless = false;
        std::optional<float> dirtiness;
        RecordingInfo versions;
        PointFilter default_filter{0.1f, 100.0f, 5.0f};

        void AddImu(int64_t host_ns, int64_t stamp_ns, uint32_t seq, float accel_z_mps2 = 9.80665f, float gyro_x = 0)
        {
            Packet p;
            p.kind = PacketKind::Imu;
            p.record = {host_ns, host_ns, stamp_ns, seq, kPacketTypeImu, 0.0f, 0};
            ImuReading r{{0, 0, accel_z_mps2}, {gyro_x, 0, 0}, stamp_ns, seq};
            p.imu.push_back(r);
            queue_.push_back(p);
        }

        void AddPoints(int64_t host_ns, int64_t stamp_ns, uint32_t seq, uint32_t count, float intensity = 100.0f,
                       double dt_s = 1e-5)
        {
            Packet p;
            p.kind = PacketKind::Points;
            p.info = {stamp_ns, count > 1 ? (int64_t)((count - 1) * dt_s * 1e9) : 0, seq, count};
            p.record = {host_ns, host_ns, stamp_ns, seq, kPacketTypePoints, (float)dt_s, count};
            p.dt_s = dt_s;
            p.intensity = intensity;
            queue_.push_back(p);
        }

        void AddNothing() { queue_.push_back(Packet{}); }

        size_t Pending() const { return queue_.size(); }

        bool Prepare(const StopToken &) override
        {
            if (on_prepare)
                on_prepare();
            prepared = true;
            return prepare_ok;
        }
        SensorInfo Info() override { return {versions, default_filter}; }

        Polled Poll() override
        {
            Polled out;
            if (queue_.empty() && endless)
            {
                endless_n_++;
                AddPoints(endless_n_ * 3000000, 1000000000000LL + endless_n_ * 3000000, (uint32_t)endless_n_, 300);
            }
            if (queue_.empty())
            {
                out.kind = PacketKind::EndOfStream;
                return out;
            }
            current_ = queue_.front();
            queue_.pop_front();
            out.kind = current_.kind;
            out.record = current_.record;
            if (current_.kind == PacketKind::Points)
                stamps_.push_back(current_.info.first_ns);
            return out;
        }

        void TakeImu(std::vector<ImuReading> &out) override
        {
            out.insert(out.end(), current_.imu.begin(), current_.imu.end());
            current_.imu.clear();
        }
        const PointPacketInfo &Points() const override { return current_.info; }

        void ConvertPoints(int64_t zero_ns, const PointFilter &, std::vector<PointRecord> &out) override
        {
            for (uint32_t j = 0; j < current_.info.count; j++)
            {
                PointRecord p{};
                p.x = current_.range_m;
                p.intensity = current_.intensity;
                p.range = current_.range_m;
                p.time = (double)(current_.info.first_ns - zero_ns) / 1e9 + (double)j * current_.dt_s;
                p.packet_seq = current_.info.seq;
                p.sample_index = (uint16_t)j;
                p.ring = 1;
                out.push_back(p);
            }
        }

        std::optional<float> LensDirtiness() override { return dirtiness; }
        PacketDiagnostics *Diagnostics() override { return with_diagnostics ? this : nullptr; }
        void Stop() override { stopped = true; }

    private:
        std::deque<Packet> queue_;
        Packet current_;
        int64_t endless_n_ = 0;
        std::vector<int64_t> stamps_;

        const char *Header() const override { return "stamp_ns\n"; }
        void TakeRows(int64_t zero_ns, std::string &out) override
        {
            for (int64_t s : stamps_)
                out += std::to_string(s - zero_ns) + "\n";
            stamps_.clear();
        }
    };
}
