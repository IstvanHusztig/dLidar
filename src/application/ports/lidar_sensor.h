#pragma once

#include "application/ports/stop_token.h"
#include "domain/geometry/point_filter.h"
#include "domain/recording/packet_timing.h"
#include "domain/recording/point_record.h"
#include "domain/recording/recording_info.h"

#include <optional>
#include <stdint.h>
#include <string>
#include <vector>

namespace dlidar
{
    // One IMU sample in the sensor frame (the same frame as the sensor's
    // points; a sensor whose IMU is mounted differently rotates it first).
    struct ImuReading
    {
        float accel_mps2[3]; // m/s^2
        float gyro_rads[3];  // rad/s
        int64_t stamp_ns;    // sensor clock, the same clock as the point packets
        uint32_t seq;        // sequence number
    };

    // The point packet returned by the last Poll().
    struct PointPacketInfo
    {
        int64_t first_ns = 0;    // sensor stamp of the first sample
        int64_t duration_ns = 0; // from the first to the last sample
        uint32_t seq = 0;
        uint32_t count = 0; // samples in the packet (0: an empty packet)
    };

    enum class PacketKind
    {
        None,       // nothing usable this time (timeout, other packet types)
        Imu,        // TakeImu() has the samples
        Points,     // Points() / ConvertPoints() describe it
        EndOfStream // a finite source (replay) has no more packets
    };

    struct Polled
    {
        PacketKind kind = PacketKind::None;
        // The packet's record for packetsNNNN.bin, with the host arrival times
        // the sensor took when it received the packet.
        PacketTiming record{};
    };

    struct SensorInfo
    {
        // Identity for timingNNNN.json: model, versions, and the IMU position
        // in the sensor frame (the core maps it through the mount).
        RecordingInfo versions;
        PointFilter default_filter;  // the sensor's blind zone, range and intensity floor
    };

    // Optional: per-packet diagnostics written to packetstateNNNN.tsv.
    class PacketDiagnostics
    {
    public:
        virtual ~PacketDiagnostics() = default;
        virtual const char *Header() const = 0; // the file's first line, with '\n'
        // Appends the rows of the packets since the last call, stamps relative to zero_ns.
        virtual void TakeRows(int64_t zero_ns, std::string &out) = 0;
    };

    // A LiDAR with an IMU, as the capture core uses it (docs/architecture.md).
    // Pull model: the core calls Poll() in its loop; one call per packet.
    class LidarSensor
    {
    public:
        virtual ~LidarSensor() = default;

        // Brings the sensor into steady streaming and drops packets buffered
        // before the recording. False if it cannot start.
        virtual bool Prepare(const StopToken &stop) = 0;
        virtual SensorInfo Info() = 0;

        virtual Polled Poll() = 0;
        // After Poll() returned Imu: the packet's samples (one or more).
        virtual void TakeImu(std::vector<ImuReading> &out) = 0;
        // After Poll() returned Points.
        virtual const PointPacketInfo &Points() const = 0;
        // After Poll() returned Points with count > 0: appends the packet's
        // points in the sensor frame, times in s relative to zero_ns. May
        // skip points `filter` rejects (the core applies it again).
        virtual void ConvertPoints(int64_t zero_ns, const PointFilter &filter, std::vector<PointRecord> &out) = 0;

        // Lens dirtiness in percent, if the sensor reports it (cached, non-blocking).
        virtual std::optional<float> LensDirtiness() { return std::nullopt; }
        virtual PacketDiagnostics *Diagnostics() { return nullptr; }

        virtual void Stop() = 0;
    };
}
