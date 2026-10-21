// CaptureTiming and ChunkAssembler driven by FakeSensor only (no L2 code):
// the time zero rule, the order of the work done for one point packet, chunk
// boundaries, and the mount/units/filter the core applies. These pin the
// behaviour the byte-identical recordings depend on.
#include "application/capture/chunk_assembler.h"
#include "fake_sensor.h"
#include "../check.h"

#include <algorithm>
#include <cinttypes>

using namespace dlidar;
using dlidar::test::FakeSensor;

namespace
{
const int64_t kS0 = 1790000000000000000LL;
const PointFilter kFilter{0.1f, 100.0f, 5.0f};

ImuTimeline::Config SensorMode()
{
    ImuTimeline::Config cfg;
    cfg.mode = ImuTimeline::Mode::Sensor;
    cfg.log = false;
    cfg.host.log = false;
    return cfg;
}

void Feed(FakeSensor &s, CaptureTiming &t, std::vector<PointRecord> &pts, std::vector<ImuSample> &imu)
{
    while (s.Pending())
        t.OnPacket(s.Poll(), pts, imu);
}

// The zero is the stamp of the first point packet with points - even if the
// filter removes all of them - set before that packet's points are converted.
// Empty packets before it leave it at 0.
void TestZeroRule()
{
    printf("time zero rule\n");
    FakeSensor s;
    s.with_diagnostics = true;
    CaptureTiming t(s, SensorMode(), AxisMap(), kFilter);
    std::vector<PointRecord> pts;
    std::vector<ImuSample> imu;

    s.AddPoints(1, kS0, 1, 0);
    Feed(s, t, pts, imu);
    CHECK(t.ZeroNs() == 0, "empty packet set the zero to %" PRId64, t.ZeroNs());
    std::string rows;
    t.TakePacketStateRows(t.ZeroNs(), rows);
    CHECK(rows == std::to_string(kS0) + "\n", "before the zero, diagnostics stamps are absolute: %s", rows.c_str());

    s.AddPoints(2, kS0 + 1000000, 2, 4, 0.0f); // every point below the intensity floor
    Feed(s, t, pts, imu);
    CHECK(t.ZeroNs() == kS0 + 1000000, "all-filtered packet must still set the zero (%" PRId64 ")", t.ZeroNs());
    CHECK(pts.empty(), "%zu points", pts.size());

    s.AddPoints(3, kS0 + 3000000, 3, 3);
    Feed(s, t, pts, imu);
    CHECK(t.ZeroNs() == kS0 + 1000000, "zero moved");
    CHECK(pts.size() == 3, "%zu points", pts.size());
    for (size_t j = 0; j < pts.size(); j++)
    {
        double want = (double)2000000 / 1e9 + (double)j * 1e-5;
        CHECK(pts[j].time == want, "point %zu time %.15f, want %.15f", j, pts[j].time, want);
    }
}

// One point packet: packet log record (also for an empty packet), its points,
// then IMU samples that ImuTimeline releases on it (here: samples held until
// the zero is known).
void TestPacketOrder()
{
    printf("work order of a point packet\n");
    FakeSensor s;
    s.with_diagnostics = true;
    CaptureTiming t(s, SensorMode(), AxisMap(), kFilter);
    std::vector<PointRecord> pts;
    std::vector<ImuSample> imu;

    s.AddImu(1, kS0 + 500000, 10);
    s.AddImu(2, kS0 + 1500000, 11);
    Feed(s, t, pts, imu);
    CHECK(imu.empty(), "IMU written before the zero is known (%zu)", imu.size());

    s.AddPoints(3, kS0, 1, 0);
    s.AddPoints(4, kS0 + 1000000, 2, 2);
    Feed(s, t, pts, imu);
    CHECK(pts.size() == 2, "%zu points", pts.size());
    CHECK(imu.size() == 1 && imu[0].LidarTimestamp == 500000, "the first point packet releases the IMU after the zero: %zu",
          imu.size());

    std::vector<PacketTiming> log;
    t.TakePacketLog(log);
    CHECK(log.size() == 4 && log[0].type == kPacketTypeImu && log[2].type == kPacketTypePoints && log[2].point_num == 0 &&
              log[3].seq == 2 && log[3].host_mono_ns == 4,
          "packet log has every packet in arrival order (%zu)", log.size());
    std::string rows;
    t.TakePacketStateRows(t.ZeroNs(), rows);
    CHECK(std::count(rows.begin(), rows.end(), '\n') == 2, "one diagnostics row per point packet, empty ones too");
}

// Mount, m/s^2 -> g and the filter are the core's: a fake sensor that knows
// nothing about them gets the same treatment as the L2.
void TestMountUnitsFilter()
{
    printf("mount, units and filter applied by the core\n");
    FakeSensor s;
    AxisMap vertical{Axis::PosZ, Axis::PosY, Axis::NegX};
    CaptureTiming t(s, SensorMode(), vertical, kFilter);
    std::vector<PointRecord> pts;
    std::vector<ImuSample> imu;

    s.AddPoints(1, kS0, 1, 2);
    s.AddImu(2, kS0 + 100, 5, -9.80665f, 0.25f); // sensor accel -1 g on Z, gyro 0.25 rad/s about X
    Feed(s, t, pts, imu);
    // Fake points lie on sensor +X; with +X down (vertical L2) they point to output -Z.
    CHECK(pts.size() == 2 && pts[0].x == 0 && pts[0].z == -2.0f, "point (%g %g %g)", pts[0].x, pts[0].y, pts[0].z);
    // The fake IMU's sensor Z is the output front axis (X); its X gyro the output -Z.
    CHECK(imu.size() == 1 && imu[0].AccelerationX == -1.0f && imu[0].GyroZ == -0.25f, "imu accel x %g gyro z %g",
          imu.empty() ? 0 : imu[0].AccelerationX, imu.empty() ? 0 : imu[0].GyroZ);
}

// A chunk is cut right after the packet that reaches kMaxPointsPerChunk, and
// holds exactly the packets up to it.
void TestChunkBoundary()
{
    printf("chunk boundary\n");
    FakeSensor s;
    RecordingInfo info;
    ChunkAssembler as(s, SensorMode(), info, AxisMap(), kFilter);
    const int n = 300, per_chunk = ChunkAssembler::kMaxPointsPerChunk;
    int packets = 0;
    while (!as.ChunkReady())
    {
        s.AddPoints(packets + 1, kS0 + packets * 3000000LL, (uint32_t)packets, n);
        as.OnPacket(s.Poll());
        packets++;
    }
    CHECK(packets == (per_chunk + n - 1) / n, "ready after %d packets", packets);
    SensorChunk c = as.TakeChunk();
    CHECK(c.points.size() == (size_t)packets * n && c.packets.size() == (size_t)packets && c.chunk_index == 1,
          "chunk: %zu points, %zu packets", c.points.size(), c.packets.size());
    CHECK(c.packet_state_rows.empty(), "no diagnostics from a sensor without them");
    CHECK(!as.ChunkReady(), "next chunk starts empty");
}
} // namespace

int main()
{
    TestZeroRule();
    TestPacketOrder();
    TestMountUnitsFilter();
    TestChunkBoundary();
    return FinishTests("Capture timing");
}
