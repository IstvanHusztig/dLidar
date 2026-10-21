// CaptureTiming and ChunkAssembler: the time zero rule, the order of the work
// done for one point packet, and chunk boundaries. These pin the behaviour
// the byte-identical recordings depend on.
#include "pcd_manager.h"
#include "check.h"

#include <cinttypes>
#include <cmath>

namespace
{
const int64_t kS0 = 1790000000000000000LL;

LidarPointDataPacket Points(uint32_t seq, int64_t stamp, int n, uint8_t intensity = 100)
{
    LidarPointDataPacket pkt{};
    LidarPointData &d = pkt.data;
    d.info.seq = seq;
    d.info.stamp = TimeStamp{(uint32_t)(stamp / 1000000000LL), (uint32_t)(stamp % 1000000000LL)};
    d.param.range_scale = 0.001f;
    d.range_min = 0;
    d.range_max = 100;
    d.point_num = (uint32_t)n;
    d.time_increment = 1e-5f;
    for (int j = 0; j < n; j++)
    {
        d.ranges[j] = 2000;
        d.intensities[j] = intensity;
    }
    return pkt;
}

LidarImuData Imu(uint32_t seq, int64_t stamp)
{
    LidarImuData d{};
    d.info.seq = seq;
    d.info.stamp = TimeStamp{(uint32_t)(stamp / 1000000000LL), (uint32_t)(stamp % 1000000000LL)};
    d.linear_acceleration[2] = 9.80665f;
    d.angular_velocity[0] = (float)seq;
    return d;
}

ImuTimeline::Config SensorMode()
{
    ImuTimeline::Config cfg;
    cfg.mode = ImuTimeline::Mode::Sensor;
    cfg.log = false;
    cfg.host.log = false;
    return cfg;
}

// The zero is the L2 stamp of the first point packet with points - even if the
// filter removes all of them - set before that packet's points are converted.
// Empty packets before it leave it at 0.
void TestZeroRule()
{
    printf("time zero rule\n");
    CaptureTiming t(SensorMode());
    std::vector<PointDLidar> pts;
    std::vector<OutputImuData> imu;

    t.OnPoints(Points(1, kS0, 0), {1, 1}, pts, imu);
    CHECK(t.ZeroNs() == 0, "empty packet set the zero to %" PRId64, t.ZeroNs());
    std::string rows;
    t.TakePacketStateRows(t.ZeroNs(), rows);
    CHECK(rows.compare(0, 20, std::to_string(kS0) + "\t") == 0, "before the zero, stamp_ns is absolute: %.40s", rows.c_str());

    t.OnPoints(Points(2, kS0 + 1000000, 4, 0), {2, 2}, pts, imu); // every point below the intensity floor
    CHECK(t.ZeroNs() == kS0 + 1000000, "all-filtered packet must still set the zero (%" PRId64 ")", t.ZeroNs());
    CHECK(pts.empty(), "%zu points", pts.size());

    t.OnPoints(Points(3, kS0 + 3000000, 3), {3, 3}, pts, imu);
    CHECK(t.ZeroNs() == kS0 + 1000000, "zero moved");
    CHECK(pts.size() == 3, "%zu points", pts.size());
    for (size_t j = 0; j < pts.size(); j++)
    {
        double want = (double)2000000 / 1.0e9 + (double)j * (double)1e-5f;
        CHECK(pts[j].time == want, "point %zu time %.15f, want %.15f", j, pts[j].time, want);
    }
}

// One point packet: packet log record, then its packetstate row (also for an
// empty packet), then its points, then IMU samples that ImuTimeline releases
// on it (here: samples held until the zero is known).
void TestPacketOrder()
{
    printf("work order of a point packet\n");
    CaptureTiming t(SensorMode());
    std::vector<PointDLidar> pts;
    std::vector<OutputImuData> imu;

    t.OnImu(Imu(10, kS0 + 500000), {1, 1}, imu);
    t.OnImu(Imu(11, kS0 + 1500000), {2, 2}, imu);
    CHECK(imu.empty(), "IMU written before the zero is known (%zu)", imu.size());

    t.OnPoints(Points(1, kS0, 0), {3, 3}, pts, imu);
    t.OnPoints(Points(2, kS0 + 1000000, 2), {4, 4}, pts, imu);
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
    CHECK(std::count(rows.begin(), rows.end(), '\n') == 2, "one state row per point packet, empty ones too");
}

// A chunk is cut right after the packet that reaches kMaxPointsPerChunk, and
// holds exactly the packets up to it.
void TestChunkBoundary()
{
    printf("chunk boundary\n");
    RecordingInfo info;
    ChunkAssembler as(SensorMode(), info);
    const int n = 300, per_chunk = ChunkAssembler::kMaxPointsPerChunk;
    int packets = 0;
    while (!as.ChunkReady())
    {
        as.OnPoints(Points((uint32_t)packets, kS0 + packets * 3000000LL, n), {packets + 1, packets + 1});
        packets++;
    }
    CHECK(packets == (per_chunk + n - 1) / n, "ready after %d packets", packets);
    SensorChunk c = as.TakeChunk();
    CHECK(c.points.size() == (size_t)packets * n && c.packets.size() == (size_t)packets && c.chunk_index == 1,
          "chunk: %zu points, %zu packets", c.points.size(), c.packets.size());
    CHECK(!as.ChunkReady(), "next chunk starts empty");
}
} // namespace

int main()
{
    TestZeroRule();
    TestPacketOrder();
    TestChunkBoundary();
    return FinishTests("Capture timing");
}
