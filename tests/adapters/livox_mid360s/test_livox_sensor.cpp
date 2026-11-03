// LivoxMid360sSensor on scripted packets: the Polled records and Points(),
// the IMU seq widened across the 16-bit wrap, the diagnostics rows, the
// identity, and how other data types and broken packets are handled.
#include "adapters/livox_mid360s/livox_imu_range.h"
#include "adapters/livox_mid360s/livox_mid360s_sensor.h"
#include "adapters/livox_mid360s/livox_packet_builder.h"
#include "scripted_livox_source.h"
#include "../../check.h"

#include <memory>
#include <string>
#include <vector>

using namespace dlidar;
using namespace dlidar::livox;

namespace
{
const int64_t kT = 7000000000LL; // sensor clock, ns since power-on

std::vector<LivoxLidarCartesianHighRawPoint> Points96(uint8_t tag = 0)
{
    return std::vector<LivoxLidarCartesianHighRawPoint>(96, LivoxLidarCartesianHighRawPoint{3000, 0, 1000, 20, tag});
}

void TestPollAndDiagnostics()
{
    auto src = std::make_unique<test::ScriptedLivoxSource>();
    test::ScriptedLivoxSource &s = *src;
    // IMU udp_cnt 65534, 65535, 0, 1, then 3 (one lost).
    const uint16_t cnts[] = {65534, 65535, 0, 1, 3};
    for (int i = 0; i < 5; i++)
        s.Add(MakeImuPacket(kT + i * 5000000LL, cnts[i], LivoxLidarImuRawPoint{0, 0, 0.5f, 0, 0, 1.0f}),
              {100 + i, 1000 + i});
    std::vector<LivoxLidarCartesianHighRawPoint> pts = Points96();
    pts[0].tag = 0x02; // one low-confidence point
    s.Add(MakePointPacket(kT + 1000, 3, 17, 4800, pts, 1), {200, 2000});
    std::vector<uint8_t> low = MakePointPacket(kT + 2000, 3, 18, 4800, Points96());
    low[offsetof(LivoxLidarEthernetPacket, data_type)] = kLivoxLidarCartesianCoordinateLowData;
    s.Add(low, {201, 2001});
    std::vector<uint8_t> cut = MakePointPacket(kT + 3000, 3, 19, 4800, Points96());
    cut.resize(cut.size() - 5);
    s.Add(cut, {202, 2002});

    LivoxMid360sSensor sensor(std::move(src));
    std::vector<uint32_t> seqs;
    for (int i = 0; i < 5; i++)
    {
        Polled p = sensor.Poll();
        CHECK(p.kind == PacketKind::Imu, "IMU packet %d", i);
        CHECK(p.record.type == kPacketTypeImu && p.record.host_mono_ns == 100 + i && p.record.host_unix_ns == 1000 + i &&
                  p.record.sensor_ns == kT + i * 5000000LL,
              "IMU record %d", i);
        std::vector<ImuReading> r;
        sensor.TakeImu(r);
        CHECK(r.size() == 1 && r[0].accel_mps2[2] == 9.80665f && r[0].gyro_rads[2] == 0.5f, "IMU sample %d", i);
        seqs.push_back(r.empty() ? 0 : r[0].seq);
        CHECK(!r.empty() && p.record.seq == r[0].seq, "record seq = sample seq");
    }
    CHECK(seqs == std::vector<uint32_t>({65534, 65535, 65536, 65537, 65539}), "seq widened across the wrap: %u %u %u %u %u",
          seqs[0], seqs[1], seqs[2], seqs[3], seqs[4]);

    Polled p = sensor.Poll();
    CHECK(p.kind == PacketKind::Points && sensor.Points().count == 96 && sensor.Points().first_ns == kT + 1000 &&
              sensor.Points().duration_ns == 480000 && sensor.Points().seq == (3u << 16 | 17u),
          "point packet info");
    CHECK(p.record.type == kPacketTypePoints && p.record.point_num == 96 && p.record.seq == (3u << 16 | 17u) &&
              p.record.time_increment == (float)(480e-6 / 95),
          "point record");
    std::vector<PointRecord> out;
    sensor.ConvertPoints(kT, kDefaultFilter, out);
    CHECK(out.size() == 96 && out[0].x == 3.0f && out[0].z == 1.0f, "points converted: %zu", out.size());

    p = sensor.Poll();
    CHECK(p.kind == PacketKind::Points && sensor.Points().count == 0 && sensor.OtherPointPackets() == 1,
          "a low-resolution packet is logged without points");
    p = sensor.Poll();
    CHECK(p.kind == PacketKind::None && sensor.SkippedPackets() == 1, "a cut packet is skipped");
    CHECK(sensor.Poll().kind == PacketKind::EndOfStream, "end of stream");

    PacketDiagnostics *d = sensor.Diagnostics();
    CHECK(d && std::string(d->Header()) == kPacketStateHeader, "diagnostics header");
    std::string rows;
    d->TakeRows(kT, rows);
    CHECK(rows == "1000\t7000001000\t1\t1\t3\t17\t96\t4800\t1\n2000\t7000002000\t0\t2\t3\t18\t96\t4800\t0\n",
          "rows:\n%s", rows.c_str());
    rows.clear();
    d->TakeRows(kT, rows);
    CHECK(rows.empty(), "rows are taken once");
}

void TestInfoAndLowConfidence()
{
    auto src = std::make_unique<test::ScriptedLivoxSource>();
    src->Add(MakePointPacket(kT, 0, 0, 4800, Points96(0x08)), {1, 1});
    LivoxMid360sSensor sensor(std::move(src), true);
    SensorInfo info = sensor.Info();
    CHECK(info.versions.sensor_model == std::string("livox_mid360s") && info.versions.lidar_firmware_version == "scripted",
          "identity");
    CHECK(info.versions.imu_position_m && (*info.versions.imu_position_m)[0] == 0.011f &&
              (*info.versions.imu_position_m)[2] == -0.04412f,
          "IMU position in the sensor frame");
    CHECK(info.default_filter.min_range_m == 0.1f && info.default_filter.max_range_m == 100.0f &&
              info.default_filter.min_intensity == 0.0f,
          "default filter");
    sensor.Poll();
    std::vector<PointRecord> out;
    sensor.ConvertPoints(kT, kDefaultFilter, out);
    CHECK(out.empty(), "drop_low_confidence drops rain/fog/dust points: %zu", out.size());
}
void TestImuRangeCodes()
{
    ImuRangeCodes c{};
    CHECK(ToImuRangeCodes(500, 8, 2000, c) && c.rate == kLivoxLidarImuOutRate500Hz && c.accel == kLivoxLidarAccelRange8G &&
              c.gyro == kLivoxLidarGyroRange2000Dps,
          "the defaults");
    CHECK(ToImuRangeCodes(50, 32, 15.625, c) && c.rate == kLivoxLidarImuOutRate50Hz && c.accel == kLivoxLidarAccelRange32G &&
              c.gyro == kLivoxLidarGyroRange15_625Dps,
          "the other ends");
    CHECK(ToImuRangeCodes(100, 4, 62.5, c) && c.rate == kLivoxLidarImuOutRate100Hz && c.gyro == kLivoxLidarGyroRange62_5Dps,
          "62.5 deg/s");
    CHECK(!ToImuRangeCodes(400, 8, 2000, c) && !ToImuRangeCodes(500, 2, 2000, c) && !ToImuRangeCodes(500, 8, 300, c),
          "unsupported values");
}
} // namespace

int main()
{
    TestPollAndDiagnostics();
    TestInfoAndLowConfidence();
    TestImuRangeCodes();
    return FinishTests("Livox sensor");
}
