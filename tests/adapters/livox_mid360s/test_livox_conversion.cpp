// The Mid-360S packet conversion (livox_conversion.h) on hand-built packets:
// units, per-point times, the derived fields, the filters, IMU units and the
// packet checks.
#include "adapters/livox_mid360s/livox_conversion.h"
#include "adapters/livox_mid360s/livox_packet_builder.h"
#include "../../check.h"

#include <cmath>
#include <vector>

using namespace dlidar;
using namespace dlidar::livox;

namespace
{
const int64_t kT = 1234567890123456789LL; // packet stamp, ns since power-on

LivoxLidarCartesianHighRawPoint Pt(int32_t x, int32_t y, int32_t z, uint8_t r = 40, uint8_t tag = 0)
{
    return LivoxLidarCartesianHighRawPoint{x, y, z, r, tag};
}

void TestPointUnits()
{
    std::vector<LivoxLidarCartesianHighRawPoint> pts(96, Pt(5000, 0, 0));
    pts[0] = Pt(1000, -2000, 500, 40);
    pts[1] = Pt(100000, 50000, 0); // 111.8 m: beyond the cut-off
    pts[2] = Pt(50, 0, 0);         // 0.05 m: blind zone
    pts[3] = Pt(70000, 0, 0, 7);   // 70 m: raw_range saturates at 65535 mm
    pts[4] = Pt(0, 0, 3000);       // straight up
    std::vector<uint8_t> pkt = MakePointPacket(kT, 7, 300, 95 * 50, pts); // 5 us per point
    CHECK(CheckPacket(pkt.data(), pkt.size()) == PacketStatus::Ok, "packet ok");
    LivoxLidarEthernetPacket h = Header(pkt.data());
    CHECK(StampNs(h) == kT && h.dot_num == 96 && h.length == pkt.size(), "header: %lld", (long long)StampNs(h));
    CHECK(DurationNs(h) == 475000 && std::fabs(PointStepS(h) - 5e-6) < 1e-15, "duration %lld step %.12g",
          (long long)DurationNs(h), PointStepS(h));
    CHECK(PacketSeq(h) == (7u << 16 | 300u), "packet seq %u", PacketSeq(h));

    const int64_t zero = kT - 2000000000LL; // the packet is 2 s after the zero
    std::vector<PointRecord> out;
    ParsePoints(pkt.data(), zero, kDefaultFilter, false, out);
    CHECK(out.size() == 94, "%zu points (two filtered)", out.size());
    const PointRecord &a = out[0];
    CHECK(a.x == 1.0f && a.y == -2.0f && a.z == 0.5f && a.intensity == 40.0f, "(%g %g %g) i %g", a.x, a.y, a.z, a.intensity);
    CHECK(a.range == (float)std::sqrt(5.25) && a.raw_range == 2291, "range %.9g raw %u", a.range, a.raw_range);
    CHECK(a.time == 2.0 && a.sample_index == 0 && a.ring == 1 && a.packet_seq == (7u << 16 | 300u), "point 0 fields");
    CHECK(std::fabs(a.alpha - (float)std::atan2(-2.0, 1.0)) < 1e-7f &&
              std::fabs(a.theta - (float)std::atan2(0.5, std::hypot(1.0, 2.0))) < 1e-7f,
          "alpha %g theta %g", a.alpha, a.theta);
    const PointRecord &sat = out[1]; // pts[3]
    CHECK(sat.sample_index == 3 && sat.raw_range == 65535 && sat.range == 70.0f, "saturated %u %g", sat.raw_range, sat.range);
    CHECK(std::fabs(sat.time - (2.0 + 3 * 5e-6)) < 1e-12, "point 3 at T + 3 D/95: %.12f", sat.time);
    const PointRecord &up = out[2];
    CHECK(up.sample_index == 4 && std::fabs(up.theta - (float)(M_PI / 2)) < 1e-6f, "elevation of a point above %g", up.theta);
    const PointRecord &last = out.back();
    CHECK(last.sample_index == 95 && std::fabs(last.time - (2.0 + 475e-6)) < 1e-12, "last point at T + D: %.12f", last.time);
}

void TestLowConfidence()
{
    // tag: bits 0-1 glue, 2-3 rain/fog/dust; 2 = low confidence, 1 = medium.
    std::vector<LivoxLidarCartesianHighRawPoint> pts = {Pt(5000, 0, 0, 40, 0x00), Pt(5000, 0, 0, 40, 0x02),
                                                        Pt(5000, 0, 0, 40, 0x08), Pt(5000, 0, 0, 40, 0x05),
                                                        Pt(5000, 0, 0, 40, 0x30)};
    CHECK(!LowConfidence(0x00) && LowConfidence(0x02) && LowConfidence(0x08) && !LowConfidence(0x05) &&
              !LowConfidence(0x30),
          "LowConfidence bits");
    std::vector<uint8_t> pkt = MakePointPacket(kT, 0, 0, 400, pts);
    std::vector<PointRecord> keep, drop;
    ParsePoints(pkt.data(), kT, kDefaultFilter, false, keep);
    ParsePoints(pkt.data(), kT, kDefaultFilter, true, drop);
    CHECK(keep.size() == 5, "kept by default: %zu", keep.size());
    CHECK(drop.size() == 3 && drop[0].sample_index == 0 && drop[1].sample_index == 3 && drop[2].sample_index == 4,
          "dropped on request: %zu", drop.size());

    PointFilter dark{0.1f, 100.0f, 50.0f};
    std::vector<PointRecord> bright;
    ParsePoints(pkt.data(), kT, dark, false, bright);
    CHECK(bright.empty(), "an intensity floor applies to reflectivity: %zu", bright.size());
}

void TestImu()
{
    LivoxLidarImuRawPoint s{0.1f, -0.2f, 0.3f, 0.01f, -0.02f, 1.0f};
    std::vector<uint8_t> pkt = MakeImuPacket(kT + 5, 42, s);
    CHECK(CheckPacket(pkt.data(), pkt.size()) == PacketStatus::Ok, "IMU packet ok");
    ImuReading r{};
    ParseImu(pkt.data(), r);
    CHECK(r.stamp_ns == kT + 5, "stamp");
    CHECK(r.gyro_rads[0] == 0.1f && r.gyro_rads[1] == -0.2f && r.gyro_rads[2] == 0.3f, "gyro in rad/s as sent");
    CHECK(r.accel_mps2[2] == 1.0f * 9.80665f && r.accel_mps2[0] == 0.01f * 9.80665f, "accel g -> m/s^2: %g",
          r.accel_mps2[2]);
}

void TestChecks()
{
    std::vector<uint8_t> pkt = MakePointPacket(kT, 0, 0, 400, std::vector<LivoxLidarCartesianHighRawPoint>(96, Pt(1, 1, 1)));
    CHECK(CheckPacket(pkt.data(), 20) == PacketStatus::TooShort, "shorter than the header");
    CHECK(CheckPacket(pkt.data(), pkt.size() - 1) == PacketStatus::TooShort, "one byte short of 96 points");
    std::vector<uint8_t> low = pkt;
    low[offsetof(LivoxLidarEthernetPacket, data_type)] = kLivoxLidarCartesianCoordinateLowData;
    CHECK(CheckPacket(low.data(), low.size()) == PacketStatus::Ok, "low-resolution data type is a known packet");
    std::vector<uint8_t> bad = pkt;
    bad[offsetof(LivoxLidarEthernetPacket, data_type)] = 9;
    CHECK(CheckPacket(bad.data(), bad.size()) == PacketStatus::UnknownType, "unknown data type");
    std::vector<uint8_t> imu = MakeImuPacket(kT, 0, LivoxLidarImuRawPoint{});
    CHECK(CheckPacket(imu.data(), imu.size() - 1) == PacketStatus::TooShort, "IMU packet one byte short");
    std::vector<uint8_t> one = MakePointPacket(kT, 0, 0, 0, {Pt(1000, 0, 0)});
    CHECK(PointStepS(Header(one.data())) == 0.0 && DurationNs(Header(one.data())) == 0, "a single point has no step");
}
} // namespace

int main()
{
    TestPointUnits();
    TestLowConfidence();
    TestImu();
    TestChecks();
    return FinishTests("Livox conversion");
}
