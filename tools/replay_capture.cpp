// Replays the packet sequence of a recording's packetsNNNN.bin (L2 stamps,
// seq, host arrival times, time_increment, point_num) through the capture
// code into a new recording in out_dir (which must exist). No sensor needed.
//
//   bin/replay_capture ~/PointCloudDump/packets0001.bin out_dir [seconds]
//
// The packet log holds only timing, so packet contents are synthetic and
// deterministic: rays of the L2 sensor model (same formula as the parser) hit
// a box room, the IMU is at rest. Host times come from the log, so two builds
// fed the same log write the same recording. tools/compare_recording_formats.sh
// builds this file against an older commit's include/ as well (that build
// mirrors the old ProcessSensorData loop) to check that a format change keeps
// the standard content identical.
#if __has_include("packet_state.h")
#define NEW_FORMAT 1
#include "pcd_manager.h"
#if __has_include("app/device_config.h")
#define MOUNT_IN_CORE 1
#include "app/device_config.h"
#endif
#else
#define NEW_FORMAT 0
#include "datawriter.h"
#include "capture_timing.h"
#endif
#include "packet_replay.h"

#include <cmath>
#include <cstdlib>

static uint32_t Hash(uint64_t a)
{
    a ^= a >> 33; a *= 0xff51afd7ed558ccdULL; a ^= a >> 33; a *= 0xc4ceb9fe1a85ec53ULL; a ^= a >> 33;
    return (uint32_t)a;
}

static LidarPointDataPacket MakePoints(const PacketTiming &t, int64_t t0)
{
    LidarPointDataPacket pkt{};
    LidarPointData &d = pkt.data;
    d.info.seq = t.seq;
    d.info.stamp = TimeStamp{(uint32_t)(t.sensor_ns / 1000000000LL), (uint32_t)(t.sensor_ns % 1000000000LL)};
    d.point_num = t.point_num;
    d.time_increment = t.time_increment;
    double ts = (t.sensor_ns - t0) / 1e9;
    d.scan_period = 0.18f;
    d.range_min = 0.05f;
    d.range_max = 50.0f;
    d.param.range_scale = 0.001f;
    d.param.range_bias = 2.5f;
    d.param.a_axis_dist = 0.0076f;
    d.param.b_axis_dist = -0.0021f;
    d.param.theta_angle_bias = 0.0015f;
    d.param.alpha_angle_bias = -0.0021f;
    d.param.beta_angle = 0.0035f;
    d.param.xi_angle = -0.0012f;
    d.state.apd_temperature = (float)(40.0 + 2.0 * std::sin(ts / 60.0));
    d.state.apd_voltage = (float)(120.0 + 0.5 * std::sin(ts / 30.0));
    d.state.laser_voltage = 12.0f;
    d.state.imu_temperature = (float)(35.0 + ts / 100.0);
    d.state.dirty_index = 0.5f;
    d.state.sys_rotation_period = 180180;
    d.state.com_rotation_period = 4630;
    const double w_theta = 2 * M_PI * 5.55;
    d.com_horizontal_angle_start = (float)std::fmod(w_theta * ts, 2 * M_PI);
    d.com_horizontal_angle_step = (float)(w_theta * t.time_increment);
    d.angle_min = 0.0f;
    d.angle_increment = (float)(M_PI / 300.0);

    // Same geometry as parseFromPacketToPointCloud, in double: orig(r) = P0 + r * D.
    const double sb = std::sin(d.param.beta_angle), cb = std::cos(d.param.beta_angle);
    const double sx = std::sin(d.param.xi_angle), cx = std::cos(d.param.xi_angle);
    const double lo[3] = {-1.6, -2.5, -4.0}, hi[3] = {1.2, 3.0, 6.0}; // room in the physical frame (+X down)
    for (uint32_t j = 0; j < t.point_num && j < 300; j++)
    {
        double alpha = d.angle_min + d.param.alpha_angle_bias + j * (double)d.angle_increment;
        double theta = d.com_horizontal_angle_start + d.param.theta_angle_bias + j * (double)d.com_horizontal_angle_step;
        double sa = std::sin(alpha), ca = std::cos(alpha), st = std::sin(theta), ct = std::cos(theta);
        double A1 = -cb * sx + sb * cx * sa, B1 = ca * cx, C1 = sb * sx + cb * cx * sa;
        double P0[3] = {ct * d.param.b_axis_dist, st * d.param.b_axis_dist, d.param.a_axis_dist};
        double D[3] = {ct * A1 - st * B1, st * A1 + ct * B1, C1};
        double r = 1e9;
        int axis = 0;
        for (int k = 0; k < 3; k++)
        {
            if (std::fabs(D[k]) < 1e-12)
                continue;
            double tk = ((D[k] > 0 ? hi[k] : lo[k]) - P0[k]) / D[k];
            if (tk > 0 && tk < r)
                r = tk, axis = k;
        }
        uint32_t h = Hash(((uint64_t)t.seq << 20) ^ (uint64_t)t.sensor_ns ^ j);
        if (r > 40 || h % 50 == 0)
        {
            d.ranges[j] = 0;
            d.intensities[j] = 0;
            continue;
        }
        double noise_mm = (double)(h % 7) - 3.0;
        double mm = r / d.param.range_scale - d.param.range_bias + noise_mm;
        d.ranges[j] = (uint16_t)std::min(65535.0, std::max(0.0, std::round(mm)));
        double reflect = axis == 0 ? (D[0] > 0 ? 0.3 : 0.8) : 0.6; // floor, ceiling, walls
        double norm = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
        double cosi = std::fabs(D[axis]) / norm;
        double near = std::min(1.0, r);
        double v = 700.0 * reflect * cosi * near / std::max(1.0, r * r / 4.0) + (double)(h >> 8) / (double)(1u << 24) * 8;
        d.intensities[j] = (uint8_t)std::min(255.0, std::floor(v));
    }
    return pkt;
}

static LidarImuData MakeImu(const PacketTiming &t)
{
    LidarImuData d{};
    d.info.seq = t.seq;
    d.info.stamp = TimeStamp{(uint32_t)(t.sensor_ns / 1000000000LL), (uint32_t)(t.sensor_ns % 1000000000LL)};
    d.quaternion[0] = 1;
    uint32_t h = Hash(t.sensor_ns);
    auto n = [&](int s) { return ((double)((h >> s) & 0xff) - 127.5) / 127.5; };
    d.angular_velocity[0] = (float)(0.002 * n(0));
    d.angular_velocity[1] = (float)(0.002 * n(8));
    d.angular_velocity[2] = (float)(0.002 * n(16));
    d.linear_acceleration[0] = (float)(-9.80665 + 0.02 * n(24)); // +X down: at rest the sensor feels +g up
    d.linear_acceleration[1] = (float)(0.02 * n(4));
    d.linear_acceleration[2] = (float)(0.02 * n(12));
    return d;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "usage: replay_capture packetsNNNN.bin out_dir [seconds]\n");
        return 2;
    }
    std::vector<PacketTiming> log;
    if (!ReadPacketLog(argv[1], log) || log.empty())
        return 1;
    double seconds = argc > 3 ? atof(argv[3]) : 1e9;
#if MOUNT_IN_CORE
    CaptureSetup setup;
    setup.mount = kMount;
#else
    GetSensorOrientation() = SensorOrientation::VERTICAL;
    GetGlobalTimeOffsetNs() = 0;
#endif
    ImuTimeline::Config cfg;
    cfg.log = false;
    cfg.host.log = false;

    ContinuousDataWriter writer;
    if (!writer.Open(argv[2]))
        return 1;
    const int target_chunks = 800;
    int64_t t0 = log[0].sensor_ns;
#if NEW_FORMAT
    RecordingInfo info;
    info.lidar_firmware_version = "replay (synthetic packets)";
    info.lidar_hardware_version = "replay (synthetic packets)";
    info.sdk_version = unitree_lidar_sdk_VERSION;
    info.work_mode = 0;
#if MOUNT_IN_CORE
    ChunkAssembler as(cfg, info, setup);
#else
    ChunkAssembler as(cfg, info);
#endif
    for (const PacketTiming &t : log)
    {
        if ((t.host_mono_ns - log[0].host_mono_ns) / 1e9 > seconds || as.ChunksProduced() >= target_chunks)
            break;
        HostTime now{t.host_mono_ns, t.host_unix_ns};
        if (t.type == kPacketTypeImu)
            as.OnImu(MakeImu(t), now);
        else
            as.OnPoints(MakePoints(t, t0), now);
        if (as.ChunkReady())
            writer.AppendChunk(as.TakeChunk());
    }
    writer.AppendChunk(as.Finish());
#else
    // ProcessSensorData's loop before ChunkAssembler (recording format 1).
    const int max_points_per_chunk = 14000;
    int chunks_produced = 0;
    CaptureTiming timing(cfg);
    bool decision_reported = false;
    std::vector<PointDLidar> pts;
    std::vector<OutputImuData> imu;
    for (const PacketTiming &t : log)
    {
        if ((t.host_mono_ns - log[0].host_mono_ns) / 1e9 > seconds || chunks_produced >= target_chunks)
            break;
        HostTime now{t.host_mono_ns, t.host_unix_ns};
        if (t.type == kPacketTypeImu)
            timing.OnImu(MakeImu(t), now, imu);
        else
            timing.OnPoints(MakePoints(t, t0), now, pts, imu);
        if (pts.size() >= max_points_per_chunk)
        {
            chunks_produced++;
            SensorChunk c;
            c.chunk_index = chunks_produced;
            c.points = std::move(pts);
            c.imu_data = std::move(imu);
            timing.TakePacketLog(c.packets);
            if (!decision_reported && timing.Timeline().Decided())
            {
                c.timing_report = timing.Timeline().Report();
                decision_reported = true;
            }
            writer.AppendChunk(c);
            pts = std::vector<PointDLidar>();
            imu = std::vector<OutputImuData>();
        }
    }
    timing.Flush(imu);
    SensorChunk c;
    c.chunk_index = chunks_produced + 1;
    c.points = std::move(pts);
    c.imu_data = std::move(imu);
    timing.TakePacketLog(c.packets);
    c.timing_report = timing.Timeline().Report();
    writer.AppendChunk(c);
#endif
    return writer.CloseAndPackageLAZ() ? 0 : 1;
}
