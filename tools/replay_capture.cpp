// Replays the packet sequence of a recording's packetsNNNN.bin (L2 stamps,
// seq, host arrival times, time_increment, point_num) through the capture
// code into a new recording in out_dir (which must exist). No sensor needed.
//
//   bin/replay_capture ~/PointCloudDump/packets0001.bin out_dir [seconds]
//
// The packets come from SyntheticL2Source (synthetic, deterministic contents)
// through UnitreeL2Sensor, the same L2 adapter a live capture uses, with the
// device's mount. Host times come from the log, so two builds fed the same log
// write the same recording (tools/golden_replay.sh).
#include "pcd_manager.h"
#include "packet_replay.h"
#include "adapters/unitree_l2/synthetic_l2_source.h"
#include "adapters/unitree_l2/unitree_l2_sensor.h"
#include "app/device_config.h"

#include <cstdlib>
#include <memory>

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
    ImuTimeline::Config cfg;
    cfg.log = false;
    cfg.host.log = false;

    dlidar::UnitreeL2Sensor sensor(std::make_unique<dlidar::SyntheticL2Source>(std::move(log), seconds));
    dlidar::SensorInfo info = sensor.Info();

    ContinuousDataWriter writer;
    if (!writer.Open(argv[2]))
        return 1;
    const int target_chunks = 800;
    ChunkAssembler as(sensor, cfg, info.versions, kMount, info.default_filter);
    while (as.ChunksProduced() < target_chunks)
    {
        Polled p = sensor.Poll();
        if (p.kind == PacketKind::EndOfStream)
            break;
        as.OnPacket(p);
        if (as.ChunkReady())
            writer.AppendChunk(as.TakeChunk());
    }
    writer.AppendChunk(as.Finish());
    return writer.CloseAndPackageLAZ() ? 0 : 1;
}
