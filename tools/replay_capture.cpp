// Replays the packet sequence of a recording's packetsNNNN.bin (L2 stamps,
// seq, host arrival times, time_increment, point_num) through the capture
// code into a new recording in out_dir (which must exist). No sensor needed.
//
//   bin/replay_capture ~/PointCloudDump/packets0001.bin out_dir [seconds] [target_chunks] [--sensor=unitree_l2]
//
// --sensor picks the sensor whose synthetic packets are replayed (default and
// for now the only one: unitree_l2).
// Production path: SyntheticL2Source (synthetic, deterministic packet
// contents) -> UnitreeL2Sensor (the L2 adapter) -> ScannerController /
// RecordSession -> FileRecordingStorage, with the device's mount. No console
// channel is registered, so only the adapters print. Host times come from the
// log, so two builds fed the same log write the same recording
// (tools/golden_replay.sh).
#include "adapters/storage/file_recording_storage.h"
#include "adapters/storage/packet_log.h"
#include "adapters/unitree_l2/synthetic_l2_source.h"
#include "adapters/unitree_l2/unitree_l2_sensor.h"
#include "app/device_config.h"
#include "application/device/event_hub.h"
#include "application/device/scanner_controller.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    const char *usage = "usage: replay_capture packetsNNNN.bin out_dir [seconds] [target_chunks] [--sensor=unitree_l2]\n";
    std::vector<const char *> args; // the positional arguments
    std::string sensor_name = "unitree_l2";
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.compare(0, 9, "--sensor=") == 0)
            sensor_name = arg.substr(9);
        else
            args.push_back(argv[i]);
    }
    if (args.size() < 2 || sensor_name != "unitree_l2")
    {
        fprintf(stderr, "%s", usage);
        return 2;
    }
    std::vector<dlidar::PacketTiming> log;
    if (!dlidar::ReadPacketLog(args[0], log) || log.empty())
        return 1;
    double seconds = args.size() > 2 ? atof(args[2]) : 1e9;

    dlidar::SessionConfig config;
    config.timing.log = false;
    config.timing.host.log = false;
    config.setup.mount = kMount;
    config.target_chunks = args.size() > 3 ? atoi(args[3]) : 800;

    dlidar::UnitreeL2Sensor sensor(std::make_unique<dlidar::SyntheticL2Source>(std::move(log), seconds));
    dlidar::FileRecordingStorage storage(args[1], dlidar::l2::kRecordingProfile, {dlidar::l2::kRecordingProfile});
    dlidar::EventHub hub; // no output channels

    dlidar::ScannerController controller(sensor, storage, hub, config);
    controller.StartRecording();
    controller.WaitIdle();
    return controller.Status().last_ok ? 0 : 1;
}
