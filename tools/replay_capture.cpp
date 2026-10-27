// Replays the packet sequence of a recording's packetsNNNN.bin (L2 stamps,
// seq, host arrival times, time_increment, point_num) through the capture
// code into a new recording in out_dir (which must exist). No sensor needed.
//
//   bin/replay_capture ~/PointCloudDump/packets0001.bin out_dir [seconds] [target_chunks] [--sensor=unitree_l2|livox_mid360s]
//
// --sensor picks the sensor whose synthetic packets are replayed (default
// unitree_l2; livox_mid360s: SyntheticLivoxSource -> LivoxMid360sSensor, with
// the log's timing and its own mount default). --free-mb=N (test hook) makes
// the storage see N MB free, to exercise the low-disk-space stop; it also
// prints the device's events (states, warnings) like bin/dLidar.
// Production path: SyntheticL2Source (synthetic, deterministic packet
// contents) -> UnitreeL2Sensor (the L2 adapter) -> ScannerController /
// RecordSession -> FileRecordingStorage, with the device's mount. No console
// channel is registered, so only the adapters print. Host times come from the
// log, so two builds fed the same log write the same recording
// (tools/golden_replay.sh).
#include "adapters/console/console_events.h"
#include "adapters/livox_mid360s/livox_mid360s_sensor.h"
#include "adapters/livox_mid360s/synthetic_livox_source.h"
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
    const char *usage =
        "usage: replay_capture packetsNNNN.bin out_dir [seconds] [target_chunks] [--sensor=unitree_l2|livox_mid360s]\n";
    std::vector<const char *> args; // the positional arguments
    std::string sensor_name = "unitree_l2";
    long long free_mb = -1;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.compare(0, 9, "--sensor=") == 0)
            sensor_name = arg.substr(9);
        else if (arg.compare(0, 10, "--free-mb=") == 0)
            free_mb = atoll(arg.c_str() + 10);
        else
            args.push_back(argv[i]);
    }
    bool livox = sensor_name == "livox_mid360s";
    if (args.size() < 2 || (!livox && sensor_name != "unitree_l2"))
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
    config.setup.mount = livox ? kLivoxMount : kMount;
    config.target_chunks = args.size() > 3 ? atoi(args[3]) : 800;

    std::unique_ptr<dlidar::LidarSensor> sensor;
    if (livox)
        sensor = std::make_unique<dlidar::LivoxMid360sSensor>(
            std::make_unique<dlidar::livox::SyntheticLivoxSource>(std::move(log), seconds));
    else
        sensor = std::make_unique<dlidar::UnitreeL2Sensor>(std::make_unique<dlidar::SyntheticL2Source>(std::move(log), seconds));
    const dlidar::RecordingProfile &profile = livox ? dlidar::livox::kRecordingProfile : dlidar::l2::kRecordingProfile;
    dlidar::FileRecordingStorage storage(args[1], profile, {dlidar::l2::kRecordingProfile, dlidar::livox::kRecordingProfile});
    if (free_mb >= 0)
        storage.free_bytes = [free_mb](const std::string &) { return (uint64_t)free_mb * 1024 * 1024; };
    dlidar::EventHub hub; // no output channels (golden replays compare the console)
    dlidar::ConsoleEvents console;
    if (free_mb >= 0)
        hub.AddSync(console);

    dlidar::ScannerController controller(*sensor, storage, hub, config);
    controller.StartRecording();
    controller.WaitIdle();
    return controller.Status().last_ok ? 0 : 1;
}
