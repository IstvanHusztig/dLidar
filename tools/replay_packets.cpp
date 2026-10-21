// Replays a recording's packet timing log through the IMU timestamping.
//
//   bin/replay_packets ~/PointCloudDump/packets0001.bin [imu_offset_ns]
//
// Prints the auto-mode timing report (as in timingNNNN.json) and, per 60 s,
// how far the fallback (host_aligned) and the old host-stamped method are
// from the L2 IMU stamps. No sensor needed.
#include "adapters/storage/packet_log.h"
#include "application/analysis/packet_timing_replay.h"

#include <cstdlib>
#include <iostream>

using namespace dlidar;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "usage: replay_packets packetsNNNN.bin [imu_offset_ns]" << std::endl;
        return 2;
    }
    std::vector<PacketTiming> packets;
    if (!ReadPacketLog(argv[1], packets))
    {
        std::cerr << "Error: " << argv[1] << " is not a dLidar packet log." << std::endl;
        return 1;
    }
    std::cout << packets.size() << " packets in " << argv[1] << std::endl;
    PrintReplay(ReplayPackets(packets, argc > 2 ? std::atoll(argv[2]) : 0), std::cout);
    return 0;
}
