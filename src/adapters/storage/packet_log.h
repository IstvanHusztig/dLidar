#pragma once

#include "domain/recording/packet_timing.h"

#include <fstream>
#include <string>
#include <vector>

namespace dlidar
{
    // Reads packetsNNNN.bin: kPacketLogMagic, then PacketTiming records.
    inline bool ReadPacketLog(const std::string &path, std::vector<PacketTiming> &packets)
    {
        std::ifstream in(path, std::ios::binary);
        char magic[sizeof kPacketLogMagic];
        if (!in.read(magic, sizeof magic) || std::string(magic, sizeof magic) != std::string(kPacketLogMagic, sizeof magic))
            return false;
        PacketTiming p;
        while (in.read(reinterpret_cast<char *>(&p), sizeof p))
            packets.push_back(p);
        return true;
    }
}
