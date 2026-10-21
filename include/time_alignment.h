#pragma once

// Transitional: moved to src/domain/timing/time_alignment.h.
#include "imu_timestamper.h"
#include "domain/timing/time_alignment.h"

using dlidar::ImuInput;
using dlidar::ImuTimeline;
using dlidar::kPacketLogMagic;
using dlidar::kPacketTypeImu;
using dlidar::kPacketTypePoints;
using dlidar::LowerEdge;
using dlidar::PacketDurationNs;
using dlidar::PacketTiming;
