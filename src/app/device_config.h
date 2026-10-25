#pragma once

#include "domain/geometry/axis_map.h"

#include <stdint.h>

// How the scanning device is built. Shared by bin/dLidar and the replay tool.

// The L2 is mounted with its +Z axis to the front, its cable outlet up (so its
// +X points down) and its +Y to the left: output +X front, +Y left, +Z up.
// Points and IMU both go through this mapping.
const dlidar::AxisMap kMount{dlidar::Axis::PosZ, dlidar::Axis::PosY, dlidar::Axis::NegX};

// The L2 on the device's Ethernet link.
const char *const kLidarIp = "192.168.0.62";
const unsigned short kLidarPort = 6101;
const char *const kLocalIp = "192.168.0.2";
const unsigned short kLocalPort = 6201;

// Sent with setLidarWorkMode at start-up; recorded in timingNNNN.json.
const uint32_t kLidarWorkMode = 0;

// A recording ends after this many chunks or on Ctrl-C. Temporary: a push button will start
// and stop recordings.
const int kTargetChunks = 1500;
