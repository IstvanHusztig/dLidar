#pragma once

#include "domain/geometry/axis_map.h"

// How the scanning device is built. Shared by bin/dLidar and the replay tool.

// The L2 is mounted with its +Z axis to the front, its cable outlet up (so its
// +X points down) and its +Y to the left: output +X front, +Y left, +Z up.
// Points and IMU both go through this mapping.
const dlidar::AxisMap kMount{dlidar::Axis::PosZ, dlidar::Axis::PosY, dlidar::Axis::NegX};
