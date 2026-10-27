#pragma once

#include "domain/geometry/axis_map.h"

#include <stdint.h>

// The built-in defaults of the device settings (src/app/settings.h): used when
// there is no settings file, and for every key a settings file leaves out.
// Shared by bin/dLidar and the replay tool.

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

// The push button (GPIO 17, header pin 11, to GND) and the status LED (GPIO
// 27, header pin 13, through 330 ohm to GND); docs/manual.html, "Wiring and pinout".
const int kButtonGpio = 17;
const int kLedGpio = 27;

// The Livox Mid-360S: mounted in the same orientation as the L2 (to be
// confirmed at bring-up, docs/livox_mid360s.md). The host's address and
// ports are Livox's recommended ones; the sensor's own ports are fixed
// (56100-56500) and its IP is found on the network.
const dlidar::AxisMap kLivoxMount{dlidar::Axis::PosZ, dlidar::Axis::PosY, dlidar::Axis::NegX};
const char *const kLivoxHostIp = "192.168.1.50";
const unsigned short kLivoxHostCmdPort = 56101;
const unsigned short kLivoxHostPushPort = 56201;
const unsigned short kLivoxHostPointPort = 56301;
const unsigned short kLivoxHostImuPort = 56401;
const unsigned short kLivoxHostLogPort = 56501;
// The Mid-360S IMU, set at every start-up: 500 Hz for handheld SLAM (the
// sensor's own default is 200 Hz), +-8 g, +-2000 deg/s so fast turns do not clip.
const int kLivoxImuRateHz = 500;
const int kLivoxImuAccelRangeG = 8;
const double kLivoxImuGyroRangeDps = 2000;
