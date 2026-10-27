#pragma once

#include "domain/geometry/axis_map.h"
#include "domain/timing/time_alignment.h"

#include <stdint.h>
#include <string>

// The device settings file (~/.config/dlidar/dlidar.toml): which LiDAR is used,
// how it is reached and mounted, and the recording limits. A strict TOML
// subset: [section], key = value (string, integer, decimal, true/false), #
// comments. Unknown sections or keys and wrong values are errors with the
// line number, so a typo never falls back to a default silently. The defaults
// are the values in src/app/device_config.h.

enum class SensorKind
{
	UnitreeL2,
	LivoxMid360s
};

// "unitree_l2" / "livox_mid360s"
const char *SensorKindName(SensorKind kind);

struct UnitreeL2Settings
{
	std::string lidar_ip;
	unsigned short lidar_port = 0;
	std::string local_ip;
	unsigned short local_port = 0;
	uint32_t work_mode = 0;
	dlidar::AxisMap mount;
};

struct LivoxMid360sSettings
{
	std::string lidar_ip; // empty: the first Mid-360S found on the network
	std::string host_ip;
	unsigned short cmd_data_port = 0;
	unsigned short push_msg_port = 0;
	unsigned short point_data_port = 0;
	unsigned short imu_data_port = 0;
	unsigned short log_data_port = 0;
	dlidar::AxisMap mount;
	bool drop_low_confidence = false;
	int imu_rate_hz = 0;           // 50, 100, 200 or 500
	int imu_accel_range_g = 0;     // 4, 8, 16 or 32
	double imu_gyro_range_dps = 0; // 2000, 1000, 500, 250, 125, 62.5, 31.25 or 15.625
};

// The push button and the status LED on the Pi's 40-pin header (docs/manual.html, "Wiring and pinout").
struct GpioSettings
{
	int button_gpio = 0; // to GND, internal pull-up
	int led_gpio = 0;    // through a resistor to GND, active high
};

struct DeviceSettings
{
	SensorKind sensor = SensorKind::UnitreeL2;
	dlidar::ImuTimeline::Mode imu_time = dlidar::ImuTimeline::Mode::Auto;
	UnitreeL2Settings unitree_l2;
	LivoxMid360sSettings livox_mid360s;
	GpioSettings gpio;
	std::string source; // the file read, or "built-in defaults"

	// The chosen sensor's mount.
	const dlidar::AxisMap &Mount() const;
};

DeviceSettings DefaultSettings();

// ~/.config/dlidar/dlidar.toml
std::string DefaultSettingsPath();

// Reads settings text over the defaults. False with "<name>:<line>: <problem>"
// in `error`.
bool ParseSettings(const std::string &text, const std::string &name, DeviceSettings &settings, std::string &error);

// Reads the file at `path`. A missing file is an error if `explicit_path`
// (given with --config); otherwise the defaults are used.
bool LoadSettings(const std::string &path, bool explicit_path, DeviceSettings &settings, std::string &error);
