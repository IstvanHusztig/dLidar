// The settings file (src/app/settings.h): the defaults equal device_config.h,
// a full file is read, and every unsupported construct, unknown name and bad
// value is rejected with its line number.
#include "app/device_config.h"
#include "app/settings.h"
#include "../check.h"

#include <cstdio>
#include <string>
#include <unistd.h>

namespace
{
bool Same(const dlidar::AxisMap &a, const dlidar::AxisMap &b)
{
	return a.front == b.front && a.left == b.left && a.up == b.up;
}

// Parses `text` over the defaults; returns the error ("" if it was read).
std::string ParseError(const std::string &text, DeviceSettings *out = nullptr)
{
	DeviceSettings s = DefaultSettings();
	std::string error;
	bool ok = ParseSettings(text, "dlidar.toml", s, error);
	if (out)
		*out = s;
	return ok ? std::string() : error;
}

bool Has(const std::string &text, const std::string &part) { return text.find(part) != std::string::npos; }

void TestDefaults()
{
	DeviceSettings s = DefaultSettings();
	CHECK(s.sensor == SensorKind::UnitreeL2, "default sensor is the L2");
	CHECK(s.unitree_l2.lidar_ip == kLidarIp && s.unitree_l2.lidar_port == kLidarPort, "L2 address");
	CHECK(s.unitree_l2.local_ip == kLocalIp && s.unitree_l2.local_port == kLocalPort, "host address");
	CHECK(s.unitree_l2.work_mode == kLidarWorkMode && Same(s.unitree_l2.mount, kMount), "work mode and mount");
	CHECK(s.target_chunks == kTargetChunks && s.imu_time == dlidar::ImuTimeline::Mode::Auto, "limits");
	CHECK(Same(s.Mount(), kMount), "the chosen sensor's mount");
	CHECK(s.livox_mid360s.lidar_ip.empty() && s.livox_mid360s.host_ip == "192.168.1.50", "Livox addresses");
	CHECK(s.livox_mid360s.cmd_data_port == 56101 && s.livox_mid360s.log_data_port == 56501, "Livox ports");
	CHECK(!s.livox_mid360s.drop_low_confidence, "low-confidence points kept by default");
	CHECK(s.source == "built-in defaults", "%s", s.source.c_str());

	// An empty file or one with only comments changes nothing.
	DeviceSettings e;
	CHECK(ParseError("", &e).empty() && e.unitree_l2.lidar_ip == kLidarIp, "empty file");
	CHECK(ParseError("# nothing\n\n   # indented\n[device]\n", &e).empty() && e.sensor == SensorKind::UnitreeL2,
		  "comments only");
}

void TestFullFile()
{
	const char *text = "# dLidar settings\n"
					   "[device]\n"
					   "sensor = \"livox_mid360s\"   # the Mid-360S\n"
					   "target_chunks = 300\n"
					   "imu_time = \"sensor\"\n"
					   "\n"
					   "[unitree_l2]\n"
					   "lidar_ip = \"10.0.0.62\"\n"
					   "lidar_port = 7101\n"
					   "local_ip = \"10.0.0.2\"\n"
					   "local_port = 7201\n"
					   "work_mode = 8\n"
					   "mount = \"+X +Y +Z\"\n"
					   "\n"
					   "[livox_mid360s]\n"
					   "lidar_ip = \"192.168.1.112\"\n"
					   "host_ip = \"192.168.1.5\"\n"
					   "cmd_data_port = 1\n"
					   "push_msg_port = 2\n"
					   "point_data_port = 3\n"
					   "imu_data_port = 4\n"
					   "log_data_port = 65535\n"
					   "mount = \"-Y +X +Z\"\n"
					   "drop_low_confidence = true\n";
	DeviceSettings s;
	std::string error = ParseError(text, &s);
	CHECK(error.empty(), "%s", error.c_str());
	CHECK(s.sensor == SensorKind::LivoxMid360s && s.target_chunks == 300 &&
			  s.imu_time == dlidar::ImuTimeline::Mode::Sensor,
		  "[device]");
	CHECK(s.unitree_l2.lidar_ip == "10.0.0.62" && s.unitree_l2.lidar_port == 7101 &&
			  s.unitree_l2.local_ip == "10.0.0.2" && s.unitree_l2.local_port == 7201 && s.unitree_l2.work_mode == 8,
		  "[unitree_l2]");
	CHECK(s.unitree_l2.mount.Text() == "+X +Y +Z", "L2 mount %s", s.unitree_l2.mount.Text().c_str());
	CHECK(s.livox_mid360s.lidar_ip == "192.168.1.112" && s.livox_mid360s.host_ip == "192.168.1.5" &&
			  s.livox_mid360s.cmd_data_port == 1 && s.livox_mid360s.push_msg_port == 2 &&
			  s.livox_mid360s.point_data_port == 3 && s.livox_mid360s.imu_data_port == 4 &&
			  s.livox_mid360s.log_data_port == 65535 && s.livox_mid360s.drop_low_confidence,
		  "[livox_mid360s]");
	CHECK(s.Mount().Text() == "-Y +X +Z", "the chosen sensor's mount %s", s.Mount().Text().c_str());
	CHECK(s.source == "dlidar.toml", "%s", s.source.c_str());

	// Keys may be left out; they keep their defaults. Strings take escapes.
	CHECK(ParseError("[livox_mid360s]\nlidar_ip = \"\"\n", &s).empty() && s.livox_mid360s.lidar_ip.empty(),
		  "empty Livox IP means: the first one found");
	CHECK(ParseError("[device]\nsensor=\"unitree_l2\"\n", &s).empty() && s.target_chunks == kTargetChunks,
		  "no blanks around =, other keys default");
}

void TestUnsupported()
{
	struct Case
	{
		const char *text;
		const char *line;
		const char *problem;
	} cases[] = {
		{"[device]\nsensor = [\"unitree_l2\"]\n", ":2:", "arrays"},
		{"[device]\n\nsensor = { a = 1 }\n", ":3:", "inline tables"},
		{"[device]\nsensor = \"\"\"unitree_l2\"\"\"\n", ":2:", "multi-line"},
		{"[device]\nsensor = 'unitree_l2'\n", ":2:", "literal strings"},
		{"[device]\nunitree_l2.lidar_ip = \"1.2.3.4\"\n", ":2:", "dotted keys"},
		{"[[device]]\n", ":1:", "arrays of tables"},
		{"[device]\nsensor = \"unitree_l2\" extra\n", ":2:", "after the string"},
		{"[device]\nsensor = \"unitree_l2\n", ":2:", "unterminated"},
		{"[device]\ntarget_chunks = 1_000\n", ":2:", "not a value"},
		{"[device]\ntarget_chunks = 1e3\n", ":2:", "not a value"},
		{"[device]\ntarget_chunks\n", ":2:", "key = value"},
		{"[device]\n = 5\n", ":2:", "missing key"},
		{"[device]\ntarget_chunks =\n", ":2:", "missing value"},
		{"sensor = \"unitree_l2\"\n", ":1:", "outside a section"},
	};
	for (const Case &c : cases)
	{
		std::string e = ParseError(c.text);
		CHECK(Has(e, std::string("dlidar.toml") + c.line) && Has(e, c.problem), "%s -> \"%s\"", c.text, e.c_str());
	}
}

void TestUnknownAndBadValues()
{
	struct Case
	{
		const char *text;
		const char *line;
		const char *problem;
	} cases[] = {
		{"[device]\n[lidar]\n", ":2:", "unknown section [lidar]"},
		{"[unitree_l2]\nlidar_iq = \"192.168.0.62\"\n", ":2:", "unknown key \"lidar_iq\""},
		{"[device]\nmount = \"+Z +Y -X\"\n", ":2:", "unknown key \"mount\" in [device]"},
		{"[device]\n[device]\n", ":2:", "appears twice"},
		{"[device]\ntarget_chunks = 5\ntarget_chunks = 6\n", ":3:", "appears twice"},
		{"[device]\nsensor = \"velodyne\"\n", ":2:", "\"unitree_l2\" or \"livox_mid360s\""},
		{"[device]\nsensor = 1\n", ":2:", "must be a string"},
		{"[device]\ntarget_chunks = \"200\"\n", ":2:", "must be an integer"},
		{"[device]\ntarget_chunks = 2.5\n", ":2:", "must be an integer, not a decimal"},
		{"[device]\ntarget_chunks = 0\n", ":2:", "from 1"},
		{"[device]\nimu_time = \"later\"\n", ":2:", "\"auto\", \"sensor\" or \"host\""},
		{"[unitree_l2]\nlidar_port = 0\n", ":2:", "lidar_port: must be a port from 1 to 65535"},
		{"[unitree_l2]\nlocal_port = 65536\n", ":2:", "local_port: must be a port from 1 to 65535"},
		{"[unitree_l2]\nwork_mode = -1\n", ":2:", "work_mode"},
		{"[unitree_l2]\nlidar_ip = \"192.168.0\"\n", ":2:", "IPv4 address"},
		{"[unitree_l2]\nlidar_ip = \"\"\n", ":2:", "IPv4 address"},
		{"[unitree_l2]\nmount = \"+X +X +Z\"\n", ":2:", "twice"},
		{"[unitree_l2]\nmount = \"+X +Y -Z\"\n", ":2:", "not a rotation"},
		{"[livox_mid360s]\nmount = \"Z Y X\"\n", ":2:", "three signed axes"},
		{"[livox_mid360s]\ndrop_low_confidence = 1\n", ":2:", "true or false"},
		{"[livox_mid360s]\nhost_ip = \"\"\n", ":2:", "IPv4 address"},
	};
	for (const Case &c : cases)
	{
		std::string e = ParseError(c.text);
		CHECK(Has(e, std::string("dlidar.toml") + c.line) && Has(e, c.problem), "%s -> \"%s\"", c.text, e.c_str());
	}

	// A failed file leaves the settings untouched.
	DeviceSettings s = DefaultSettings();
	std::string error;
	CHECK(!ParseSettings("[device]\nsensor = \"livox_mid360s\"\ntarget_chunks = 0\n", "x", s, error) &&
			  s.sensor == SensorKind::UnitreeL2,
		  "settings unchanged after an error");
}

void TestFiles()
{
	std::string missing = "/nonexistent/dlidar/dlidar.toml";
	DeviceSettings s = DefaultSettings();
	std::string error;
	CHECK(LoadSettings(missing, false, s, error) && s.source == "built-in defaults", "no default file: defaults");
	CHECK(!LoadSettings(missing, true, s, error) && Has(error, missing), "missing --config file: %s", error.c_str());

	char path[] = "/tmp/dlidar_settings_XXXXXX";
	int fd = mkstemp(path);
	CHECK(fd >= 0, "temp file");
	const char text[] = "[device]\nsensor = \"livox_mid360s\"\n";
	CHECK(write(fd, text, sizeof text - 1) == (ssize_t)(sizeof text - 1), "write");
	close(fd);
	s = DefaultSettings();
	CHECK(LoadSettings(path, true, s, error) && s.sensor == SensorKind::LivoxMid360s && s.source == path,
		  "file read: %s", error.c_str());
	unlink(path);

	CHECK(Has(DefaultSettingsPath(), "/.config/dlidar/dlidar.toml"), "%s", DefaultSettingsPath().c_str());
}
} // namespace

int main()
{
	TestDefaults();
	TestFullFile();
	TestUnsupported();
	TestUnknownAndBadValues();
	TestFiles();
	return FinishTests("settings");
}
