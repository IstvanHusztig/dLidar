#include "app/settings.h"

#include "app/device_config.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sstream>
#include <vector>

namespace
{
// One value as written in the file.
struct Value
{
	enum Type
	{
		String,
		Integer,
		Decimal,
		Boolean
	} type;
	std::string text; // String: unescaped; others: as written
	long long integer = 0;
	bool boolean = false;
};

const char *TypeName(Value::Type t)
{
	switch (t)
	{
	case Value::String:
		return "a string";
	case Value::Integer:
		return "an integer";
	case Value::Decimal:
		return "a decimal number";
	default:
		return "true or false";
	}
}

// A known key: checks the value and stores it, or returns the problem.
struct Key
{
	const char *section;
	const char *name;
	std::function<std::string(const Value &, DeviceSettings &)> set;
};

std::string Expect(const Value &v, Value::Type type)
{
	return v.type == type ? std::string() : std::string("must be ") + TypeName(type) + ", not " + TypeName(v.type);
}

Key Port(const char *section, const char *name, unsigned short *(*field)(DeviceSettings &))
{
	return {section, name, [field](const Value &v, DeviceSettings &s) {
				std::string e = Expect(v, Value::Integer);
				if (!e.empty())
					return e;
				if (v.integer < 1 || v.integer > 65535)
					return std::string("must be a port from 1 to 65535");
				*field(s) = (unsigned short)v.integer;
				return std::string();
			}};
}

Key Ip(const char *section, const char *name, bool allow_empty, std::string *(*field)(DeviceSettings &))
{
	return {section, name, [allow_empty, field](const Value &v, DeviceSettings &s) {
				std::string e = Expect(v, Value::String);
				if (!e.empty())
					return e;
				in_addr addr;
				if (!(allow_empty && v.text.empty()) && inet_pton(AF_INET, v.text.c_str(), &addr) != 1)
					return std::string("must be an IPv4 address like \"192.168.0.2\"") +
						   (allow_empty ? " (or \"\" for the first one found)" : "");
				*field(s) = v.text;
				return std::string();
			}};
}

Key Mount(const char *section, dlidar::AxisMap *(*field)(DeviceSettings &))
{
	return {section, "mount", [field](const Value &v, DeviceSettings &s) {
				std::string e = Expect(v, Value::String);
				if (!e.empty())
					return e;
				dlidar::AxisMap m;
				std::string error;
				if (!dlidar::ParseAxisMap(v.text, m, error))
					return "\"" + v.text + "\" " + error;
				if (!m.IsProperRotation())
					return "\"" + v.text + "\" is not a rotation (it mirrors the axes)";
				*field(s) = m;
				return std::string();
			}};
}

// One of `allowed` (integers, or decimals too when `decimal`); stored in `out`.
std::string OneOf(const Value &v, const std::vector<double> &allowed, bool decimal, double &out)
{
	if (v.type != Value::Integer && !(decimal && v.type == Value::Decimal))
		return std::string("must be ") + (decimal ? "a number" : "an integer") + ", not " + TypeName(v.type);
	double x = v.type == Value::Integer ? (double)v.integer : strtod(v.text.c_str(), nullptr);
	std::string list;
	for (double a : allowed)
	{
		if (x == a)
		{
			out = x;
			return std::string();
		}
		char t[32];
		snprintf(t, sizeof t, "%g", a);
		list += (list.empty() ? "" : ", ") + std::string(t);
	}
	return "must be one of " + list;
}

const std::vector<Key> &Schema()
{
	using S = DeviceSettings;
	static const std::vector<Key> keys = {
		{"device", "sensor",
		 [](const Value &v, S &s) {
			 std::string e = Expect(v, Value::String);
			 if (!e.empty())
				 return e;
			 if (v.text == "unitree_l2")
				 s.sensor = SensorKind::UnitreeL2;
			 else if (v.text == "livox_mid360s")
				 s.sensor = SensorKind::LivoxMid360s;
			 else
				 return "\"" + v.text + "\" is not supported; use \"unitree_l2\" or \"livox_mid360s\"";
			 return std::string();
		 }},
		{"button", "gpio",
		 [](const Value &v, S &s) {
			 std::string e = Expect(v, Value::Integer);
			 if (!e.empty())
				 return e;
			 if (v.integer < 0 || v.integer > 27)
				 return std::string("must be a GPIO of the 40-pin header, 0 to 27");
			 s.gpio.button_gpio = (int)v.integer;
			 return std::string();
		 }},
		{"led", "gpio",
		 [](const Value &v, S &s) {
			 std::string e = Expect(v, Value::Integer);
			 if (!e.empty())
				 return e;
			 if (v.integer < 0 || v.integer > 27)
				 return std::string("must be a GPIO of the 40-pin header, 0 to 27");
			 s.gpio.led_gpio = (int)v.integer;
			 return std::string();
		 }},
		// Removed: recordings have no length limit any more.
		{"device", "target_chunks",
		 [](const Value &, S &) {
			 return std::string("is no longer supported: recordings are started and stopped with the button "
								"([button]); remove target_chunks");
		 }},
		{"device", "imu_time",
		 [](const Value &v, S &s) {
			 std::string e = Expect(v, Value::String);
			 if (!e.empty())
				 return e;
			 if (v.text == "auto")
				 s.imu_time = dlidar::ImuTimeline::Mode::Auto;
			 else if (v.text == "sensor")
				 s.imu_time = dlidar::ImuTimeline::Mode::Sensor;
			 else if (v.text == "host")
				 s.imu_time = dlidar::ImuTimeline::Mode::HostAligned;
			 else
				 return "\"" + v.text + "\" is not supported; use \"auto\", \"sensor\" or \"host\"";
			 return std::string();
		 }},

		Ip("unitree_l2", "lidar_ip", false, [](S &s) { return &s.unitree_l2.lidar_ip; }),
		Port("unitree_l2", "lidar_port", [](S &s) { return &s.unitree_l2.lidar_port; }),
		Ip("unitree_l2", "local_ip", false, [](S &s) { return &s.unitree_l2.local_ip; }),
		Port("unitree_l2", "local_port", [](S &s) { return &s.unitree_l2.local_port; }),
		{"unitree_l2", "work_mode",
		 [](const Value &v, S &s) {
			 std::string e = Expect(v, Value::Integer);
			 if (!e.empty())
				 return e;
			 if (v.integer < 0 || v.integer > 0xffffffffLL)
				 return std::string("must be from 0 to 4294967295");
			 s.unitree_l2.work_mode = (uint32_t)v.integer;
			 return std::string();
		 }},
		Mount("unitree_l2", [](S &s) { return &s.unitree_l2.mount; }),

		Ip("livox_mid360s", "lidar_ip", true, [](S &s) { return &s.livox_mid360s.lidar_ip; }),
		Ip("livox_mid360s", "host_ip", false, [](S &s) { return &s.livox_mid360s.host_ip; }),
		Port("livox_mid360s", "cmd_data_port", [](S &s) { return &s.livox_mid360s.cmd_data_port; }),
		Port("livox_mid360s", "push_msg_port", [](S &s) { return &s.livox_mid360s.push_msg_port; }),
		Port("livox_mid360s", "point_data_port", [](S &s) { return &s.livox_mid360s.point_data_port; }),
		Port("livox_mid360s", "imu_data_port", [](S &s) { return &s.livox_mid360s.imu_data_port; }),
		Port("livox_mid360s", "log_data_port", [](S &s) { return &s.livox_mid360s.log_data_port; }),
		Mount("livox_mid360s", [](S &s) { return &s.livox_mid360s.mount; }),
		{"livox_mid360s", "imu_rate_hz",
		 [](const Value &v, S &s) {
			 double x = 0;
			 std::string e = OneOf(v, {50, 100, 200, 500}, false, x);
			 if (e.empty())
				 s.livox_mid360s.imu_rate_hz = (int)x;
			 return e;
		 }},
		{"livox_mid360s", "imu_accel_range_g",
		 [](const Value &v, S &s) {
			 double x = 0;
			 std::string e = OneOf(v, {4, 8, 16, 32}, false, x);
			 if (e.empty())
				 s.livox_mid360s.imu_accel_range_g = (int)x;
			 return e;
		 }},
		{"livox_mid360s", "imu_gyro_range_dps",
		 [](const Value &v, S &s) {
			 return OneOf(v, {2000, 1000, 500, 250, 125, 62.5, 31.25, 15.625}, true, s.livox_mid360s.imu_gyro_range_dps);
		 }},
		{"livox_mid360s", "drop_low_confidence",
		 [](const Value &v, S &s) {
			 std::string e = Expect(v, Value::Boolean);
			 if (!e.empty())
				 return e;
			 s.livox_mid360s.drop_low_confidence = v.boolean;
			 return std::string();
		 }},
	};
	return keys;
}

bool IsBareKeyChar(char c)
{
	return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
}

std::string Trim(const std::string &s)
{
	size_t b = s.find_first_not_of(" \t\r"), e = s.find_last_not_of(" \t\r");
	return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
}

// After a value: only blanks or a # comment may follow.
bool RestIsComment(const std::string &rest)
{
	std::string r = Trim(rest);
	return r.empty() || r[0] == '#';
}

// Parses the value part of `key = value`. Empty result on success.
std::string ParseValue(const std::string &raw, Value &v)
{
	if (raw.empty())
		return "missing value";
	char c = raw[0];
	if (c == '[')
		return "arrays are not supported";
	if (c == '{')
		return "inline tables are not supported";
	if (raw.compare(0, 3, "\"\"\"") == 0 || raw.compare(0, 3, "'''") == 0)
		return "multi-line strings are not supported";
	if (c == '\'')
		return "literal strings ('...') are not supported; use \"...\"";
	if (c == '"')
	{
		v.type = Value::String;
		for (size_t i = 1; i < raw.size(); i++)
		{
			char ch = raw[i];
			if (ch == '"')
				return RestIsComment(raw.substr(i + 1)) ? std::string() : "unexpected text after the string";
			if (ch == '\\')
			{
				if (++i >= raw.size())
					break;
				switch (raw[i])
				{
				case '"':
				case '\\':
					v.text += raw[i];
					break;
				case 'n':
					v.text += '\n';
					break;
				case 't':
					v.text += '\t';
					break;
				default:
					return std::string("unsupported escape \\") + raw[i];
				}
				continue;
			}
			v.text += ch;
		}
		return "unterminated string";
	}
	size_t end = raw.find('#');
	std::string token = Trim(raw.substr(0, end));
	v.text = token;
	if (token == "true" || token == "false")
	{
		v.type = Value::Boolean;
		v.boolean = token == "true";
		return std::string();
	}
	// Integer or decimal: [+-]digits[.digits], nothing else (no _, exponent, hex).
	size_t i = (token[0] == '+' || token[0] == '-') ? 1 : 0;
	size_t digits = 0, dot = 0, frac = 0;
	for (; i < token.size(); i++)
	{
		if (token[i] >= '0' && token[i] <= '9')
			(dot ? frac : digits)++;
		else if (token[i] == '.' && !dot)
			dot = 1;
		else
			return "\"" + token + "\" is not a value (strings need \"...\")";
	}
	if (digits == 0 || (dot && frac == 0))
		return "\"" + token + "\" is not a value (strings need \"...\")";
	if (dot)
	{
		v.type = Value::Decimal;
		return std::string();
	}
	errno = 0;
	v.integer = strtoll(token.c_str(), nullptr, 10);
	if (errno == ERANGE)
		return "\"" + token + "\" is too large";
	v.type = Value::Integer;
	return std::string();
}
} // namespace

const char *SensorKindName(SensorKind kind)
{
	return kind == SensorKind::LivoxMid360s ? "livox_mid360s" : "unitree_l2";
}

const dlidar::AxisMap &DeviceSettings::Mount() const
{
	return sensor == SensorKind::LivoxMid360s ? livox_mid360s.mount : unitree_l2.mount;
}

DeviceSettings DefaultSettings()
{
	DeviceSettings s;
	s.sensor = SensorKind::UnitreeL2;
	s.unitree_l2.lidar_ip = kLidarIp;
	s.unitree_l2.lidar_port = kLidarPort;
	s.unitree_l2.local_ip = kLocalIp;
	s.unitree_l2.local_port = kLocalPort;
	s.unitree_l2.work_mode = kLidarWorkMode;
	s.unitree_l2.mount = kMount;
	s.livox_mid360s.host_ip = kLivoxHostIp;
	s.livox_mid360s.cmd_data_port = kLivoxHostCmdPort;
	s.livox_mid360s.push_msg_port = kLivoxHostPushPort;
	s.livox_mid360s.point_data_port = kLivoxHostPointPort;
	s.livox_mid360s.imu_data_port = kLivoxHostImuPort;
	s.livox_mid360s.log_data_port = kLivoxHostLogPort;
	s.livox_mid360s.mount = kLivoxMount;
	s.gpio.button_gpio = kButtonGpio;
	s.gpio.led_gpio = kLedGpio;
	s.livox_mid360s.imu_rate_hz = kLivoxImuRateHz;
	s.livox_mid360s.imu_accel_range_g = kLivoxImuAccelRangeG;
	s.livox_mid360s.imu_gyro_range_dps = kLivoxImuGyroRangeDps;
	s.source = "built-in defaults";
	return s;
}

std::string DefaultSettingsPath()
{
	const char *home = getenv("HOME");
	return std::string(home ? home : "") + "/.config/dlidar/dlidar.toml";
}

bool ParseSettings(const std::string &text, const std::string &name, DeviceSettings &settings, std::string &error)
{
	DeviceSettings s = settings;
	std::istringstream in(text);
	std::string line, section;
	std::vector<std::string> seen_sections, seen_keys;
	int number = 0, gpio_line = 0;
	auto fail = [&](const std::string &problem) {
		error = name + ":" + std::to_string(number) + ": " + problem;
		return false;
	};
	while (std::getline(in, line))
	{
		number++;
		std::string t = Trim(line);
		if (t.empty() || t[0] == '#')
			continue;
		if (t[0] == '[')
		{
			if (t.compare(0, 2, "[[") == 0)
				return fail("arrays of tables are not supported");
			size_t close = t.find(']');
			if (close == std::string::npos || !RestIsComment(t.substr(close + 1)))
				return fail("expected [section]");
			std::string sec = Trim(t.substr(1, close - 1));
			bool known = false;
			for (const Key &k : Schema())
				known = known || sec == k.section;
			if (!known)
				return fail("unknown section [" + sec + "]; known: [device], [unitree_l2], [livox_mid360s], [button], [led]");
			for (const std::string &done : seen_sections)
				if (done == sec)
					return fail("section [" + sec + "] appears twice");
			seen_sections.push_back(sec);
			section = sec;
			continue;
		}
		size_t eq = t.find('=');
		if (eq == std::string::npos)
			return fail("expected key = value");
		std::string key = Trim(t.substr(0, eq));
		if (key.empty())
			return fail("missing key before =");
		for (char c : key)
			if (!IsBareKeyChar(c))
				return fail(c == '.' ? "dotted keys are not supported" : "\"" + key + "\" is not a plain key");
		if (section.empty())
			return fail("key \"" + key + "\" is outside a section; put it under [device], [unitree_l2], [livox_mid360s], [button] or [led]");
		const Key *known = nullptr;
		for (const Key &k : Schema())
			if (section == k.section && key == k.name)
				known = &k;
		if (!known)
			return fail("unknown key \"" + key + "\" in [" + section + "]");
		std::string full = section + "." + key;
		for (const std::string &done : seen_keys)
			if (done == full)
				return fail("key \"" + key + "\" appears twice in [" + section + "]");
		seen_keys.push_back(full);
		Value v{};
		std::string problem = ParseValue(Trim(t.substr(eq + 1)), v);
		if (problem.empty())
			problem = known->set(v, s);
		if (!problem.empty())
			return fail(key + ": " + problem);
		if (section == "button" || section == "led")
			gpio_line = number;
	}
	if (s.gpio.button_gpio == s.gpio.led_gpio)
	{
		number = gpio_line;
		return fail("[button] gpio and [led] gpio are both " + std::to_string(s.gpio.led_gpio) +
					"; the LED and the button need different lines");
	}
	s.source = name;
	settings = s;
	return true;
}

bool LoadSettings(const std::string &path, bool explicit_path, DeviceSettings &settings, std::string &error)
{
	std::ifstream in(path);
	if (!in)
	{
		if (!explicit_path && errno == ENOENT)
			return true; // no settings file: the defaults
		error = path + ": cannot read the settings file";
		return false;
	}
	std::stringstream text;
	text << in.rdbuf();
	return ParseSettings(text.str(), path, settings, error);
}
