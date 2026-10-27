# Spec Delta

## Purpose

Defines the settings file that configures the scanning device: which LiDAR is used, how it is reached on the network and how it is mounted, and the recording limits. The goal is to switch sensors or adapt the device without rebuilding dLidar.

## ADDED Requirements

### Requirement: Settings file location
dLidar SHALL read its settings from `~/.config/dlidar/dlidar.toml`, or from the file given with `--config PATH`. The settings file SHALL never be placed in or read from the recording directory.

#### Scenario: Default location
- **WHEN** dLidar starts without `--config` and `~/.config/dlidar/dlidar.toml` exists
- **THEN** that file's settings are used

#### Scenario: Explicit file
- **WHEN** dLidar starts with `--config /path/to/file.toml`
- **THEN** only that file is read

### Requirement: Defaults without a settings file
Without a settings file at the default location, dLidar SHALL use the built-in defaults: the Unitree L2 at `192.168.0.62:6101`, the host at `192.168.0.2:6201`, work mode 0, the L2's current mount and the current chunk limit. A recording SHALL then be identical to one made before settings files existed, apart from the additions of the current recording format.

#### Scenario: No settings file
- **WHEN** no file exists at the default location and no `--config` is given
- **THEN** dLidar records with the Unitree L2 using the built-in defaults, and prints which settings it used

#### Scenario: Missing explicit file
- **WHEN** `--config` names a file that does not exist
- **THEN** dLidar exits with an error naming the file, before connecting to any sensor or writing any file

### Requirement: Settings file syntax
The settings file SHALL be valid TOML restricted to:
- `[section]` headers
- `key = value` lines, whose value is a double-quoted string, an integer, a decimal number or `true`/`false`
- `#` comments
- blank lines

Any other construct SHALL be rejected.

#### Scenario: Comments and blank lines
- **WHEN** the file contains comments, blank lines and the supported value types
- **THEN** it is read without error

#### Scenario: Unsupported construct
- **WHEN** the file contains an array, an inline table, a multi-line string or a dotted key
- **THEN** dLidar exits with an error giving the file name and line number

### Requirement: Settings are validated before start-up
dLidar SHALL reject an unknown section, an unknown key, a value of the wrong type, an out-of-range value and a mount that is not a proper rotation. It SHALL exit with an error that names the file, the line and the problem, before recovering leftovers, connecting to a sensor or writing any file.

#### Scenario: Misspelled key
- **WHEN** the file contains `lidar_iq = "192.168.0.62"` in a sensor section
- **THEN** dLidar exits with an error naming the file, the line and the unknown key `lidar_iq`

#### Scenario: Invalid mount
- **WHEN** a sensor's `mount` is `"+X +X +Z"` or describes a reflection
- **THEN** dLidar exits with an error that the mount is not a rotation

#### Scenario: Port out of range
- **WHEN** a port value is 0 or above 65535
- **THEN** dLidar exits with an error naming the key and its allowed range

### Requirement: Sensor selection
The `[device]` section's `sensor` key SHALL select the LiDAR as `"unitree_l2"` or `"livox_mid360s"`, with `"unitree_l2"` as the default. dLidar SHALL use only the selected sensor's section for its network setup and mount. The other sensor's section MAY be present, and it SHALL still be validated.

#### Scenario: Livox selected
- **WHEN** the file sets `sensor = "livox_mid360s"`
- **THEN** dLidar records with the Livox Mid-360S using the `[livox_mid360s]` settings

#### Scenario: Unknown sensor
- **WHEN** the file sets `sensor = "velodyne"`
- **THEN** dLidar exits with an error listing the supported sensors

### Requirement: Mount written as axes
A sensor's `mount` SHALL be written as three signed axes, separated by spaces: the sensor axes that become output +X (front), +Y (left) and +Z (up), for example `"+Z +Y -X"`. It SHALL be applied identically to points and IMU samples.

#### Scenario: L2 mount
- **WHEN** the L2's mount is `"+Z +Y -X"`
- **THEN** the recording equals one made with today's hardcoded L2 mount

### Requirement: Command line overrides the file
A command-line option SHALL take precedence over the same setting in the file. This applies to `--imu-time`.

#### Scenario: IMU time mode
- **WHEN** the file sets `imu_time = "auto"` and dLidar is started with `--imu-time=host`
- **THEN** the recording uses the host-aligned IMU time method
