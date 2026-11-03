# Spec Delta

## Purpose

Defines what the Livox Mid-360S delivers to the dLidar core and how dLidar starts, runs and stops it, so that its recordings follow the same files, units, axes and timestamp rules as the Unitree L2's.

## ADDED Requirements

### Requirement: Mid-360S points in SI units on the sensor clock
dLidar SHALL record each Mid-360S point:
- in metres, in the sensor frame before the mount;
- with its reflectivity (0–255) as intensity;
- with its distance from the sensor origin as range;
- with a time on the Mid-360S's own clock, relative to the recording's time zero.

Within a packet, point times SHALL be evenly spaced from the packet stamp over the packet's time interval.

#### Scenario: Unit conversion
- **WHEN** a packet holds a point at (1000, -2000, 500) mm with reflectivity 40
- **THEN** the recorded point before the mount is (1.0, -2.0, 0.5) m with intensity 40 and range √5.25 m

#### Scenario: Per-point time
- **WHEN** a packet stamped T ns has 96 points spanning a time interval of D ns
- **THEN** point i is recorded at T + i·D/95 ns, relative to the time zero

### Requirement: Mid-360S IMU samples
dLidar SHALL record every Mid-360S IMU sample:
- with angular velocity in rad/s and acceleration in g;
- in the same output frame as the points, through the same mount;
- stamped on the same Mid-360S clock and time zero as the points.

#### Scenario: Device at rest and level
- **WHEN** the device is at rest and level with the correct mount configured
- **THEN** the IMU CSV shows accZ ≈ +1 g, accX and accY ≈ 0, and gyro ≈ 0

### Requirement: Mid-360S recordings use the common files
A Mid-360S recording SHALL produce the same set of files and formats as an L2 recording:
- `imuNNNN.csv`, `lidarNNNN.laz`, `timingNNNN.json`, `packetsNNNN.bin`;
- `packetstateNNNN.tsv`, with the Mid-360S's own columns.

Per-point fields the Mid-360S does not have SHALL be 0, or a documented equivalent.

#### Scenario: dMapper reads it
- **WHEN** dMapper reads a folder with a Mid-360S recording
- **THEN** the IMU CSV and the LAZ standard fields have the same columns, units and time base as an L2 recording

### Requirement: No packet loss inside dLidar
dLidar SHALL hand every point and IMU packet that the SDK delivers to the recording. A packet that the network or the sensor lost SHALL show up as a gap in the packet log, not be hidden.

#### Scenario: Burst of packets
- **WHEN** the core is briefly slower than the packet rate, at about 2,300 packets/s
- **THEN** every packet is recorded later in arrival order, and none is dropped

### Requirement: Start-up does not depend on command replies
Starting the Mid-360S SHALL succeed whenever point and IMU data arrive, even if the sensor does not answer configuration commands. A command without an answer SHALL produce a console warning.

#### Scenario: Commands time out, data flows
- **WHEN** the sensor sends data but every configuration command times out
- **THEN** the recording starts after the backlog drain, and the console shows one warning per unanswered command

#### Scenario: No sensor
- **WHEN** no Mid-360S is found within 10 s, or no point data arrives within 60 s
- **THEN** the start fails, the device ends in Error with a warning, and no recording files are written

### Requirement: Mid-360S default point filter
Without other settings, dLidar SHALL drop Mid-360S points closer than 0.1 m or farther than 100 m. It SHALL drop no point for its intensity.

When `drop_low_confidence = true` is set, dLidar SHALL also drop the points the sensor flags as low confidence, both glued-object points and rain/fog/dust points.

#### Scenario: Blind zone
- **WHEN** a point has a range of 0.05 m
- **THEN** it is not recorded

#### Scenario: Low-confidence points kept by default
- **WHEN** a point carries the sensor's low-confidence flag and `drop_low_confidence` is not set
- **THEN** it is recorded

### Requirement: Mid-360S identity in the recording
A Mid-360S recording SHALL record:
- sensor model `livox_mid360s`;
- the firmware version, or `unknown` if the sensor does not answer;
- the SDK version;
- the IMU's position relative to the LiDAR origin in the output frame.

#### Scenario: Timing report
- **WHEN** a Mid-360S recording finishes
- **THEN** `timingNNNN.json` has `"sensor_model": "livox_mid360s"` and the IMU position in metres
