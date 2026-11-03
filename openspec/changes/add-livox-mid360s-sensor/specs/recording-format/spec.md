# Spec Delta

## Purpose

Defines what a recording states about itself: the format version, which sensor made it, and the meaning of its per-point extra fields. It also defines the rule that the format only ever grows, so dMapper and older tools keep reading every recording.

## ADDED Requirements

### Requirement: Format changes only add data
Each new recording format version SHALL keep these unchanged in meaning, units, layout and values for the same input:
- the LAZ standard point fields and their order;
- the names, types and layout of every existing extra byte;
- the IMU CSV columns and text.

A new version MAY only add metadata keys, header values or files that dMapper does not read.

#### Scenario: L2 recording in format 3
- **WHEN** the same L2 packet log is replayed through format 2 and format 3 builds
- **THEN** the LAZ standard fields, the extra-byte values and the IMU CSV are identical. Only the timing report keys and the LAZ header identification differ

### Requirement: Recording names its sensor
From format 3 on, every recording SHALL name the sensor model that made it (`unitree_l2` or `livox_mid360s`), both in `timingNNNN.json` (`sensor_model`) and in the LAZ header's system identifier. The recording format version SHALL be 3.

#### Scenario: L2 recording
- **WHEN** an L2 recording finishes
- **THEN** `timingNNNN.json` has `"recording_format_version": 3` and `"sensor_model": "unitree_l2"`, and the LAZ system identifier names the Unitree L2

### Requirement: IMU position recorded when known
When the sensor's IMU position relative to the LiDAR origin is known, the recording SHALL state it in metres in the output frame, after the mount, in `timingNNNN.json`. When it is not known, the key SHALL be absent rather than zero.

#### Scenario: Sensor without a known IMU position
- **WHEN** the sensor does not report an IMU position
- **THEN** `timingNNNN.json` has no IMU position key

### Requirement: Extra-byte descriptions match the sensor
The description text of each LAZ extra byte SHALL describe what the recording's sensor stores in it. Names, types and layout SHALL stay the same for every sensor. Each extra byte's meaning per sensor SHALL be documented in the recording format reference.

#### Scenario: Mid-360S range
- **WHEN** a Mid-360S recording is opened in a LAS reader
- **THEN** the `raw_range` description says it is the distance in mm saturated at 65535, not the L2's raw value

### Requirement: Older recordings stay readable
Recovery and finalization SHALL accept the points temp files of formats 1, 2 and 3. Each recovered recording SHALL be finished in the format it was recorded in.

#### Scenario: Leftover from before the update
- **WHEN** dLidar starts and finds a format 2 `points_tempNNNN.bin` from a killed recording
- **THEN** it is finalized into a complete LAZ with the format 2 extra bytes
