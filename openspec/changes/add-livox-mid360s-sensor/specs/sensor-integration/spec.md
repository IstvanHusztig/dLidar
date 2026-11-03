# Spec Delta

## ADDED Requirements

### Requirement: Several sensors, one chosen at start-up
dLidar SHALL support more than one sensor model in the same build. Exactly one SHALL be chosen at start-up from the settings. Adding a sensor model SHALL NOT require changes to the core: the time alignment, chunking, recording, device control or recovery.

#### Scenario: Same build, other sensor
- **WHEN** the settings switch from `unitree_l2` to `livox_mid360s` and dLidar is restarted
- **THEN** it records with the Mid-360S without being rebuilt

#### Scenario: Core unchanged by a new sensor
- **WHEN** the Mid-360S adapter is added
- **THEN** the core tests pass unchanged, and the architecture check finds no adapter include in the core

### Requirement: Sensor states its identity and recording profile
A sensor SHALL state:
- its model name;
- the descriptions of the per-point extra fields as it fills them;
- its diagnostics columns, if it has any;
- its IMU position relative to its LiDAR origin, in its own frame, if known.

The recording SHALL take these from the chosen sensor and not assume the Unitree L2.

#### Scenario: Sensor without diagnostics
- **WHEN** the chosen sensor has no diagnostics columns
- **THEN** no `packetstateNNNN.tsv` is written for its recordings

### Requirement: Leftovers are finished with the sensor that recorded them
Recovering an unfinished recording SHALL use the sensor identity stored with it, not the currently chosen sensor. Recordings from before format 3 SHALL be treated as Unitree L2 recordings.

#### Scenario: Sensor switched after a crash
- **WHEN** a Mid-360S recording was killed, the settings were switched to the L2, and dLidar starts
- **THEN** the leftover is finalized with the Mid-360S's sensor model and extra-byte descriptions
