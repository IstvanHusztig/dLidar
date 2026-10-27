# Proposal

## Why

The handheld device should also record with a **Livox Mid-360S**:
- 360°×59° FoV
- 200k pts/s
- built-in 200 Hz IMU

The Unitree L2 must keep working. Today the L2 is wired into `src/app/main.cpp` and its setup is hardcoded in `src/app/device_config.h`, so switching sensors means editing and rebuilding.

The clean-architecture restructure made the core sensor-neutral behind the `LidarSensor` port. A second sensor is now one adapter plus a way to choose it.

The capture computer also changes: a **Raspberry Pi 5 with 1 or 2 GB RAM** replaces the Pi Zero 2 W, which is broken and cannot be replaced. That raises the CPU and memory budget the Mid-360S's three times larger point rate needs.

## What Changes

- **Settings file** `~/.config/dlidar/dlidar.toml` (`--config PATH` to override). It is a strict TOML subset, read by a small parser in `src/app` with no new dependency.
  - It chooses the sensor (`unitree_l2` or `livox_mid360s`) and holds each sensor's network setup and mount, the chunk limit and the IMU time mode.
  - Unknown keys or bad values stop the program with the line number.
  - Without a file, dLidar uses today's L2 values, so nothing changes for an existing setup.
- **Sensor factory** in the composition root: the only code that knows both adapters. The mount (`AxisMap`) can be written as text (`"+Z +Y -X"`).
- **Livox Mid-360S adapter** (`src/adapters/livox_mid360s/`, CMake `dlidar_livox`), built on **Livox-SDK2** (MIT) as a pinned git submodule (`external/Livox-SDK2`).
  - It delivers points and IMU samples in the sensor frame in SI units, on the sensor's own clock. Its per-packet diagnostics go to `packetstateNNNN.tsv`.
  - The SDK's callbacks are bridged to the core's pull model without dropping packets.
  - The start-up treats SDK commands as best effort: a known SDK issue makes commands time out on the Mid-360S while data flows.
  - A synthetic source replays packet timing for tests and `replay_capture --sensor=livox_mid360s`.
- **Recording format 3** (adds data only):
  - `timingNNNN.json` gains `sensor_model` and, when known, the IMU position in the output frame.
  - The LAZ header gains `system_identifier`.
  - The LAZ extra-byte descriptions come from the sensor; names, types and layout are unchanged.
  - Standard LAZ fields and the IMU CSV of L2 recordings stay identical to format 2.
- **Docs and target hardware**:
  - `docs/how-dlidar-works.html` becomes `docs/manual.html`, with a Settings section and a Connecting a sensor section.
  - New `docs/livox_mid360s.md`.
  - `CLAUDE.md`, `docs/architecture.md` and `openspec/config.yaml` describe the Pi 5 1–2 GB target and the settings file.

## Capabilities

### New Capabilities
- `device-settings`: the settings file. Where it is, what it may contain, its defaults, its validation, and how it selects the sensor and its setup.
- `livox-mid360s-sensor`: what the Mid-360S adapter delivers and how it starts, runs and stops, including its units, times, sequence numbers, filter and diagnostics.
- `recording-format`: what a recording records about the sensor that made it (format 3), and the rule that format changes only add data.

### Modified Capabilities
- `sensor-integration` (defined in the not yet archived change `clean-architecture-sensor-port`): adds requirements for sensor identity and recording profile, and for choosing among several sensors at start-up. These are written as ADDED requirements so the two changes can be archived in either order.

## Impact

- **Code**:
  - New `src/app/settings.*` and `src/app/sensor_factory.*`. `main.cpp` and `device_config.h` shrink to the defaults.
  - New `src/adapters/livox_mid360s/*`.
  - Domain: `ParseAxisMap`, `RecordingProfile`, and the new `RecordingInfo` fields.
  - Storage: `FileRecordingStorage` takes a `RecordingProfile`, and `laz_encoder.cpp` uses its descriptions.
  - `tools/replay_capture.cpp` gets `--sensor`.
- **Build**: a new submodule `external/Livox-SDK2`, of which only its static library is built (`git submodule update --init --recursive` now also fetches it), and a new target `dlidar_livox`. `dLidar` and `replay_capture` link both adapters.
- **Output and dMapper**:
  - L2 recordings: `timingNNNN.json` gains keys, and the LAZ header has a new `system_identifier` and `generating_software` text. Points, extra-byte values and the IMU CSV are unchanged, so dMapper reads them as before.
  - Mid-360S recordings use the same files, units, axes and timestamp rules. What differs is the sensor's data: 200 Hz IMU, an IMU-to-LiDAR translation of a few cm (recorded in `timingNNNN.json`), a different FoV, and ring always 1.
- **Runtime (Pi 5, 1–2 GB)**:
  - The Mid-360S writes about 9.6 MB/s to `points_tempNNNN.bin`, against about 3 MB/s for the L2, and its finalization takes about 3× longer.
  - Each second of SD stall holds about 10 MB in the unbounded chunk queue.
- **Out of scope**:
  - Lens-dirtiness reporting from the Livox HMS codes.
  - PTP or GPS time sync.
  - The 500 Hz IMU setting.
  - A recording length in seconds instead of chunks.
