# Tasks

After every group, run these on the Pi 5 (aarch64):
- `cmake --build build`
- `ctest --test-dir build --output-on-failure`

Groups 1–3 must also leave `tools/golden_replay.sh <base>` byte-identical, where `<base>` is the commit before group 1.

## 1. Target hardware and the manual (docs only)

- [x] 1.1 Replace the Pi Zero 2 W as capture target with "Raspberry Pi 5, 1 or 2 GB RAM" in `CLAUDE.md` (Project) and in the `context` of `openspec/config.yaml` (resource limits: 1 GB RAM, the SD card write rate). Leave the earlier OpenSpec changes as they are, except task 10.3 of `clean-architecture-sensor-port`, which is reworded to a 1 GB Pi 5. Verify `grep -rn "Zero 2" CLAUDE.md openspec/config.yaml docs/` finds nothing.
- [x] 1.2 `git mv docs/how-dlidar-works.html docs/manual.html`; retitle it "dLidar manual" and update its hardware text. Update the links in `docs/tools.html`, `docs/recording-format-2.html` and `CLAUDE.md` (Reference Docs). Verify `grep -rn how-dlidar-works --exclude-dir=build* .` finds only `openspec/changes/` history, and every `href` in `docs/*.html` points to an existing file.

## 2. Settings file and sensor factory (L2 only, byte-identical)

- [x] 2.1 Add `ParseAxisMap(text, AxisMap&, std::string& error)` to `src/domain/geometry/axis_map.h`, the inverse of `Describe()`. Extend `tests/domain/test_axis_map.cpp`: all 48 signed permutations round-trip through `Describe`; `"+Z +Y -X"` equals today's `kMount`; malformed text (`"+Z +Y"`, `"+Z +Z -X"`, `"Z Y X"`, extra spaces) is rejected with a message. Verify with ctest `axis_map`.
- [x] 2.2 Write `src/app/settings.{h,cpp}`, the TOML subset of design D1, using one schema table per section: `[device]` `sensor`, `target_chunks`, `imu_time`; `[unitree_l2]` `lidar_ip`, `lidar_port`, `local_ip`, `local_port`, `work_mode`, `mount`; `[livox_mid360s]` `lidar_ip`, `host_ip`, the five host ports, `mount`, `drop_low_confidence`. The defaults are today's `device_config.h` values (Mid-360S: 192.168.1.50 host, 56101–56501, mount `"+Z +Y -X"`). Add `tests/app/test_settings.cpp` covering:
  - no file means the defaults;
  - a full file;
  - comments and blank lines;
  - each unsupported construct (array, inline table, multi-line string, dotted key) with its line number;
  - an unknown section or key;
  - the wrong type;
  - a port out of range;
  - an invalid mount;
  - an unknown sensor.

  Add a CMake target `dlidar_app_settings` (the `src/app` sources besides `main.cpp`) so the test links it. Verify with ctest `settings`.
- [x] 2.3 Write `src/app/sensor_factory.{h,cpp}` with only the L2 case for now (design D3): `ParseSensorKind`, `OpenSensor(const DeviceSettings&)`, and the packet-state header for the storage. Change `main.cpp`:
  - load the settings first (`--config PATH`, else `~/.config/dlidar/dlidar.toml` if present);
  - print the source ("settings: <path>" or "settings: built-in defaults") to stderr, so the console log that golden compares is unchanged;
  - map the settings to `SessionConfig`, letting `--imu-time` override `imu_time`;
  - open the sensor through the factory.

  `device_config.h` keeps only the defaults. Verify:
  - `bin/dLidar --config /nonexistent` exits 2 before touching the output directory;
  - a file with a typo exits 2 with the line number;
  - `tools/golden_replay.sh` is identical.
- [x] 2.4 Add `--sensor=unitree_l2` to `tools/replay_capture.cpp` (the default; other values are rejected until group 5). Rewrite the comments in `src/domain/recording/packet_timing.h` (102/104 are dLidar's packet codes, shared with the L2) and in `time_alignment.h` (the `_l2` keys mean "the sensor's clock"). Verify `tools/golden_replay.sh` is identical.
- [x] 2.5 Document the settings file: every key, its default and its range. It goes in `docs/manual.html` (new "Settings" section), `docs/architecture.md` (`src/app`: settings and factory; "How to add a sensor" step 4 names the factory) and `CLAUDE.md` (Runtime Assumptions: from the settings file, defaults in `device_config.h`). Verify that an example file copied from the manual is read without error by `bin/dLidar --config` (stop it with Ctrl-C during the L2 start-up).

## 3. Recording profile (refactor, byte-identical)

- [x] 3.1 Add `src/domain/recording/recording_profile.h` (design D4), with a static_assert on the LAS 31-character limits for each text. Add `l2::kRecordingProfile` in `src/adapters/unitree_l2/l2_packet_state.h`, holding today's `kLazExtraAttributes` descriptions, the packet-state header, sensor id 1 and model `unitree_l2`. Verify the build passes.
- [x] 3.2 Make `FileRecordingStorage`, `ContinuousDataWriter`, `RecoverInterruptedRecording` and `laz_encoder.cpp` take a current `RecordingProfile` plus the list of known profiles, in place of the bare header and the fixed descriptions. Do not use the system identifier yet, so the bytes are unchanged. Update `main.cpp`, `replay_capture.cpp` and `tests/adapters/storage/test_datawriter.cpp`. Verify that ctest `datawriter` passes and `tools/golden_replay.sh` is identical.

## 4. Recording format 3

- [ ] 4.1 Make these changes:
  - set `kRecordingFormatVersion = 3`;
  - give `RecordingInfo` the fields `sensor_model` and an optional `imu_position_m`, and have `AddRecordingInfo` write `sensor_model` after `recording_format_version` and `imu_position_m` only when set;
  - have `SensorInfo` carry the model and the IMU position in the sensor frame;
  - map the IMU position through the mount (select and negate) where `RecordSession` adds the recording info;
  - fill `sensor_model` in `UnitreeL2Sensor`; the L2 has no IMU position.

  Extend `tests/application/test_capture_timing.cpp` or `test_record_session.cpp`: with `FakeSensor`, the key is present when a position is given, absent when not, and mapped by the mount. Verify with ctest.
- [ ] 4.2 Write points temp files with magic `dLptsv3\n` and the sensor id in the reserved u32. `DetectPointsTempLayout` returns the id (1 for format 1 and 2 files). Recovery picks the matching profile, and an unknown id keeps the temp file and reports it. Set the LAS `system_identifier` from the profile. Extend `test_datawriter`:
  - format 1, 2 and 3 leftovers are each finalized;
  - a format 3 leftover with sensor id 2 gets profile 2's descriptions even when the current profile is the L2's;
  - an unknown id is kept.

  Verify with ctest `datawriter`.
- [ ] 4.3 Run `tools/golden_replay.sh` against the commit before group 4 and `tools/verify_recording_format.py` on its `base/rec` and `new/rec` (`compare_recording_formats.sh` only builds format 1 revisions; the verifier is generalized so the format version may increase and the extra-byte values are compared). The standard LAZ fields, the extra-byte values and the IMU CSV must be identical. The golden replay may differ only in the timing JSON (`recording_format_version`, `sensor_model`) and the LAZ header (`system_identifier`, `generating_software`). Record both outputs in the commit message, and update the golden base in `tools/golden_replay.sh`'s default to this commit.
- [ ] 4.4 Turn `docs/l2-recording-format.md` into the recording format reference:
  - the format 3 section;
  - the timing JSON keys;
  - the temp header;
  - one extra-bytes meaning column per sensor (L2 and Mid-360S).

  Rename it to `docs/recording-format.md` if the links are updated in the same commit. Add the "dMapper effect" paragraph: nothing dMapper reads changes. Update `CLAUDE.md` (format version 3). Verify that the doc's example keys match a real `timingNNNN.json` from `replay_capture`.

## 5. Livox adapter without the SDK library (conversion, sensor, synthetic source)

- [ ] 5.1 Add `external/Livox-SDK2` as a git submodule, pinned to the latest master commit that has `kLivoxLidarTypeMid360s` and builds with GCC 12, and record its hash and date. Copy nothing: the adapter includes the submodule's `include/` on `dlidar_livox` only. Verify:
  - `git submodule status` shows the pin;
  - `grep kLivoxLidarTypeMid360s external/Livox-SDK2/include/livox_lidar_def.h` finds it.
- [ ] 5.2 Write `src/adapters/livox_mid360s/livox_packet_source.h` and `livox_conversion.{h,cpp}` (design D7): `ParsePoints`, `ParseImu`, the per-point time, `raw_range` saturation, alpha/theta, packet seq, the tag filter, and the length and data-type checks. Write `livox::kDefaultFilter` (0.1 m, 100 m, intensity 0). Add `tests/adapters/livox_mid360s/test_livox_conversion.cpp`, with hand-built packed packets checking:
  - the units, (1000, −2000, 500) mm → (1, −2, 0.5) m;
  - range √5.25;
  - point i's time T + i·D/95;
  - saturation at 65535;
  - the tag-filter on and off;
  - the IMU g → m/s²;
  - a wrong length and data types 2/3, which are skipped and counted.

  Verify with ctest `livox_conversion`.
- [ ] 5.3 Write `livox_packet_state.h` (header, rows, `livox::kRecordingProfile` with sensor id 2, model `livox_mid360s` and the Mid-360S descriptions), and `livox_mid360s_sensor.{h,cpp}` (`LidarSensor` plus diagnostics, IMU seq unwrap, `Info()` with the IMU position (0.011, 0.02329, −0.04412) m). Add a CMake target `dlidar_livox`, without the SDK library yet. Verify:
  - ctest `architecture` passes;
  - a test feeding a scripted source checks the `Polled` records, `Points()`, the seq unwrap across 65535 → 0, and the diagnostics rows.
- [ ] 5.4 Write `synthetic_livox_source.{h,cpp}`. It produces packed point packets (96 points, about 2083/s) and IMU packets (200 Hz) on a simulated Mid-360S clock, with rate error and offset, batched or stalled delivery and lost packets; it can also replay a `packetsNNNN.bin`. Add `tests/adapters/livox_mid360s/test_livox_time_alignment.cpp`, the counterpart of `test_l2_time_alignment.cpp`, running 10 min through `LivoxMid360sSensor` and `CaptureTiming`:
  - auto mode picks `sensor`;
  - the IMU and point times agree on the sensor clock;
  - the fallback triggers when the IMU stamps are corrupted;
  - lost packets show as gaps.

  Verify with ctest `livox_time_alignment`.
- [ ] 5.5 Add the Mid-360S case to `sensor_factory.cpp` for profiles, and make `replay_capture --sensor=livox_mid360s` use `SyntheticLivoxSource`. Verify that `bin/replay_capture packets0001.bin /tmp/x 60 --sensor=livox_mid360s` writes a complete format 3 recording:
  - `sensor_model livox_mid360s`;
  - the IMU position key;
  - `packetstate` with the Livox columns;
  - a LAZ readable with `~/venv-laspy`, with the Mid-360S descriptions.

## 6. Live Mid-360S through Livox-SDK2

- [ ] 6.1 Build the SDK's static library from the submodule (design D10: `sdk_core` or root with `EXCLUDE_FROM_ALL`, `-w` on its targets, samples not built), and link it privately into `dlidar_livox`. Verify:
  - a clean `cmake --preset ninja && cmake --build build` succeeds on the Pi 5;
  - `ninja -C build -t targets all | grep -c sample` is 0;
  - `dlidar_core` still links nothing.
- [ ] 6.2 Write the callback-to-pull bridge as its own class, `livox_packet_pool.h` (design D8), and add `tests/adapters/livox_mid360s/test_livox_packet_pool.cpp`. A producer thread pushes 2,300 packets/s for 20 s while the consumer stalls for 5 s twice. Check that every packet comes out in order with its arrival time, that the pool grows past 8,192 slots with exactly one warning, and that no packet is lost. Verify with ctest `livox_packet_pool`.
- [ ] 6.3 Write `live_livox_source.{h,cpp}`: config JSON in `$XDG_RUNTIME_DIR`, init with a path, the info-change wait, best-effort commands with 2 s waits and one warning each, the data wait (10 s / 60 s), the 2 s drain, versions (SDK version, firmware or `unknown`), and `Stop` (Sleep, then Uninit). Only one instance may exist. It honours the `StopToken` at every wait. Verify:
  - with no sensor connected, `bin/dLidar` with `sensor = "livox_mid360s"` ends in Error after about 10 s and writes no files;
  - Ctrl-C during the wait ends it within about 0.1 s.
- [ ] 6.4 Add the live Mid-360S case to `OpenSensor`. Verify that `bin/dLidar --config` with each sensor reaches that sensor's start-up messages: the L2 as today, the Mid-360S as "searching for Mid-360S on <host_ip>".
- [ ] 6.5 Write `docs/livox_mid360s.md`:
  - specs;
  - network setup (Pi address in 192.168.1.0/24, the L2 stays on 192.168.0.x, the ports);
  - the SDK pin and known issues #125/#135;
  - frames and the IMU position;
  - the tag filter;
  - what the recordings contain for this sensor (200 Hz IMU, ICM40609, IMU position, FoV −7°…52°, ring always 1);
  - the bring-up checklist (group 8).

  Add the Mid-360S to `docs/architecture.md` (adapter list, CMake targets), `docs/manual.html` ("Connecting a sensor") and `CLAUDE.md` (Build & Run submodule note, adapter paragraph). Verify that ctest `architecture` passes and the docs' file paths exist.

## 7. Integration checks without hardware

- [ ] 7.1 Clean build from a fresh clone (`git clone --recursive`, `cmake --preset ninja`, `cmake --build build`) and full ctest; verify everything passes.
- [ ] 7.2 `bin/dLidar` with no settings file on the L2 rig: start-up, recording, Ctrl-C finalization; verify the files are complete, `timingNNNN.json` says format 3 / `unitree_l2`, and `tools/check_imu_timestamps.py` is clean.
- [ ] 7.3 Measure resources with `replay_capture --sensor=livox_mid360s` over 300 s of synthetic data on the Pi 5, under `/usr/bin/time -v` and inside `systemd-run --scope -p MemoryMax=900M`, writing to the SD card. Verify:
  - no backlog warning;
  - peak RSS is recorded;
  - the temp write rate is about 9.6 MB/s;
  - the finalization time is recorded in `docs/livox_mid360s.md`.

## 8. Bring-up with the Mid-360S (when connected)

- [ ] 8.1 Network: the sensor is found, the firmware version is printed or a warning appears, and point and IMU packets flow. Record which commands time out (#125) in `docs/livox_mid360s.md`.
- [ ] 8.2 Frame and mount check:
  - with the device level, the IMU CSV shows accZ ≈ +1 g;
  - nose down gives the expected accX sign, and left side down the expected accY sign;
  - a wall in front appears at +X in the LAZ.

  Set `mount` in `dlidar.toml` and the default in `settings.cpp` if it differs. Verify by repeating the check.
- [ ] 8.3 Timing:
  - `timingNNNN.json` shows method `sensor` with the auto checks passed and `imu_period_ns` ≈ 5,000,000;
  - in `packetstateNNNN.tsv`, packet k+1's stamp minus packet k's equals `time_interval·100·dot_num/(dot_num−1)/dot_num`, to within 1 µs (this confirms the per-point time formula);
  - `bin/replay_packets packetsNNNN.bin` shows no unexpected gaps.
- [ ] 8.4 Resources over a 5 min recording on a 1 GB Pi 5 (or the dev Pi 5 under `MemoryMax=900M`): no backlog warning; peak RSS and CPU recorded in `docs/livox_mid360s.md`.

## Workflow follow-up

- Archive `clean-architecture-sensor-port` after its hardware tasks are done, then this change. The ADDED-only deltas work in either order.
