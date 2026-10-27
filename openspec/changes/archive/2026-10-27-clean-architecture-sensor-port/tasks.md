# Tasks

Each group is one or more commits. After every group: `cmake --build build`, `ctest --test-dir build --output-on-failure`, and (from group 2 on) `tools/golden_replay.sh <commit0>` reports all files identical. Run on the Pi 5 (aarch64).

## 1. Baseline (commit 0)

- [x] 1.1 Get the pending `include/pcd_manager.h` change (`target_chunks` 800 → 200) committed or stashed by the user, so the base is clean; verify `git status` shows no changes under `include/`, `src/`, `tests/` or `tools/`
- [x] 1.2 Add `-ffp-contract=off` to all targets in `CMakeLists.txt` and to the `g++` line of `tools/compare_recording_formats.sh`; verify `objdump -d bin/replay_capture | grep -c 'fmadd\|fmsub\|fnmadd\|fnmsub'` drops to only vendor/libm functions, and ctest passes
- [x] 1.3 Add `offsetof`/`sizeof` static_asserts for every field of `PacketTiming` (40 B) and `PointDLidar` (48 B); verify the build passes
- [x] 1.4 Record this commit's hash as the golden base in the change's `design.md` Migration Plan; verify that hash is written there

## 2. Verification tooling

- [x] 2.1 Write `tools/golden_replay.sh BASE_REV [PACKETS] [SECONDS]`: it builds BASE_REV in a temporary `git worktree` (cmake, `-ffp-contract=off`), runs both `replay_capture` builds on the same log, `cmp`s every output file and diffs the console logs; verify it reports "identical" for HEAD against itself, and reports a difference when one byte of the new LAZ is altered by hand
- [x] 2.2 Write `tools/check_architecture.py` (layer include rules from design D8, permissive mode until group 7) and register it as ctest `architecture`; verify it passes now, and fails for a test file under `src/domain/` that includes `unitree_lidar_sdk.h`
- [x] 2.3 Document both tools in `docs/tools.html` and in CLAUDE.md (Build & Run); verify the documented commands run as written

## 3. Domain layer

- [x] 3.1 Create `src/domain/recording/` (`PointRecord`, `ImuSample`, `PacketTiming` + type codes + `PacketDurationNs`, `SensorChunk`, `RecordingInfo`, `kRecordingFormatVersion`) in `namespace dlidar`, with aliases from the old names; verify the build passes and golden is identical
- [x] 3.2 Move `imu_timestamper.h` and `time_alignment.h` to `src/domain/timing/` (content unchanged apart from the namespace and includes); verify `test_imu_timestamper` and `test_time_alignment` pass and golden is identical
- [x] 3.3 Add `src/domain/geometry/axis_map.h` (signed permutation, proper-rotation check, select-and-negate apply) and `point_filter.h` (float thresholds on range and intensity); add `tests/domain/test_axis_map.cpp` (all 24 proper rotations accepted, the 24 mirrors rejected, ±0.0 and NaN keep their sign, vertical L2 mount maps as today) and `test_point_filter.cpp`; verify both pass in ctest

## 4. Core owns zero, mount, units and filter

- [x] 4.1 Move the time zero from `GetGlobalTimeOffsetNs` into `CaptureTiming` (first packet with `point_num > 0`, `== 0` sentinel, set before conversion, read at chunk collect); the conversion takes the zero as a parameter; verify golden is identical and a new test covers the zero rule (empty packets first, an all-filtered first packet)
- [x] 4.2 Make the conversion emit the sensor frame and m/s²; apply `AxisMap` and `/ 9.80665f` in `CaptureTiming` to points and IMU; delete `GetSensorOrientation`; use explicit float `std::sin`/`std::cos`; verify golden is identical and `test_sdk_conversion` passes
- [x] 4.3 Apply `PointFilter` in the core and pass it into the conversion for early rejection; delete `kMinPointRangeM`/`kMinPointIntensity` from `capture_timing.h` (the values become the L2 default filter); verify golden is identical
- [x] 4.4 Add a test of the in-packet order (log record → state row → points → IMU emitted by `PushPointPacket`) and `ChunkReady` after every packet; verify it passes

## 5. Sensor port and Unitree L2 adapter

- [x] 5.1 Define `src/application/ports/lidar_sensor.h` (design D3) and `stop_token.h`; add `tests/application/fake_sensor.h` (a scripted sensor with no L2 code); verify a test runs `CaptureTiming`/`ChunkAssembler` from `FakeSensor` only
- [x] 5.2 Create `src/adapters/unitree_l2/`: `L2PacketSource` (live `UnitreeLidarReader`, which stamps host time right after `runParse()`), `l2_conversion` (dLidar parts of `unitree_lidar_utilities.h`), `l2_packet_state` (header + `TakeRows(zero)`), and `UnitreeL2Sensor` (`Prepare` = restart, dirty/time-delay prints, version queries, 2 s drain; `Stop`; cached `LensDirtiness`); verify the build passes and `test_sdk_conversion` passes against the adapter
- [x] 5.3 Move the vendor headers into `src/adapters/unitree_l2/vendor/`; reduce `unitree_lidar_utilities.h` to vendor content; verify `test_sdk_conversion` passes (the prebuilt `.a` still links and agrees)
- [x] 5.4 Add `SyntheticL2Source` (packet synthesis moved from `tools/replay_capture.cpp`, logged host times); verify golden is identical when `replay_capture` uses it
- [x] 5.5 Move the L2 tests to `tests/adapters/unitree_l2/` (`test_l2_time_alignment.cpp`, `test_sdk_conversion.cpp`, `test_l2_conversion.cpp`), driven through `UnitreeL2Sensor` with a queued packet source (`l2_feed.h`); the 10-minute L2 clock simulation stays an L2 test because it models the L2 specifically, and sensor independence is proven by the `FakeSensor` tests (`tests/application/`); verify all pass with the same scenario list

## 6. Storage port and adapter

- [x] 6.1 Define `src/application/ports/recording_storage.h` (`Open`, `Append`, `Close`, `RecoverLeftovers`); move `ContinuousDataWriter`, points temp, LAZ encoder, session files, recovery and crash truncation into `src/adapters/storage/` (`.cpp` where LASzip is used); verify golden is identical
- [x] 6.2 Give the packet-state header to the storage at construction (from `Diagnostics()`); write no `packetstate` file when there is no header; verify `test_datawriter` (moved to `tests/adapters/storage/`) passes, including the partial-row recovery, and a new case covers "no diagnostics → no file"

## 7. Use case, device control and app

- [x] 7.1 Create `RecordSession` (was `ProcessSensorData`: queue, consumer thread, storage opened in parallel with `Prepare`, target chunks, finish) depending only on ports; `LensMonitor` without statics; verify a core test records a full session from `FakeSensor` into a memory storage fake
- [x] 7.2 Create `DeviceEvents`, `EventHub` (sync and async registration; async = bounded queue, own thread, exceptions caught) and the console channel reproducing today's texts (banner, dirty-lens warning, timing summary, completion); verify a test where a blocking and a throwing channel do not change a `FakeSensor` recording, and golden console logs are identical
- [x] 7.3 Create `ScannerController` (state machine, `StartRecording`/`StopRecording`/`Status`/`WaitIdle`, start rejected outside Idle, `StopToken` = command or signal flag); verify tests for every scenario in `specs/device-control/spec.md` with fake input/output channels
- [x] 7.4 Rewrite `src/app/main.cpp` as the composition root (`device_config.h`: endpoints, work mode, mount, constants; flags and exit codes unchanged; `--finalize` opens no sensor) and move the signal installation to `src/app/signals.h` (header-only, async-signal-safe handlers); `replay_capture` uses `ScannerController` + `RecordSession` with `target_chunks` as a parameter; verify golden is identical, and `bin/dLidar --finalize` plus a bad flag give today's output and exit codes

## 8. Enforce the boundaries

- [x] 8.1 Create CMake targets `dlidar_core` (include path `src/` only), `dlidar_l2` and `dlidar_storage`; delete `include/`; verify the core tests build in a target without the vendor or `3rd/` include paths
- [x] 8.2 Switch `check_architecture.py` to strict mode; verify ctest `architecture` passes and the 2.2 negative check still fails as expected
- [x] 8.3 Update `tools/compare_recording_formats.sh` for the new layout (build the old revision in a worktree like `golden_replay.sh`); verify it runs against `d68191b` and `verify_recording_format.py` passes

## 9. Documentation

- [x] 9.1 Write `docs/architecture.md` (layers, dependency rule, ports, data flow, how to add a sensor / input channel / output channel, deferred items such as the JSON serializer and format 3); verify every path it names exists
- [x] 9.2 Update CLAUDE.md (Architecture, Build & Run, file paths) and the file references in `docs/*.md`; verify `grep -rn 'include/' CLAUDE.md docs/*.md` finds no stale paths

## 10. Integration checks

- [x] 10.1 Full golden replay of `~/PointCloudDump/packets0001.bin` (whole log) against commit 0; verify every file is identical
- [ ] 10.2 Live capture on the Pi 5 with the L2: start-up prints, recording, Ctrl-C finalization, a second-signal kill, and recovery on the next start; verify files are complete and `tools/check_imu_timestamps.py` is clean **Dropped 2026-10-27:** the L2's IMU is broken and will not be replaced soon; not done.
- [ ] 10.3 The same live capture on a Pi 5 with 1 GB RAM (or the dev Pi 5 under `systemd-run --scope -p MemoryMax=900M`; the Pi Zero 2 W was dropped as target); verify no dropped chunks or warnings beyond the baseline, and similar CPU/RAM use (`/usr/bin/time -v`) **Dropped 2026-10-27:** the L2's IMU is broken and will not be replaced soon; not done.
- [ ] 10.4 Import the live recording into dMapper with its configuration unchanged; verify the trajectory and map look like a pre-refactor recording of the same scene **Dropped 2026-10-27:** the L2's IMU is broken and will not be replaced soon; not done.

## 11. Concurrency review (high effort)

- [x] 11.1 Review `ScannerController`, `EventHub`, `RecordSession`, `signals.h` (own review plus an independent Opus review); stress tests and ASan/UBSan builds (ThreadSanitizer does not run on the Pi 5 kernel)
- [x] 11.2 Controller: one worker thread for its lifetime; the stored state decides whether a start is accepted (no refusal while Idle, start from an Idle callback works); exceptions end in Error instead of `std::terminate`; `WaitIdle` from a callback returns
- [x] 11.3 Session: a storage failure stops the capture (was: unbounded queue, 5.7 GB in 15 s); exceptions join the writer and close the files; Error unless everything was written; backlog warning at 50 chunks
- [x] 11.4 EventHub: async channels held by `shared_ptr` (a detached stuck thread keeps its channel alive), threads start at registration, sync channels get one event at a time, one shared 1 s shutdown deadline
- [x] 11.5 L2 start-up: stop checked during the restart sleeps; dirtiness/time-delay queries time out after 10 s (Error instead of hanging forever); `main` starts no recording after an early stop
- [x] 11.6 Golden replay (60 s and whole log) identical; 100 stress runs of each test binary without failure
- [x] 11.8 No files for a recording that never started: the storage opens once the sensor runs (sensor failure, stop during the start-up); checked with the real L2 adapter against an address without an L2 (Error after 17 s, stop ends within ~1 s, empty output folder)
- [x] 11.9 L2 start-up waits while packets arrive (10 s without a packet, 60 s without the status; the L2 sends points only ~20 s after its restart, so the old 10 s limit always failed). Watchdog: no points for 10 s during the recording stops it and ends in Error with the captured data written (an L2 that restarted mid-recording sent only IMU on its reset clock for 82 s, and the capture waited forever when the link died); tests `TestSensorSilent`, `TestSensorRestarted`; golden replay identical
- [ ] 11.7 On the L2: Ctrl-C during the start-up ends within ~0.1 s; an unplugged L2 ends in Error after ~10 s **Dropped 2026-10-27:** the L2's IMU is broken and will not be replaced soon; not done.

## Workflow follow-up

- Archive the change after review (`/opsx:archive`).
- Follow-up changes: GPIO buttons/LEDs adapter, Bluetooth phone channel, daemon mode, the queue bug when `Open` fails, the timing-report serializer, and second-sensor support (IMU clock capability, format 3 extra bytes).
