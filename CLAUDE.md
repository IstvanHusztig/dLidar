# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

dLidar is a C++17 capture tool for a **Unitree L2 LiDAR** (running on a Raspberry Pi / aarch64). It connects to the sensor over UDP, streams point-cloud and IMU packets, and writes a single LAZ point cloud plus an IMU CSV. The output is post-processed by **dMapper**, our SLAM solution. Its code is on GitHub next to dLidar: https://github.com/IstvanHusztig/dMapper. Check it when a change affects what dMapper reads (file names, units, axes, timestamps). The capture runs on a Raspberry Pi Zero 2 W; development and testing happen on a Raspberry Pi 5. Changes are planned with OpenSpec (`openspec/`, `/opsx:*` commands; the CLI is in `~/.local/node/bin` on the Pi 5). The README says the project is still in development and not production ready.

## Build & Run

`3rd/` is the LASzip git submodule and must be checked out first:

```bash
git submodule update --init --recursive
cmake --preset ninja          # configures into build/ (Ninja, Debug)
cmake --build build           # binary goes to bin/dLidar
./bin/dLidar
```

- The Unitree SDK comes as a prebuilt static lib at `lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a` (both `aarch64` and `x86_64` are included). Only its headers are available as source, unmodified in `src/adapters/unitree_l2/vendor/` (SDK 2.0.4); only the L2 adapter (CMake target `dlidar_l2`) includes them.
- The build type is hardcoded to Debug in `CMakeLists.txt` (`-O0 -Wall -g`).
- Tests need no sensor: `ctest --test-dir build --output-on-failure`. `tests/test_imu_timestamper.cpp` feeds `ImuTimestamper` simulated arrivals (bursts, random latency, startup dump, drops, stall). `tests/test_datawriter.cpp` covers the termination paths (kill, crash, SIGINT, leftover temp file). `tests/test_time_alignment.cpp` runs a simulated L2 (its own clock with rate error and offset, batched/stalled delivery, lost packets) through `CaptureTiming` with synthetic SDK packets for 10 min. It checks that IMU and point times agree on the L2 clock, covers the fallback cases and packet-log replay, and shows the old method's lag.
- Refactors must not change any output: `tools/golden_replay.sh [BASE_REV]` replays `~/PointCloudDump/packets0001.bin` through BASE_REV (built in a cached git worktree) and the working tree and `cmp`s every recording file and the console log. All targets build with `-ffp-contract=off` so the `-O2` replay matches the `-O0` binary. ctest `architecture` runs `tools/check_architecture.py` (layer include rules under `src/`, see `docs/tools.html`).
- VS Code: the `build` task runs `cmake --build build`, and "Debug dLidar" launches `bin/dLidar` under lldb.

## Runtime Assumptions (hardcoded)

- LiDAR at `192.168.0.62:6101`, host at `192.168.0.2:6201` (`src/dLidar.cpp`).
- Output goes to `/home/$USER/PointCloudDump/`: `imuNNNN.csv` and `lidarNNNN.laz`, one pair per recording, plus `timingNNNN.json`, `packetsNNNN.bin` and `packetstateNNNN.tsv`. dMapper reads every `.csv`/`.las`/`.laz` in the folder, so no other file may use those extensions. Every field is documented in `docs/l2-recording-format.md` (recording format version `kRecordingFormatVersion`, currently 2); changes must only add data and keep the format 1 fields and the IMU CSV identical. Each recording takes the next free number NNNN in the directory (`NextSessionIndex`), so files group by number and earlier recordings are never overwritten. `points_tempNNNN.bin` exists only while recording. The directory must already exist.
- Capture stops after `target_chunks` chunks of at least 14000 points each (`ProcessSensorData` in `include/pcd_manager.h`), or on SIGINT/SIGTERM/SIGHUP, which finalizes normally (`include/shutdown.h`). A second signal kills.
- On start, each leftover `points_tempNNNN.bin` (kill, crash, power loss) is finalized into `recovered_<time>/` before the new recording. `./bin/dLidar --finalize` finalizes it in place and exits.

## Architecture

Nearly all of the logic is header-only. `src/dLidar.cpp` just initializes the reader and calls `ProcessSensorData`.

- **`include/pcd_manager.h`**: capture loop. It runs a producer/consumer pipeline:
  - The main thread calls `lreader->runParse()` and dispatches on the packet type (IMU or point data). `ChunkAssembler` collects points, IMU samples and packet records into a `SensorChunk`, which is pushed onto a mutex/condvar queue. Before recording, the L2 firmware/hardware versions are queried (`QueryRecordingInfo`, bounded wait) for `timingNNNN.json`.
  - A consumer thread drains the queue into `ContinuousDataWriter`.
  - **Timestamps (see `docs/imu_lidar_time_alignment.md`)**: point times and IMU timestamps both come from the L2's own packet stamps (`kUseSystemTimestamp = false`). With `true`, the SDK would overwrite only the IMU stamp with host arrival time, which caused the 50–80 ms IMU lag. `CaptureTiming` (`include/capture_timing.h`) handles each parsed packet and logs its timing. `ImuTimeline` (`include/time_alignment.h`, no SDK dependency) writes IMU times: `sensor` = L2 stamp − zero (+ `kImuOffsetNs`). `auto` (default, `--imu-time=auto|sensor|host`) holds the first 2 s and checks that the L2 IMU stamps increase, are evenly spaced, run at the host rate, and share the points' clock. If not, it uses `host_aligned`: `ImuTimestamper` on the host arrivals, mapped onto the L2 clock by the lower edge of the point packets' (L2 end time, arrival). Each recording gets `timingNNNN.json` (method, decision, drops, clock rates, latencies, old-method lag) and `packetsNNNN.bin` (per-packet arrival + L2 stamp). `bin/replay_packets packetsNNNN.bin` replays a log offline.
  - `ImuTimestamper` (`include/imu_timestamper.h`, standalone, no SDK dependency, now only the `host_aligned` fallback) rebuilds IMU acquisition times from host arrival times, where the L2 sends IMU samples in batches. Model: `t_k = t_0 + k·T + o_k` with `t_k <= arrival_k`, so the clock is the lower edge of the arrivals. `T` is the lower-convex-hull (LP) fit of (k, arrival) over 20 s. Output is delayed 0.5 s, and `o_k` follows the lowest bound `arrival_j - (j-k)·T` of nearby arrivals, slewing at most 0.25% of T per step normally and 1% hard. Gaps are exactly n·T. `k` comes from `seq` if it counts IMU packets (detected at startup). Otherwise lost samples are detected when the lead over the line stays above 0.75·T for 0.2 s. No output for the first ~2 s (until T settles), and those samples are written afterwards. Samples that would be negative are dropped. `timestampUnix` is `timestamp` plus one constant offset. Call `Flush()` at the end. The end-of-capture summary prints the period, seq use, lost samples and anomalies. `ContinuousDataWriter` warns if an IMU timestamp is non-increasing or a point time goes backwards.
  - The capture drains the SDK's packet backlog for 2 s before recording (instead of sleeping), so stale packets stay out of the recording.
  - `bin/replay_capture packetsNNNN.bin out_dir [seconds]` replays a recording's packet timing with synthetic packet contents into a new recording (no sensor). `tools/compare_recording_formats.sh [OLD_REV] ...` builds it against an older commit too and runs `tools/verify_recording_format.py` (needs laspy[lazrs]): use it to prove a recording format change keeps the standard LAZ fields and the IMU CSV identical.
  - `tools/repair_imu_timestamps.py` rewrites the timestamps of `imu*.csv` files recorded before this fix, and `tools/check_imu_timestamps.py` reports their ordering and rate.
  - `CaptureTiming` (the core) owns the time zero (L2 stamp of the first point packet with points), the mount (`kMount`, a `dlidar::AxisMap` in `src/app/device_config.h`, applied by select-and-negate to points and IMU alike), the m/s² → g conversion and the point filter (`dlidar::PointFilter`, L2 defaults `kL2DefaultFilter`: 0.1 m min range against housing self-reflection, 100 m max, intensity ≥ 5/255 against noise; tune those for point-cloud clarity).
  - `CheckDirtyPercentage` polls the lens dirty-index every 10 s during capture (non-blocking) and warns past `kDirtyPercentageWarnThreshold` (5%) — relevant for handheld cave/outdoor use where the lens can fog or collect dust mid-run.
- **`include/datawriter.h`** (`ContinuousDataWriter`): writes IMU rows to CSV per chunk with one `write()` of whole rows (space-separated, with a header, format unchanged). A failed write is cut back, a fatal signal truncates the CSV to its last complete row, and recovery trims a partial last row. Points are dumped as raw 48-byte `PointDLidar` records after a 16-byte header to `points_tempNNNN.bin` (header-less 32-byte records from older builds are still recovered, without extra bytes). `FinalizePointsTemp` reads it twice (bounds, then points) and encodes LAS 1.2 / point format 1 LAZ with 20 extra bytes (`range`, `raw_range`, `sample_index`, `packet_seq`, `alpha`, `theta`) through the LASzip DLL API (`laszip_api.h`) into `lidarNNNN.laz.tmp`, renamed when complete. Point packet state/calibration rows go to `packetstateNNNN.tsv` (`src/adapters/unitree_l2/l2_packet_state.h`). The temp file is deleted last, so an interrupted finalization is redone on the next start (`RecoverInterruptedRecording`).
- **`src/adapters/unitree_l2/`** (the L2 adapter behind the `dlidar::LidarSensor` port in `src/application/ports/lidar_sensor.h`): `UnitreeL2Sensor` turns packets of an `L2PacketSource` (`LiveL2Source` = SDK reader with the start-up sequence: restart, dirtiness/time-delay prints, version queries, 2 s drain; `SyntheticL2Source` = replay of a packet log) into port data, and its packetstate rows into diagnostics. `l2_conversion.cpp` (`l2::ParsePoints`, `l2::ParseImu`) is Unitree's point math with dLidar's changes and gives the **sensor frame** in SI units: point times are `double` seconds relative to the zero passed in, records carry the corrected and raw range, `j`, the packet seq and the alpha/theta angles (`dlidar::PointRecord`, recording format 2, `src/domain/recording/point_record.h`). Changes to L2 parsing belong here; mount, g conversion and filtering are the core's (`CaptureTiming`).

## Reference Docs

- `docs/unitree_l2_slam_tuning.md`: L2 hardware specs, the SLAM tuning targets, and how dLidar's output units, frames and timestamps match the SLAM config.
