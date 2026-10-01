# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

dLidar is a C++17 capture tool for a **Unitree L2 LiDAR** (running on a Raspberry Pi / aarch64). It connects to the sensor over UDP, streams point-cloud and IMU packets, and writes a single LAZ point cloud plus an IMU CSV. The output is meant to be post-processed in HDmapper. The README says the project is still in development and not production ready.

## Build & Run

`3rd/` is the LASzip git submodule and must be checked out first:

```bash
git submodule update --init --recursive
cmake --preset ninja          # configures into build/ (Ninja, Debug)
cmake --build build           # binary goes to bin/dLidar
./bin/dLidar
```

- The Unitree SDK comes as a prebuilt static lib at `lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a` (both `aarch64` and `x86_64` are included). Only the headers in `include/unitree_lidar_*.h` / `udp_handler.h` are available as source.
- The build type is hardcoded to Debug in `CMakeLists.txt` (`-O0 -Wall -g`).
- Tests need no sensor: `ctest --test-dir build --output-on-failure`. `tests/test_imu_timestamper.cpp` feeds `ImuTimestamper` simulated arrivals (bursts, random latency, startup dump, drops, stall). `tests/test_datawriter.cpp` covers the termination paths (kill, crash, SIGINT, leftover temp file).
- VS Code: the `build` task runs `cmake --build build`, and "Debug dLidar" launches `bin/dLidar` under lldb.

## Runtime Assumptions (hardcoded)

- LiDAR at `192.168.0.62:6101`, host at `192.168.0.2:6201` (`src/dLidar.cpp`).
- Output goes to `/home/$USER/PointCloudDump/`: `imu.csv` and `lidar0001.laz`, `lidar0002.laz`, ... (`kMaxPointsPerLaz` points each). `points_temp.bin` exists only while recording. The directory must already exist.
- Capture stops after `target_chunks` chunks of at least 14000 points each (`ProcessSensorData` in `include/pcd_manager.h`), or on SIGINT/SIGTERM/SIGHUP, which finalizes normally (`include/shutdown.h`). A second signal kills.
- On start, a leftover `points_temp.bin` (kill, crash, power loss) is finalized into `recovered_<time>/` before the new recording. `./bin/dLidar --finalize` finalizes it in place and exits.

## Architecture

Nearly all of the logic is header-only. `src/dLidar.cpp` just initializes the reader and calls `ProcessSensorData`.

- **`include/pcd_manager.h`**: capture loop. It runs a producer/consumer pipeline:
  - The main thread calls `lreader->runParse()` and dispatches on the packet type (IMU or point data). It collects points and IMU samples into a `SensorChunk` and pushes the chunk onto a mutex/condvar queue.
  - A consumer thread drains the queue into `ContinuousDataWriter`.
  - `ImuTimestamper` (`include/imu_timestamper.h`, standalone, no SDK dependency) rebuilds IMU acquisition times. By default (`kUseSystemTimestamp` in `src/dLidar.cpp`) the SDK stamps every packet with its host arrival time, and the L2 sends IMU samples in batches. Model: `t_k = t_0 + k·T + o_k` with `t_k <= arrival_k`, so the clock is the lower edge of the arrivals. `T` is the lower-convex-hull (LP) fit of (k, arrival) over 20 s. Output is delayed 0.5 s, and `o_k` follows the lowest bound `arrival_j - (j-k)·T` of nearby arrivals, slewing at most 0.25% of T per step normally and 1% hard. Gaps are exactly n·T. `k` comes from `seq` if it counts IMU packets (detected at startup). Otherwise lost samples are detected when the lead over the line stays above 0.75·T for 0.2 s. No output for the first ~2 s (until T settles), and those samples are written afterwards. Samples that would be negative are dropped. `timestampUnix` is `timestamp` plus one constant offset. Call `Flush()` at the end. The end-of-capture summary prints the period, seq use, lost samples and anomalies. `ContinuousDataWriter` warns if an IMU timestamp is non-increasing or a point time goes backwards.
  - The capture drains the SDK's packet backlog for 2 s before recording (instead of sleeping), so stale packets with compressed host stamps stay out of the recording.
  - `tools/repair_imu_timestamps.py` rewrites the timestamps of `imu*.csv` files recorded before this fix, and `tools/check_imu_timestamps.py` reports their ordering and rate.
  - `GetPointCloud` filters points at parse time via `kMinPointRangeM` (0.1 m, rejects housing self-reflection) and `kMinPointIntensity` (5/255, rejects low-reflectivity noise). Tune these two constants for point-cloud clarity.
  - `CheckDirtyPercentage` polls the lens dirty-index every 10 s during capture (non-blocking) and warns past `kDirtyPercentageWarnThreshold` (5%) — relevant for handheld cave/outdoor use where the lens can fog or collect dust mid-run.
- **`include/datawriter.h`** (`ContinuousDataWriter`): writes IMU rows to CSV per chunk with one `write()` of whole rows (space-separated, with a header, format unchanged). A failed write is cut back, a fatal signal truncates the CSV to its last complete row, and recovery trims a partial last row. Points are dumped as raw 32-byte `PointDLidar` records to `points_temp.bin`. `FinalizePointsTemp` reads it twice per chunk (bounds, then points) and encodes LAS 1.2 / point format 1 LAZ through the LASzip DLL API (`laszip_api.h`) into `lidarNNNN.laz.tmp`, renamed when complete. The temp file is deleted last, so an interrupted finalization is redone on the next start (`RecoverInterruptedRecording`). Higher-numbered `lidar*.laz` from an earlier recording in the same directory are removed.
- **`include/unitree_lidar_utilities.h`**: vendor header that has been **heavily modified locally**. Changes to parsing and coordinate conventions belong here:
  - `PointDLidar` stores time as a `double`; the vendor `PointUnitree` uses `float`.
  - `GetSensorOrientation()` is a global switch. `dLidar.cpp` sets it to `VERTICAL`, which remaps both LiDAR points and IMU axes (physical +Z front, +X down → output +X front, +Y left, +Z up). Any change to one axis mapping must be mirrored in the other so points and IMU stay consistent.
  - `GetGlobalTimeOffsetNs()` is a shared zero point, set by whichever IMU or point packet arrives first. All point times (seconds, `double`) and IMU timestamps (ns) are relative to it, which preserves precision downstream.
  - `parseFromImuPacket` converts acceleration from m/s² to **g**. Gyro stays in rad/s.
- `unitree_lidar_sdk.h`, `unitree_lidar_protocol.h`, `udp_handler.h`: unmodified vendor SDK interfaces (SDK version 2.0.4).

## Reference Docs

- `docs/unitree_l2_slam_tuning.md`: L2 hardware specs, the SLAM tuning targets, and how dLidar's output units, frames and timestamps match the SLAM config.
