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
- There are no tests. `Testing/` only holds leftover CTest output.
- VS Code: the `build` task runs `cmake --build build`, and "Debug dLidar" launches `bin/dLidar` under lldb.

## Runtime Assumptions (hardcoded)

- LiDAR at `192.168.0.62:6101`, host at `192.168.0.2:6201` (`src/dLidar.cpp`).
- Output goes to `/home/$USER/PointCloudDump/lidar0001.laz` and `imu0001.csv`. The directory must already exist.
- Capture stops after 80 chunks of at least 14000 points each (`ProcessSensorData` in `include/pcd_manager.h`).

## Architecture

Nearly all of the logic is header-only. `src/dLidar.cpp` just initializes the reader and calls `ProcessSensorData`.

- **`include/pcd_manager.h`**: capture loop. It runs a producer/consumer pipeline:
  - The main thread calls `lreader->runParse()` and dispatches on the packet type (IMU or point data). It collects points and IMU samples into a `SensorChunk` and pushes the chunk onto a mutex/condvar queue.
  - A consumer thread drains the queue into `ContinuousDataWriter`.
  - `ImuTimestamper` (`include/imu_timestamper.h`) builds the IMU timestamps. By default (`kUseSystemTimestamp` in `src/dLidar.cpp`) the SDK stamps every packet with its host arrival time, and the L2 sends IMU samples in batches, so arrival spacing is meaningless (~20 µs inside a batch, up to ~6 ms between batches). The timestamper fits the period online (least squares of arrival vs sample index), assigns `t_prev + n·period` (with `n` from the packet `seq` when seq counts IMU packets), never lets `t` be later than arrival, pulls it slowly toward the arrival envelope, and resyncs and logs on gaps over 20 ms. `timestampUnix` is `timestamp` plus one constant offset. The first 256 samples are held until the first fit, and `Flush()` covers captures shorter than that. `ContinuousDataWriter` warns if an IMU timestamp is non-increasing or a point time goes backwards.
  - `tools/repair_imu_timestamps.py` rewrites the timestamps of `imu*.csv` files recorded before this fix, and `tools/check_imu_timestamps.py` reports their ordering and rate.
  - `GetPointCloud` filters points at parse time via `kMinPointRangeM` (0.1 m, rejects housing self-reflection) and `kMinPointIntensity` (5/255, rejects low-reflectivity noise). Tune these two constants for point-cloud clarity.
  - `CheckDirtyPercentage` polls the lens dirty-index every 10 s during capture (non-blocking) and warns past `kDirtyPercentageWarnThreshold` (5%) — relevant for handheld cave/outdoor use where the lens can fog or collect dust mid-run.
- **`include/datawriter.h`** (`ContinuousDataWriter`): writes IMU rows to CSV as they arrive (space-separated, with a header). Points are dumped as raw `PointDLidar` bytes to a temporary `*_temp.bin` while bounds are tracked. On close, the file is re-read and encoded into LAS 1.2 / point format 1 through the LASzip DLL API (`laszip_api.h`), and then the temp file is deleted. This two-stage approach exists because the LAS header needs the point count and bounds before any points are written.
- **`include/unitree_lidar_utilities.h`**: vendor header that has been **heavily modified locally**. Changes to parsing and coordinate conventions belong here:
  - `PointDLidar` stores time as a `double`; the vendor `PointUnitree` uses `float`.
  - `GetSensorOrientation()` is a global switch. `dLidar.cpp` sets it to `VERTICAL`, which remaps both LiDAR points and IMU axes (physical +Z front, +X down → output +X front, +Y left, +Z up). Any change to one axis mapping must be mirrored in the other so points and IMU stay consistent.
  - `GetGlobalTimeOffsetNs()` is a shared zero point, set by whichever IMU or point packet arrives first. All point times (seconds, `double`) and IMU timestamps (ns) are relative to it, which preserves precision downstream.
  - `parseFromImuPacket` converts acceleration from m/s² to **g**. Gyro stays in rad/s.
- `unitree_lidar_sdk.h`, `unitree_lidar_protocol.h`, `udp_handler.h`: unmodified vendor SDK interfaces (SDK version 2.0.4).

## Reference Docs

- `docs/unitree_l2_slam_tuning.md`: L2 hardware specs, the SLAM tuning targets, and how dLidar's output units, frames and timestamps match the SLAM config.
