# IMU–LiDAR time alignment

dMapper measured that the IMU timestamps in dLidar recordings lag the LiDAR point times by 50–80 ms (recording 44), with the lag shrinking by about 0.1 ms per second. This document covers how each timestamp was made, the root cause, the fix, and the field check that is still needed.

**Status:** fixed in code and covered by simulated tests. Not yet verified on hardware, because the scanner is broken. No raw packet captures from earlier sessions exist on the recording machine, so nothing could be replayed.

## 1. How the timestamps were made (before the fix, commit `e7d2fe9`)

The SDK is a prebuilt library (`lib/aarch64/libunitree_lidar_sdk.a`, version 2.0.4). Its behaviour below comes from disassembling `UnitreeLidarReaderImp::runParse()` (`objdump -d unitree_lidar_reader_imp.cpp.o`).

### SDK: one host time per packet, applied to the IMU only

- `runParse()` reads at most one UDP datagram, then calls `clock_gettime(CLOCK_REALTIME)` once (`runParse+0x9c`) and parses one packet.
- **IMU packet** (type 104, `runParse+0xc74`): the packet is copied into the reader. If `use_system_timestamp` is set, `info.stamp.sec/nsec` is **overwritten with that host time** (`+0xcb8`…`+0xce4`, stores to the packet's `info.stamp`). `getImuData()` / `getLidarImuDataPacket()` return this overwritten stamp.
- **Point packet** (type 102, `+0xe14`): the packet is copied unchanged (`memcpy` of 1044 bytes to `this+0xa58`). `use_system_timestamp` only changes the SDK's own cloud stamp (`+0x1140`: now − `scan_period`; otherwise `info.stamp`). That cloud is used by `getPointCloud()`, which dLidar does not call. **`getLidarPointDataPacket()` (`this+0xa58`) always carries the L2's own stamp.**
- The SDK's convention: a point packet's stamp is the time of its **first** point, and point j is at stamp + j · `time_increment`. This is implied by the system-time path, which reconstructs the start as arrival − `scan_period`.
- `syncLidarTimeStamp()` sends the host time to the L2 once (packet type 106). dLidar never called it in any version. The latency query (`sendRequestOfTimeDelay`, `getTimeDelay`) carries only a counter: it measures the round trip and does not set any clock.

### Point `gps_time`

1. Source: `LidarPointDataPacket.data.info.stamp`, the **L2 clock**, time of the packet's first point.
2. `parseFromPacketToPointCloud` (old `include/unitree_lidar_utilities.h:313-323`): `time = (stamp − zero) / 1e9 + j · time_increment`. Here j is the index within the packet, so filtered points don't shift the others. The per-point offset was accumulated in `float`.
3. `PointDLidar.time` (`double`, s) → `points_tempNNNN.bin` → `WriteLazChunk`: `gps_time = time` (old `include/datawriter.h:276`).

### IMU `timestamp` / `timestampUnix`

1. `initializeUDP(..., use_system_timestamp = true)` (old `src/dLidar.cpp:17`). Before commit `346ae97` the call used the SDK default, which is also `true`. So every IMU stamp was the **host `CLOCK_REALTIME` at parse time**, in every dLidar version.
2. `parseFromImuPacket` (old `unitree_lidar_utilities.h:207-214`): `timestamp = host stamp − zero`.
3. `ImuTimestamper::Push` (old `include/pcd_manager.h:218`) rebuilt a steady clock from those arrival times (`ImuTimestamper`, now `src/domain/timing/imu_timestamper.h`):
   - model `t_k = t_0 + k·T + o_k` with `t_k ≤ arrival_k` (the lower edge of the arrivals);
   - `T` = slope of the lower convex hull of (k, arrival) over 20 s, refit every 0.5 s;
   - output delayed 0.5 s, `o_k` follows the lowest arrival bound and slews at most 0.25 % of T per step;
   - `k` from `seq`, which counts IMU packets.

   This involves no resampling, interpolation or duplication: one row per received packet. Samples are only dropped if they come before zero or are duplicates.
4. `timestampUnix = timestamp + (host Unix at the first sample − its arrival)` (old `imu_timestamper.h:109`). This is constant.
5. Hand-off: the main thread collects samples into a `SensorChunk`, a mutex/condvar queue carries it to the writer thread, and the writer formats the CSV. Times are not changed after `ImuTimestamper`.

### Zero point

The zero (then the global `GetGlobalTimeOffsetNs()`) was set by **whichever packet came first**: the host time if an IMU packet came first, the L2 stamp if a point packet did. Both streams subtract the same constant, so it shifted both equally but never aligned them.

### Where ~890 Hz comes from

It is the number of IMU packets the L2 delivers per second, one sample per packet with consecutive `seq`, measured on the host clock. dLidar wrote exactly one row per packet. Earlier sessions measured 837 Hz and 882 Hz (`docs/unitree_l2_slam_tuning.md`). The rate written into recording 44's grid is the L2's real packet rate, not an artefact of the reconstruction.

Why it differs from the data sheet (1000 Hz sampling, 500 Hz output) can't be settled offline. The L2's own IMU stamps were thrown away, so no earlier recording shows the sensor-side spacing. The new `timingNNNN.json` reports it (`imu_period_ns_l2`, `imu_stamp_spacing_ms_p1/p50/p99`).

### Does the L2 stamp both packet types on one clock?

Yes. Both carry `DataInfo.stamp` from the L2. dLidar used it for points, but the SDK flag replaced it for the IMU.

## 2. Root cause

The **IMU and the points were on two different clocks**:

- IMU: host `CLOCK_REALTIME`, rebuilt as the lower edge of the arrival times.
- Points: the L2's own clock.

For the same physical instant, the written IMU time minus the point time was:

    lag = (host Unix − L2 clock)  +  minimum IMU delivery latency  −  (L2 rate − host rate) · t

This matches every observation:

- **Constant part, 50–80 ms:** the L2 clock was behind the host's Unix time, plus the IMU's minimum delivery latency. How the L2 clock gets close to Unix time at all isn't visible here, since dLidar never synced it. The timing report now measures it every session.
- **Drift, −0.098 ms/s:** the L2 clock runs about 98 ppm fast relative to the host clock. The fitted period was not the cause. The online `ImuTimestamper` stays bounded by the arrival times, so it tracks the host clock without drift; the drift is between the two clocks.
- **Different lag per session:** the L2-vs-host offset is not controlled, and recording 43 used an earlier IMU reconstruction (bursty arrival stamps).
- **Perfect 1.12 ms grid with constant `timestampUnix − timestamp`:** this is the lower-edge reconstruction, on the host clock.

## 3. Fix

All changes are in dLidar; the file format is unchanged.

- `LiveL2Config::use_system_timestamp = false` (`src/adapters/unitree_l2/live_l2_source.h`). The SDK leaves the L2 stamp on every packet. dLidar takes the host arrival time itself (`HostNow()` in `live_l2_source.cpp`, `CLOCK_MONOTONIC` + `CLOCK_REALTIME`), for diagnostics and the fallback only.
- `l2::ParseImu` (`src/adapters/unitree_l2/l2_conversion.cpp`) returns the absolute L2 stamp and no longer touches the zero.
- `CaptureTiming` owns the zero: the L2 stamp of the **first point packet** with points. In `l2::ParsePoints` the point offset is now `j · time_increment` in `double` instead of a running `float` sum.
- `src/application/capture/capture_timing.h` (`CaptureTiming`): handles each packet the sensor returns (`OnPacket`), the same code in the capture loop (`RecordSession`) and the tests, and logs every packet's timing.
- `src/domain/timing/time_alignment.h` (`ImuTimeline`) writes the IMU time on the L2 clock:
  - **sensor:** `timestamp = L2 IMU stamp − zero + kImuOffsetNs` (line 626). Nothing is rebuilt, resampled, interpolated or filtered. A sample whose stamp is not after the previous one, or whose `seq` repeats, is dropped and counted. Lost samples (`seq` gaps) and stamp jumps are counted.
  - **auto** (default) holds the first 2 s and checks the L2 IMU stamps (line 534): ≥ 99 % increasing; ≥ 95 % of steps within 0.5–1.5 × the median, so they are sample times rather than batch send times; rate within 1 % of the host's; and the same clock as the points, with minimum arrival latencies within 100 ms. If the checks pass it uses **sensor**, otherwise **host_aligned**.
  - **host_aligned** (fallback): `ImuTimestamper` rebuilds the IMU clock from the arrivals. Then the lower edge of the point packets' (L2 end-of-packet time, host arrival) over 30 s maps it onto the L2 clock (line 646). The IMU period is thereby measured against the L2 clock and doesn't drift. This assumes both packet types have the same minimum delivery latency.
  - `--imu-time=auto|sensor|host` forces a mode.
  - `timestampUnix = timestamp + one constant per recording`: the zero plus the L2-to-Unix offset at the lower edge of the point packets seen so far (line 567).
- New files per recording, written by the storage adapter (`src/adapters/storage/`) and moved along by crash recovery:
  - `timingNNNN.json`: the IMU time source, the decision and why, the method, `imu_offset_ns`, the zero, the epoch offset, drops, measured clock rates, the L2-side IMU spacing, latencies, and the lag the old method would have written. It is written once the mode is decided, so a killed recording still has it, and again at the end.
  - `packetsNNNN.bin`: the arrival time and L2 stamp of every packet (40 bytes each, `PacketTiming`). `bin/replay_packets packetsNNNN.bin` replays it through all modes and the old method.

## 4. Tests

`ctest --test-dir build --output-on-failure`: all 3 suites pass.

`tests/adapters/unitree_l2/test_l2_time_alignment.cpp` simulates an L2 with its own clock running +98 ppm against the host and 78 ms behind Unix time, like recording 44:

- 889.7 Hz IMU sent in 8 ms batches;
- point packets of 120 points;
- 1 ms minimum latency plus jitter;
- a 40 ms host stall every 7 s;
- 5 IMU packets lost every 3 s.

It builds real `LidarImuData` / `LidarPointDataPacket` structs and runs them through `CaptureTiming` for 10 minutes. For every written IMU sample and point, the residual is the written time minus (the L2 clock at its true measurement time − zero), the same reference for both streams.

| Case | Mode chosen | IMU residual max | Drift (first vs last 60 s) | Points |
|---|---|---|---|---|
| Good stamps (auto) | sensor | 0.0000 ms | 0.0000 ms | 0.0000 ms |
| Duplicated packets (auto) | sensor | 0.0000 ms | 0.0000 ms | 0.0000 ms |
| Smooth delivery (auto) | sensor | 0.0000 ms | 0.0000 ms | 0.0000 ms |
| Good stamps, forced host_aligned | host_aligned | 0.36 ms | 0.002 ms | 0.0000 ms |
| Frozen IMU stamps (auto) | host_aligned | 0.36 ms | 0.002 ms | 0.0000 ms |
| IMU stamps on another clock (auto) | host_aligned | 0.36 ms | 0.002 ms | 0.0000 ms |
| IMU stamps = batch send time (auto) | host_aligned | 0.36 ms | 0.002 ms | 0.0000 ms |

- **Old method on the same data:** +76.2 ms over the first 60 s and +23.3 ms over the last 60 s, i.e. dMapper's observation reproduced.
- **Diagnostics recover the simulated truth:** L2 clock +98.00 ppm, IMU period 1.12408 ms, IMU − point minimum latency +0.04 → +0.12 ms, old-method lag 79.0 → 20.3 ms.
- **Replaying the packet log** gives the same report as the live run.
- **Also covered:** the conversion of one synthetic packet of each type, which is exact; IMU timestamps strictly increasing; `timestampUnix − timestamp` constant.
- `tests/adapters/storage/test_datawriter.cpp` checks that the new files are written and recovered.

## 5. What changes for dMapper

- `imuNNNN.csv` and `lidarNNNN.laz`: **same format, columns, units, axes and names.**
  - `timestamp` and `gps_time` are now on the same clock, so the assumption "IMU timestamp / 1e9 = gps_time for one instant" holds.
  - The IMU spacing is the L2's own; it may differ from the old perfect grid.
  - The zero is now the first point packet, so the few IMU samples before it are dropped.
- `timestampUnix` is still `timestamp` + one constant. It now means the L2 time mapped to host Unix time, including the minimum transport latency of a few ms, instead of the IMU arrival time.
- **New files** `timingNNNN.json` and `packetsNNNN.bin` sit in the same folder. They don't match `imu*.csv` or `lidar*.laz`. If dMapper pairs *all* files in a folder by sorted order, it needs to ignore them.
- **Recordings made before the fix can't be corrected from their files.** They hold no L2 IMU stamps. Re-stamping by the measured lag, as dMapper did for recording 44, remains the way to use them.

## 6. Field check (when the scanner is back)

Build, then record with the default `auto` mode:

```bash
cmake --build build
./bin/dLidar            # Ctrl-C to stop
```

During the first seconds the console prints `IMU time source: sensor (sensor stamps accepted: …)`. At the end it prints the IMU and clock summary.

**Recordings:**

1. **Walk** for at least 60 s. Start with 3–5 s at rest (as the data sheet asks), and include turns.
2. **Rotation test:** in a room, turn the scanner back and forth about its vertical axis a few times.

**Checks, for each recording N:**

1. `timingNNNN.json`:
   - `imu_time_source` is `"sensor"`.
   - `dropped_non_increasing` and `stamp_jumps` are 0 or close to it.
   - `imu_stamp_spacing_ms_p1`…`p99` show the L2's real IMU spacing.
   - `lidar_clock_rate_ppm_vs_host` ≈ `imu_stamp_rate_ppm_vs_host`, within ~1 ppm: one oscillator.
   - `imu_minus_lidar_min_latency_ms_start` ≈ `_end`: no drift between the streams.
   - `old_method_imu_lag_ms_start/_end` should resemble dMapper's measurements on recordings 43/44, tens of ms. That confirms the root cause on real data.
2. `./bin/replay_packets ~/PointCloudDump/packetsNNNN.bin`: per 60 s, how far the old method and the fallback are from the L2 stamps.
3. dMapper `tools/imu_time_offset.py` on `imuNNNN.csv` + `lidarNNNN.laz`. **Pass:** residual offset below 5 ms on every axis and in every 60 s segment, in both recordings.

**If it fails:**

- **Constant residual of the same sign in both recordings, no drift:** the L2's IMU stamp marks a fixed point other than the sample instant (e.g. filter delay). If dMapper reports the IMU r ms *later* than the LiDAR, set `kImuOffsetNs = −r · 1e6` in `src/app/main.cpp`, rebuild and record again. The value is written to `timingNNNN.json` as `imu_offset_ns`.
- **Residual drifts, or `timingNNNN.json` says `host_aligned`:** send `timingNNNN.json` and `packetsNNNN.bin`. A recording with `--imu-time=host` gives the fallback's numbers for comparison.

## 7. Assumptions not yet verified on hardware

- The L2's IMU packet stamp is the sample's measurement time and shares the point packets' clock. The SDK structures and the `auto` checks support this, and the field check measures any remaining offset.
- A point packet's stamp is the time of its first point (the SDK's convention, section 1).
- `host_aligned` is only as good as its assumption that IMU and point packets have the same minimum delivery latency. In simulation it is within 0.4 ms. On hardware it is a fallback only.
