# Change: rebuild-imu-timestamps-and-safe-termination

> OpenSpec change document. When `openspec init` is run in this repository,
> split it into `openspec/changes/rebuild-imu-timestamps-and-safe-termination/`:
> the **Proposal** section → `proposal.md`, the **Design** section → `design.md`,
> the **Tasks** section → `tasks.md`, and each `specs/<capability>/spec.md`
> section → that file. Status: implemented on branch `improve_timestamp`;
> verified in simulation only, not yet on real recordings.

---

## Proposal

### Why

The SLAM backend integrates the IMU using the per-sample `timestamp` column of
`imu*.csv`. It needs timestamps that reflect acquisition time at a steady rate,
on the same clock as the LiDAR point times. Offline repair of the timestamps
measurably improved SLAM results on a recent recording and avoided a failure in
a stairwell, so the recorder must produce such timestamps itself.

Problems measured on a 231 s recording (true rate 837 Hz, T = 1.1947 ms):

1. **Startup buffer dump (first ~2 s):** steps of 0.238 ms, gaps of 28 ms and
   63 ms, bursts; the first ~158 samples had negative timestamps (down to −35 ms).
2. **Clock corrections as jumps:** about 1,500 steps of only 0.77–0.90 ms because
   the correction toward arrival time was applied as a step; the period in use
   (~1.201 ms) was ~0.5% too long.
3. **Interrupted recordings:** the last CSV line was cut off mid-number and the
   points stayed in `points_temp.bin` without being converted to LAZ, so the
   SLAM found no point files.

### What Changes

- Rewrite `ImuTimestamper` (`include/imu_timestamper.h`) as a standalone,
  testable component that treats the acquisition clock as the lower edge of the
  host arrival times (model below). Output is delayed 0.5 s; no output until the
  period estimate has settled (~2 s).
- Use the packet `seq` as the sample index when it counts IMU packets (detected
  at runtime); otherwise detect lost samples from the arrival times.
- Write IMU CSV rows only as whole rows; cut back failed writes; truncate the CSV
  to its last complete row on a crash; trim a partial last row on the next start.
- Finalize `points_temp.bin` into `lidar0001.laz`, `lidar0002.laz`, … on every
  termination path: normal end, SIGINT/SIGTERM/SIGHUP, and — after a kill, crash
  or power loss — on the next start (into `recovered_<time>/`), or via
  `./bin/dLidar --finalize`.
- Drain the SDK packet backlog for 2 s before capture instead of `sleep(2)`.
- Write the points still buffered when the capture stops (previously discarded).
- Add sensor-free unit tests and CTest targets.
- **Output file names:** `imu.csv`, `lidarNNNN.laz` (max 20 M points each),
  temporary `points_temp.bin`. CSV format unchanged.

### Impact

- Affected capabilities: `imu-timestamping`, `recording-output`, `capture-lifecycle`.
- Affected code: `include/imu_timestamper.h` (rewritten), `include/datawriter.h`,
  `include/pcd_manager.h`, `include/shutdown.h` (new), `src/dLidar.cpp`,
  `CMakeLists.txt`, `tests/` (new), `CLAUDE.md`, `docs/unitree_l2_slam_tuning.md`.
- **Behavioural changes users will notice:**
  - IMU rows start ~2 s into the recording and lag capture by ~0.5 s internally.
  - IMU samples acquired before time zero (the first packet) are dropped.
  - Higher-numbered `lidar*.laz` left from an earlier recording in the output
    directory are deleted when a recording is finalized.
  - A previous unfinished recording is moved to `recovered_<time>/` on start.
- Not changed: CSV columns/format, units, axis mapping, point timestamps and the
  shared zero point (`GetGlobalTimeOffsetNs`).

---

## Design

### Clock model

The IMU samples at a constant period T. Each sample arrives at the host at
`arrival_k >= acquisition_k` — never earlier, sometimes much later (sensor
batching, host buffering). With `use_system_timestamp` (default) the SDK stamps
each packet with its host arrival time. Hence:

```
t_k = t_0 + k·T + o_k        with   t_k <= arrival_k
```

### Sample index k

- At the first fit (0.5 s of data) the `seq` steps of the buffered IMU packets are
  inspected. If ≥ 90 % are +1, `seq` counts IMU packets: `k` follows `seq`
  (gaps = lost samples; duplicates dropped; jumps > 100 000 treated as a reset).
- Otherwise `k` is counted, and lost samples are detected: if the lead
  `arrival − line(k)` stays above **0.75·T** for **0.2 s** of arrivals, `k` is
  advanced by `round(lead / T)` (lead measured on the later half of the streak)
  from the first streak sample whose lead shows the jump.
  *Deviation from the original brief (1.5·T):* with 1.5·T a single lost packet
  (lead ≈ T + latency floor) is never detected and timestamps get compressed.
- A sample arriving more than 0.5·T before its predicted time means `k` is too
  high (overestimated drop); `k` is moved back.
- Limitation: without a usable `seq`, a sample lost from inside a batch cannot be
  located exactly (the batch arrives at once); the gap may land 1–2 samples early.

### Period T

Slope of the **lower convex hull** of (k, arrival) over the last 20 s, taken at
the hull edge spanning the mean k (the LP clock-skew estimator: the line below
all arrivals that is closest to them on average). Refit every 0.5 s. Chosen over
least squares through 1 s block minima because it needs no prior T and ignores
late arrivals entirely.

### Offset o_k (output)

Output is delayed by 0.5 s. For the oldest pending sample k:

- **Envelope** = min over arrivals j within ±0.5 s of `arrival_j − (j−k)·T`
  (every arrival bounds the acquisition time from above).
- **Ceiling** = min over later pending j of `arrival_j − (j−k)·T + 1%·T·(j−k)`
  (highest t from which all later arrivals stay reachable at the hard slew).
- `t_k = min(ceiling, clamp(envelope, t_prev + n·T ± 0.25%·T))`, n = index step.
  Across a gap the step is exactly n·T (offset moves no more than over one step).
- Strictly increasing: if `t_k <= t_prev` it becomes `t_prev + 1`; if that would
  exceed the arrival, the sample is dropped and counted.
- `t_k < 0` (acquired before the shared zero point) → dropped and counted.
- `timestampUnix = timestamp + epoch_offset`, offset fixed at the first sample.

### Startup and shutdown

- No output until arrivals span 2 s **and** T changed < 0.1 % between fits; held
  samples are then written with times computed backwards from the fitted clock.
- `Flush()` at the end writes everything held.
- Arrival clock stepping back > 50 ms → flush, restart the estimator, last written
  time stays the floor. With `seq`, a lead > 50 ms for 0.2 s → resync (jump).

### Writer and termination

- CSV via POSIX fd: each chunk's rows are formatted into one buffer and written
  with one `write()` loop; on failure `ftruncate` to the last committed size.
  The committed size is published to a fatal-signal handler (`include/shutdown.h`)
  that truncates the CSV before re-raising SIGSEGV/SIGBUS/SIGFPE/SIGILL/SIGABRT.
- SIGINT/SIGTERM/SIGHUP set a stop flag; the capture loop exits through the
  normal path. A second signal kills (default action).
- `FinalizePointsTemp`: per chunk of ≤ `kMaxPointsPerLaz` points, read bounds,
  then encode LAS 1.2 / point format 1 (scale 0.0001) into `lidarNNNN.laz.tmp`,
  rename when complete; delete the temp file last. A trailing partial 32-byte
  record is ignored. Stale higher-numbered `lidar*.laz` are removed.
- `RecoverInterruptedRecording(dir, move_aside)` runs at program start.

### Performance

~2 % of one core at `-O0` for a 231 s recording; one ~40 ms burst when the
startup samples are released.

---

## specs/imu-timestamping/spec.md

## ADDED Requirements

### Requirement: Acquisition-time IMU timestamps
The recorder SHALL assign each IMU sample a timestamp that estimates its
acquisition time as `t_0 + k·T + o_k`, following the lower edge of the host
arrival times, and SHALL never assign a timestamp later than the sample's arrival.

#### Scenario: Batched delivery
- **WHEN** samples arrive in bursts of 4 within 0.02 ms followed by ~4.5 ms gaps
- **THEN** after the first 2 s every timestamp is within 0.2 ms of the true acquisition time
- **AND** every step is within T ± 2 %

#### Scenario: Random transport latency
- **WHEN** each sample arrives 0–3 ms after acquisition
- **THEN** after the first 2 s every timestamp is within 0.2 ms of the true acquisition time
- **AND** no timestamp is later than its arrival

#### Scenario: Long stall
- **WHEN** delivery stalls for 60 ms and the delayed samples then arrive in a rush
- **THEN** the delayed samples keep a steady spacing of T ± 2 % and are not treated as lost

### Requirement: Continuously estimated period
The recorder SHALL estimate the sample period continuously from the arrival
times (lower convex hull over the last 20 s) and SHALL track slow crystal drift.

#### Scenario: Period drift
- **WHEN** the true period drifts by 50 ppm over the recording
- **THEN** timestamps stay within 0.2 ms of the true acquisition times

### Requirement: Slewed clock corrections
The recorder SHALL correct the offset toward the arrival envelope by at most
0.25 % of T per step under normal conditions and at most 1 % of T per step when
forced by an arrival bound, and SHALL NOT apply corrections as steps.

#### Scenario: No step corrections
- **WHEN** the arrival envelope moves
- **THEN** consecutive timestamps differ by n·T within ± 2 % of T

### Requirement: Sample index from sequence number
The recorder SHALL use the packet sequence number as the sample index when it
counts IMU packets, and SHALL otherwise detect lost samples from the arrival times.

#### Scenario: Seq counts IMU packets
- **WHEN** ≥ 90 % of IMU `seq` steps during the first 0.5 s are +1
- **THEN** the sample index follows `seq` and gaps in `seq` become gaps of n·T

#### Scenario: Seq shared with other packets
- **WHEN** IMU `seq` steps are not consistently +1
- **THEN** `seq` is ignored and the summary reports it as not usable

#### Scenario: Dropped samples without seq
- **WHEN** 5 consecutive samples are lost and the lead over the clock line stays above 0.75·T for 0.2 s
- **THEN** the index is advanced by round(lead / T)
- **AND** the step across the gap equals the number of missing samples × T within 2 % of T

### Requirement: Converged startup without negative timestamps
The recorder SHALL NOT write IMU rows before the period estimate has converged
(≥ 2 s of data and < 0.1 % change between fits), SHALL write the held samples
afterwards with timestamps computed backwards from the converged clock, and
SHALL NOT write negative timestamps.

#### Scenario: Startup buffer dump
- **WHEN** the first 300 samples are delivered at once
- **THEN** they are written with steady spacing and accurate times

#### Scenario: Samples before time zero
- **WHEN** samples were acquired before the shared zero point
- **THEN** they are dropped and counted, not written with negative timestamps

### Requirement: Monotonic, synchronised timestamps
IMU timestamps SHALL be strictly increasing, SHALL share the time base of the
LiDAR point timestamps without any additional offset, and `timestampUnix` SHALL
equal `timestamp` plus one constant offset.

#### Scenario: Strict ordering
- **WHEN** any recording is written
- **THEN** no two IMU rows have equal timestamps and none decreases
- **AND** `timestampUnix − timestamp` is the same for every row

---

## specs/recording-output/spec.md

## ADDED Requirements

### Requirement: Atomic IMU CSV rows
The recorder SHALL never leave a partial row in the IMU CSV after any termination.

#### Scenario: Crash during a write
- **WHEN** the process receives a fatal signal while a row is half written
- **THEN** the CSV is truncated to its last complete row before the process dies

#### Scenario: Kill or power loss
- **WHEN** the process is killed or loses power mid-write
- **THEN** the next start removes any partial last row

#### Scenario: Failed write
- **WHEN** a write fails (e.g. disk full)
- **THEN** the CSV is cut back to its last complete row and the loss is logged

### Requirement: Unchanged CSV format
The IMU CSV SHALL keep the header `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix`,
space separators, 17-decimal fixed floats and integer nanosecond timestamps.

#### Scenario: Byte-identical rows
- **WHEN** a row is written
- **THEN** it is byte-identical to the previous `std::fixed << setprecision(17)` formatting

### Requirement: Point cloud finalized as LAZ chunks
The recorder SHALL convert `points_temp.bin` (32-byte `PointDLidar` records) into
`lidar0001.laz`, `lidar0002.laz`, … (LAS 1.2, point format 1) with at most
`kMaxPointsPerLaz` points each, and SHALL delete the temp file only after all
chunks are complete.

#### Scenario: Partial trailing point
- **WHEN** the temp file ends with a partial record
- **THEN** the partial record is ignored and all complete points are written

#### Scenario: Interrupted finalization
- **WHEN** finalization is interrupted
- **THEN** no incomplete `lidar*.laz` exists (chunks are written as `.tmp` and renamed) and the temp file remains for the next start

#### Scenario: Stale chunks
- **WHEN** the output directory contains higher-numbered `lidar*.laz` from an earlier recording
- **THEN** they are removed so the recording's chunks are not mixed with old ones

---

## specs/capture-lifecycle/spec.md

## ADDED Requirements

### Requirement: Graceful stop on signals
The recorder SHALL stop capturing on SIGINT, SIGTERM or SIGHUP, write all
remaining IMU and point data and finalize the LAZ files; a second signal SHALL
terminate immediately.

#### Scenario: Ctrl-C
- **WHEN** the user presses Ctrl-C during capture
- **THEN** the recording ends with a complete `imu.csv`, `lidar*.laz` and no `points_temp.bin`

### Requirement: Recovery on next start
On start, the recorder SHALL finalize a recording left unfinished by a previous
run into a `recovered_<time>/` subdirectory before starting a new recording, and
SHALL offer `--finalize` to finalize it in place without capturing.

#### Scenario: Leftover temp file
- **WHEN** `points_temp.bin` exists at start
- **THEN** the CSV is trimmed, the points are converted to `lidar*.laz`, and both are moved to `recovered_<time>/`

#### Scenario: Finalize only
- **WHEN** the recorder is started with `--finalize`
- **THEN** the leftover recording is finalized in the output directory and the program exits without connecting to the LiDAR

### Requirement: Clean capture start
The recorder SHALL discard packets buffered before the capture starts.

#### Scenario: Backlog before capture
- **WHEN** packets accumulated during LiDAR restart and status queries
- **THEN** they are read and discarded for 2 s before recording begins

### Requirement: No data lost at stop
The recorder SHALL write points and IMU samples still buffered when the capture stops.

#### Scenario: Partial last chunk
- **WHEN** the capture stops with fewer than 14 000 points collected for the current chunk
- **THEN** those points and the held IMU samples are written as a final chunk

---

## Tasks

## 1. IMU timestamping
- [x] 1.1 Rewrite `ImuTimestamper` with the lower-edge clock model (hull fit, delayed envelope, slew limits)
- [x] 1.2 Runtime detection of `seq` as IMU sample counter; drop detection without `seq`
- [x] 1.3 Converged startup, backwards-computed held samples, drop `t < 0`
- [x] 1.4 Extended end-of-capture statistics
- [x] 1.5 Unit tests: bursts, random latency, startup dump (absolute and zero-at-first-arrival), drops (with/without seq), 60 ms stall, combined scenarios; 3 seeds each

## 2. Recording output
- [x] 2.1 Whole-row CSV writes via POSIX fd with cut-back on failure
- [x] 2.2 Fatal-signal handler truncating the CSV (`include/shutdown.h`)
- [x] 2.3 `FinalizePointsTemp` into `lidarNNNN.laz` chunks via `.tmp` + rename
- [x] 2.4 `TrimPartialCsvLine`, `RecoverInterruptedRecording`, stale chunk removal
- [x] 2.5 Tests: row format unchanged, SIGKILL mid-write + recovery, SIGABRT truncation, SIGINT finalization, leftover temp → LAZ chunks (read back with laszip)

## 3. Capture lifecycle
- [x] 3.1 SIGINT/SIGTERM/SIGHUP graceful stop
- [x] 3.2 Recovery at start into `recovered_<time>/`; `--finalize` flag
- [x] 3.3 Drain packet backlog instead of `sleep(2)`
- [x] 3.4 Write remaining points in the final chunk

## 4. Documentation and build
- [x] 4.1 CTest targets `imu_timestamper`, `datawriter`
- [x] 4.2 Update `CLAUDE.md` and `docs/unitree_l2_slam_tuning.md`

## 5. Verification on hardware (open)
- [ ] 5.1 Check the summary line: is `seq` usable on the L2?
- [ ] 5.2 Walking recordings: SLAM results vs offline-repaired timestamps
- [ ] 5.3 Confirm the SLAM loads multiple `lidar*.laz` chunks in order
- [ ] 5.4 Optionally evaluate device timestamps (`kUseSystemTimestamp = false`)
