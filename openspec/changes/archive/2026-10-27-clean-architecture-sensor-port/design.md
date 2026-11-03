# Design

## Context

See proposal.md (Why) and specs/ for the requirements. This section lists the facts from the code that constrain the design.

**Current structure**
- Header-only code in `include/`. Vendor coupling per file was measured with a coupling script:
  - Vendor types reach `capture_timing.h`, `pcd_manager.h`, `datawriter.h` (only for `PointDLidar`), `packet_state.h`, all tests except `test_imu_timestamper`, and `replay_capture`.
  - The globals `GetGlobalTimeOffsetNs`, `GetSensorOrientation` and `StopRequested` appear in 7 files.
  - Already neutral: `imu_timestamper.h`, `time_alignment.h`, `packet_replay.h`, `shutdown.h`.

**Floating point**
- `bin/replay_capture` (-O2) has 51 fused multiply-add instructions; `bin/dLidar` (-O0) has none in our code.
  - GCC 12 on aarch64 contracts `a*b+c` when optimizing, including `packet_base_time + j*time_step` (the LAZ gps_time) and doubles in `ImuTimeline`.
  - As a result, today's golden replay does not reproduce production arithmetic.
  - Moving code between translation units or targets with different flags would change bits, and could even flip the auto-mode decision.

**Byte-identity details the restructure must preserve**
- *Time zero:* the stamp of the first point packet with `point_num > 0`.
  - It is set before that packet is converted, before filtering, with a `== 0` "unset" sentinel.
  - It is read when a chunk is collected, so it is 0 before any point packet. `packetstate` then prints absolute stamps.
- *Order within a point packet:* packet-log record, then state row (also for `point_num == 0`, before the early return), then conversion, then `ImuTimeline::PushPointPacket`, which may emit IMU into the same chunk. `ChunkReady` is checked after every packet.
- *Mount:* VERTICAL is a pure select-and-negate (`z = -x`, `-a[0] / 9.80665f`).
  - A matrix multiply would turn −0.0 into +0.0 and change the `%.17f` CSV text and NaN signs.
  - LAZ coordinates are quantized, so LAZ is not affected.
- *Filter:* the filter compares `range_float` (stored bit-exact as `point.range`) and the uint8 intensity against float thresholds. It runs after the packet validity checks. The `alpha_cur += step` accumulation runs over every sample, filtered or not.
- *Trig on floats:* `sin`/`cos` on floats resolve to `sinf`/`cosf` via `<math.h>`. With only `<cmath>`, unqualified calls silently become double.
- *Packet log:* `packetsNNNN.bin` is a memcpy of the 40-byte `PacketTiming`, with L2 fields (`time_increment`, `point_num`, type codes 102/104).

**Startup and shutdown**
- The consumer thread opens the writer in parallel with the L2 start-up (restart rotation, dirtiness and time-delay prints, version queries, 2 s drain). A stop during start-up therefore still produces a complete, empty session.
- `replay_capture` has its own single-threaded loop around `ChunkAssembler` with `target_chunks = 800`; the live code uses 200.

**Future direction**
- The device will gain buttons, LEDs and a Bluetooth link to a phone, and possibly a different LiDAR.

## Goals / Non-Goals

**Goals:**
- Dependency rule enforced at compile time (include paths per target) plus an architecture check in ctest.
- Seams for sensors (driven/output side) and for device I/O channels (driving input side and status output side), each proven by a fake in the tests.
- Byte-identical recordings and console messages at every migration step.
- No process-wide mutable state in the core.

**Non-Goals:**
- Implementing GPIO, LEDs, Bluetooth, a daemon mode or a second sensor.
- A config file or runtime sensor selection. Constants and endpoints move to `app/` unchanged.
- Recording format changes (a sensor-defined extra-bytes schema is format 3, later).
- Moving `ImuTimeline::Report()` and `AddRecordingInfo` JSON building into a serializer. It is presentation logic in the wrong layer, but moving it risks byte drift. Deferred and noted in `docs/architecture.md`.
- Moving the timestamp-order warnings out of the writer. Deferred for the same reason.
- Build-type changes (`-O0` stays) and performance work.
- Fixing the unbounded queue when `Open` fails (a separate bug-fix change).

## Decisions

### D1 Layers, targets and layout

```
src/domain/                       entities and pure rules (no I/O)
  recording/  point_record.h (48-byte format-2 record, offsetof asserts), imu_sample.h,
              packet_timing.h (40-byte log record, offsetof asserts, type codes),
              sensor_chunk.h, recording_info.h, format.h (kRecordingFormatVersion)
  timing/     imu_timestamper.h, time_alignment.h (LowerEdge, ImuTimeline)   (moved as is)
  geometry/   axis_map.h (mount), point_filter.h
src/application/                  use cases and ports
  ports/      lidar_sensor.h, recording_storage.h, device_events.h, stop_token.h
  capture/    capture_timing.h, chunk_assembler.h, record_session.h (was ProcessSensorData), lens_monitor.h
  device/     scanner_controller.h (state machine + command interface), event_hub.h
  recovery/   recover_recordings.h
src/adapters/
  unitree_l2/ unitree_l2_sensor.{h,cpp}, l2_conversion.{h,cpp}, l2_packet_state.{h,cpp},
              l2_packet_source.h (live reader | synthetic), vendor/ (SDK headers, reduced utilities.h)
  storage/    recording_writer.{h,cpp} (was ContinuousDataWriter), points_temp, laz_encoder, session_files,
              recovery_io, crash_guard (fd truncation used by the signal handler)
  console/    console_events.{h,cpp} (today's console text)
src/app/      main.cpp (composition root, CLI), signals.cpp, device_config.h (endpoints, mount, constants)
tools/        replay_capture.cpp (UnitreeL2Sensor over the synthetic source + RecordSession), replay_packets.cpp
```

Build targets:

| Target | Contents | May use |
|---|---|---|
| `dlidar_core` | INTERFACE or STATIC: domain + application | `src/` only; not `vendor/` or `3rd/` |
| `dlidar_l2` | L2 adapter | vendor `.a`, pthread |
| `dlidar_storage` | storage adapter | LASzip |
| `console` | console output | built into `dLidar` with `app/` |

Future adapters (`gpio/`, `ble/`) become their own targets.

**Splitting into `.h`/`.cpp`:** only in adapters, where a `.cpp` hides a dependency (vendor, LASzip). The core stays mostly header-only. With D2, the split no longer changes the arithmetic.

*Alternatives:*
- One target per layer. Rejected: too much ceremony for about 5k lines, and the include-path rule on `dlidar_core` already enforces the important boundary.
- Plain directories without targets. Rejected: nothing would be enforced.

### D2 `-ffp-contract=off` everywhere, as commit 0

Set it in `CMakeLists.txt` and in `golden_replay.sh`, and rebuild the base revision's replay with the same flag. At `-O0` there is no contraction anyway, so production `bin/dLidar` output does not change; only the `-O2` tools change, and they then match production. After that, IEEE results do not depend on optimization level or on how code is split into translation units on aarch64, which is what makes the restructure verifiable.

*Alternative:* hunt FMA differences function by function. Rejected as fragile.

### D3 LiDAR sensor port (driven side)

```cpp
namespace dlidar {
struct HostTime   { int64_t mono_ns, unix_ns; };
struct ImuReading { float accel_mps2[3], gyro_rads[3]; int64_t stamp_ns; uint32_t seq; };   // sensor frame
struct PointPacket{ int64_t first_ns, duration_ns; uint32_t seq, count; PacketTiming log; };  // log = packet-log record
enum class PacketKind { None, Imu, Points };
struct Polled     { PacketKind kind; HostTime host; };
struct SensorInfo { RecordingInfo versions; PointFilter default_filter; /* later: imu_clock, imu_rate */ };

class LidarSensor {
public:
  virtual ~LidarSensor() = default;
  virtual bool Prepare(const StopToken &) = 0;            // L2: restart, status prints, versions, 2 s drain
  virtual SensorInfo Info() = 0;
  virtual Polled Poll() = 0;                               // next packet; host time stamped by the adapter
  virtual void TakeImu(std::vector<ImuReading> &out) = 0;  // after Poll()==Imu; one or more samples
  virtual const PointPacket &Points() const = 0;           // after Poll()==Points
  virtual void ConvertPoints(int64_t zero_ns, const PointFilter &, std::vector<PointRecord> &out) = 0;
  virtual std::optional<float> LensDirtiness() { return {}; }   // cached, non-blocking
  virtual PacketDiagnostics *Diagnostics() { return nullptr; }  // header + TakeRows(zero_ns, out)
  virtual void Stop() = 0;
};
}
```

- **Pull, not callbacks.** The core keeps control of the loop (stop, chunk target, dirtiness interval) as today, and the replay stays deterministic. A future callback-based SDK (Livox, Ouster) queues inside its adapter.
- **Host time comes back with the packet.**
  - The live L2 adapter stamps `CLOCK_MONOTONIC`/`CLOCK_REALTIME` right after `runParse()` returns, exactly as today.
  - The replay source returns the logged times.
  - The core therefore needs no Clock port.
- **Generic packet header plus a frozen log record.**
  - The core uses `first_ns`, `duration_ns`, `seq` and `count`. The L2 computes `duration_ns` with today's `PacketDurationNs`.
  - The L2-shaped fields (`time_increment`, `point_num`, type codes) go only into the `PacketTiming log` record, whose format is frozen.
- **Two-phase points.** `Poll()` reports a point packet; `ConvertPoints(zero, filter)` is called only for `count > 0`. This lets the core own the zero (D4) while the adapter keeps today's double arithmetic for point time (`packet_base_time + j*time_step`) and its sequential angle accumulation.
- **The filter is passed in.** The adapter may reject points early, before trig, to save CPU on the Zero 2 W. The core applies the same predicate again (idempotent), so the policy holds for every sensor.
- **One virtual call per packet**, never per point.

*Alternatives:*
- Templates/CRTP. Rejected: they leak adapter types into core headers.
- Core-owned absolute point times. Rejected: they change the LAZ doubles.

### D4 Core owns the time zero, the mount, units and the filter

- **Time zero:** `CaptureTiming` keeps the zero with today's exact rule (first `count > 0` packet, `== 0` sentinel, set before conversion). It passes the zero to `ConvertPoints`, to `ImuTimeline::SetZero`, and to `Diagnostics()->TakeRows` when a chunk is collected. This removes `GetGlobalTimeOffsetNs`.
- **Mount:** `AxisMap` is a signed axis permutation (output axis i = sign_i · sensor axis src_i). It is validated as a proper rotation at start-up, so mirrors are rejected.
  - It is applied by select-and-negate only (no multiply), to points and to IMU accel/gyro in one place.
  - Example: `device_config.h` has `kMount = {front: +Z, left: +Y, up: −X}`.
  - This removes `GetSensorOrientation`.
  - The mount belongs to the installation (the handheld), not to the sensor, which is why it lives in the core.
  - The user's rotatable mount fits this as long as it stays a 90° permutation. Small calibration angles would need a real rotation and a format decision, so they come later.
- **Units:** `accel_g = accel_mps2 / 9.80665f`, a float divide in the core. Negation commutes exactly with it, so applying the mount before or after does not matter.
- **Filter:** `PointFilter{min_range_m, max_range_m, min_intensity}` with float thresholds, compared against the sensor's `range` (not ‖xyz‖, which includes the axis offsets) and its intensity. The values come from `SensorInfo::default_filter` (L2: 0.1 m, 100 m, 5) unless the app overrides them.

### D5 Recording storage port

One port, `RecordingStorage`, with `Open()`, `Append(SensorChunk)`, `Close()` and `RecoverLeftovers(move_aside)`. The adapter (`RecordingWriter` plus the recovery I/O) keeps every file detail.

- The packet-state header comes from the sensor's `Diagnostics()` and is given to the storage at construction, so recovery's partial-line trim writes the same header as today.
- No diagnostics means no `packetstate` file.
- `RecordSession` keeps today's producer/consumer queue and consumer thread, including opening the storage in parallel with `Prepare`.

*Alternative:* separate sink and store ports. Rejected for this size; it can be split later if a second backend appears.

### D6 Device control (driving side) and status output

- **`ScannerController`** (application) owns the state machine Idle → Preparing → Recording → Finalizing → Idle | Error.
  - Its thread-safe command interface is `StartRecording()`, `StopRecording()`, `Status()` and `WaitIdle()`.
  - Each recording runs `RecordSession` on a worker thread with a `StopToken`, which is set by `StopRecording()` or by the process stop flag.
  - A start command outside Idle is rejected with the current state.
- **Signals:** the handler stays async-signal-safe and only sets the atomic flag, as now. The second-signal kill and crash truncation are unchanged. The capture loop sees the flag through the `StopToken`.
- **`DeviceEvents` output port:** `OnState(DeviceStatus)`, `OnWarning(kind, text)`, `OnProgress(chunks, points)` (per chunk, not per packet) and `OnSummary(RecordingSummary)`. `EventHub` fans out to the registered channels.
  - Isolation for external channels (LED, BLE): each runs behind an async wrapper with a bounded queue (drop oldest), its own thread and exception catching, so it cannot stall or break a recording.
  - The console channel is registered synchronously, so its lines keep today's order relative to adapter prints.
- **Console channel:** prints exactly today's texts: the start banner, the dirty-lens warning, `PrintTimingSummary`, "Capture complete…". `LensMonitor` (core) keeps the 10 s interval and 5% threshold, using the host times of the polled packets (no static).
- **CLI one-shot mode (`app/main.cpp`):** install signals, parse flags, recover leftovers, build the sensor/storage/console, then `StartRecording()`, `WaitIdle()`, `sensor.Stop()` and exit with today's codes. A future `--daemon` would only skip the auto-start and leave buttons/BLE to send commands.

*Alternatives:*
- Keep `ProcessSensorData` as a blocking call and add channels around it. Rejected: buttons/BLE need start/stop/status while the app is idle and while it records.
- A general message bus. Rejected as more than needed.

### D7 Replay through the production path

`replay_capture` builds `UnitreeL2Sensor` over `SyntheticL2Source`, which holds the packet synthesis code from today's tool and the logged host times. It runs it through `ScannerController`/`RecordSession` with `target_chunks` as a parameter (800 for the golden replay) and a no-op `Prepare`. The golden replay therefore covers conversion, the use case, threading and storage.

A separate `FakeSensor` (domain events directly, no L2 code) in the core tests proves the sensor-independence requirement.

### D8 Verification tooling

- **`tools/golden_replay.sh BASE_REV [PACKETS] [SECONDS]`**
  1. `git worktree add` the base revision and build it with `-ffp-contract=off`.
  2. Run its replay and the working tree's replay on the same log.
  3. `cmp` every file, and diff the console logs.
  - It must run on the Pi (aarch64), since x86_64 without `-march` never contracts.
  - Base = commit 0 (D2).
- **`tools/check_architecture.py`** (ctest `architecture`): parses `#include`s under `src/`.
  - `domain` → std + domain only.
  - `application` → std + domain + application only.
  - Adapters never include `app/`, and no adapter includes another, except `storage` → `crash_guard`.
- **Unit tests** for `AxisMap` (±0.0, NaN, all 24 proper rotations; mirrors rejected), the float `sin` overload in the conversion, `/9.80665f`, and the zero rule and in-packet ordering (in `CaptureTiming` tests with `FakeSensor`).

### D9 Naming and namespaces

All dLidar code goes into `namespace dlidar`, with no `using namespace` in headers. Vendor code stays in `unitree_lidar_sdk`, visible only under `adapters/unitree_l2`. The vendor `unitree_lidar_utilities.h` is reduced to the vendor's own content (types the prebuilt `.a` uses: `PointUnitree`, `PointCloudUnitree`, crc32, …); dLidar's additions move to `l2_conversion`.

## Risks / Trade-offs

- **[Base rev differs once `-ffp-contract=off` is applied.]** → Commit 0 is the flag alone. Golden = commit 0 against later commits. Production (-O0) output is unaffected by construction.
- **[The prebuilt `.a` depends on types in the vendor utilities header.]** → Only remove dLidar additions. `test_sdk_conversion` (links the `.a`, loopback UDP) must stay green.
- **[The port is shaped by the L2.]** → It is generic where cheap now (host time from the adapter, first/duration stamps, multi-sample IMU, common frame). The rest is deferred to the second-sensor change: IMU-clock capability, multi-return, ring meaning, the format-3 extra-bytes schema.
- **[The format-2 point record carries L2 semantics]** (`alpha`/`theta`, `raw_range` as uint16 mm, which overflows past 65 m) → Documented as the L2 format. Another sensor fills zeros or needs format 3.
- **[The controller's worker thread changes thread timing.]** → The output does not depend on timing (the queue keeps order). The console channel is synchronous. The golden check plus one live capture verify it.
- **[Async channels drop events under load.]** → Acceptable for LEDs and BLE (status is idempotent). The summary is re-sent as the final state.
- **[Large diff while `improve_timestamp` has uncommitted work]** (`target_chunks` 800 → 200 in `pcd_manager.h`). → Commit or stash it before commit 0. Do one step per commit.

## Migration Plan

One commit per step. After each: build, `ctest`, `tools/golden_replay.sh <commit0>` identical.

Golden base (commit 0): `36ecead`.

0. Pending work committed; add `-ffp-contract=off`; add the `offsetof` asserts.
1. Golden harness and architecture check (the check is permissive at first, and tightened as layers appear).
2. Domain types into `src/domain` + `namespace dlidar` (aliases keep the old names compiling).
3. `AxisMap`, `PointFilter`, and core-owned zero in `CaptureTiming`. The L2 conversion is still in place but stops using the globals.
4. Sensor port and L2 adapter (live and synthetic sources); `replay_capture` uses it.
5. Storage port and adapter.
6. `RecordSession`, `ScannerController`, `EventHub`, console channel, `app/main.cpp`, signals.
7. Targets and include paths enforced; vendor header reduced; tests regrouped; architecture check strict.
8. Docs: `docs/architecture.md`, `CLAUDE.md`, file references in `docs/*.md`, `compare_recording_formats.sh`.

Finally, one live capture on the device (and on the Zero 2 W), checking the endpoints, start-up queries, Ctrl-C finalization and dMapper import.

Rollback: revert the commits. No data migration is involved.
