# Design

## Context

See proposal.md (Why) and specs/ for the requirements. These facts from the code and the vendor material shape the design.

**The core is already sensor-neutral; a few L2 traces remain**
- `CaptureTiming` and `ImuTimeline` (`src/application/capture/capture_timing.h`, `src/domain/timing/time_alignment.h`) contain no IMU rate constant: the period is measured. The auto-mode limits fit 200 Hz (5 ms):
  - a spacing histogram up to 20 ms;
  - a stamp jump at 50 ms;
  - seq deltas of 1–10, which must count IMU samples.
- L2 traces:
  - The packet type codes 102/104 (`packet_timing.h`) are matched by `packet_timing_replay.h`.
  - The timing JSON keys `zero_l2_ns`, `imu_period_ns_l2` and `map_l2_per_host_ppm`.
  - The L2 texts in `kLazExtraAttributes` (`laz_encoder.cpp`).
  - `FileRecordingStorage(dir, packet_state_header)`, built in `main.cpp` with `l2::kPacketStateHeader`.
  - `replay_capture.cpp` (L2 only).
- The time zero uses `0` as its "unset" value (`capture_timing.h`). The Mid-360S stamps since power-on are never 0 at the first point packet.
- `points_tempNNNN.bin` has a 16-byte header: magic `dLptsv2\n`, the record size, and 4 reserved zero bytes (`recording_files.h`). The packetstate header is passed to recovery only to rewrite a file that trimming emptied (`recovery.cpp`).
- No settings parser or JSON/TOML library exists in the repo. `3rd/` is the LASzip submodule itself.
- `tools/check_architecture.py` maps any `src/adapters/<name>/…` to an adapter, so a new adapter directory is accepted. Adapters must not include each other.

**Livox Mid-360S and SDK2** (Livox wiki protocol page, `livox_lidar_def.h`, the Mid-360S user manual; see `docs/livox_mid360s.md` once written)
- UDP packets (`LivoxLidarEthernetPacket`, packed, 36-byte header):
  - `time_interval` is in 0.1 µs, from the first to the last point.
  - `dot_num` is 96 for point packets. `udp_cnt` (u16) resets every frame, and `frame_cnt` is u8.
  - `time_type`: 0 = since power-on, 1 = PTP, 2 = GPS.
  - `timestamp` is u64 ns.
- Data types:
  - 0: IMU, six floats, gyro in rad/s and accel in g.
  - 1: Cartesian high, `int32` mm xyz, reflectivity u8, tag u8.
  - 2, 3: low-resolution and spherical.
- The IMU and the points share one frame and one clock. The IMU sits at (+11.0, +23.29, −44.12) mm in the point frame. Neither the firmware nor the SDK applies extrinsics.
- SDK2 behaviour:
  - It calls the point and IMU callbacks on its single data IO thread. Command replies arrive on its command thread, and timeouts call back with a null response.
  - `LivoxLidarSdkInit(nullptr, host_ip)` crashes (issue #135). Commands time out on a Mid-360S while data flows (issue #125, open).
  - Mid-360S (dev_type 35) support is on master past tag v1.3.1.
  - It builds with CMake ≥ 3.0 and vendored rapidjson/spdlog/FastCRC. GCC ≥ 13 fails on missing `<cstdint>`; the Pi's GCC 12.2 is unaffected.

## Goals / Non-Goals

**Goals:**
- L2 recordings unchanged by every step except the format 3 additions, each step proven by `tools/golden_replay.sh` or `tools/compare_recording_formats.sh`.
- The Mid-360S adapter fully testable without the sensor, so bring-up only confirms frame, timing and rate assumptions.
- The core untouched: no new port methods beyond the identity and profile data.

**Non-Goals:**
- Running both sensors at once, or switching sensors at runtime without a restart.
- Using the Livox HMS, lens or temperature status, PTP sync or IMU rate settings (see proposal, Out of scope).
- Deskewing, voxelizing or any SLAM-side processing for the wider FoV.

## Decisions

**D1. A TOML subset with our own parser, in `src/app/settings.{h,cpp}`.**
- About 200 lines: line-based, `[section]`, `key = value`, with value types string, integer, float and bool, and `#` comments.
- The schema is a table of known keys per section, each with a type, a range and a target field in `DeviceSettings`. Validation and error messages come from that one table. Unknown keys are errors, so a typo cannot silently fall back to a default.
- Alternatives:
  - toml++: a heavy header, slower Pi builds.
  - nlohmann/json: no comments.
  - INI: no common standard, so no tool can check the file.
- The subset is valid TOML, so the file can be checked with any TOML tool.

**D2. The settings live in the composition root, not the core.**
- `DeviceSettings` (`src/app`) is mapped to the existing `SessionConfig` and to `LiveL2Config`/`LiveLivoxConfig`. The core keeps receiving plain config structs, so no core type learns about files (dependency inversion).
- `device_config.h` keeps only the defaults, which are today's constants.
- The settings are loaded and validated first in `main`, before recovery, so a bad file writes nothing.

**D3. `src/app/sensor_factory.{h,cpp}` is the only code that names both adapters.**
- `SensorKind ParseSensorKind(text)`.
- `RecordingProfile ProfileFor(SensorKind)`.
- `std::vector<RecordingProfile> AllProfiles()`, used for recovery.
- `std::unique_ptr<LidarSensor> OpenSensor(const DeviceSettings&)`.
- `main` order: settings → storage (current profile and all profiles) → recovery → `OpenSensor` → controller. That is today's order, with the sensor behind the factory.
- Adding a third sensor means adding an adapter and a factory case (open/closed in the composition root).

**D4. `RecordingProfile` is a domain value (`src/domain/recording/recording_profile.h`). The storage takes it in place of the bare header.**
- Fields: `sensor_id` (u32: 1 = L2, 2 = Mid-360S), `sensor_model` (`unitree_l2`), `las_system_identifier` (≤ 31 chars, `Unitree L2`), six extra-byte descriptions (≤ 31 chars each, the LAS limit), and `packet_state_header` (empty = none).
- Each adapter exports its profile as a constant next to its packet-state header: `l2::kRecordingProfile`, `livox::kRecordingProfile`.
- The storage adapter never includes a sensor adapter; the app hands profiles in.
- Alternative: put the profile in `SensorInfo` only. Rejected because recovery runs before any sensor is opened and must also handle leftovers from the other sensor.

**D5. The points temp header records the sensor; recovery picks the matching profile.**
- Format 3 temp files use magic `dLptsv3\n`, with the reserved u32 set to `sensor_id`.
- `DetectPointsTempLayout` returns the sensor id: 1 for `dLptsv2`, and 1 for header-less format 1 files.
- Recovery looks the id up in the profiles the app passed. An unknown id is an error with the file name; the temp file is kept.
- The packetstate header for an emptied file comes from that same profile, which removes today's "wrong header after switching" edge case.
- Records stay 48-byte `PointRecord`.

**D6. Format 3 changes for the L2: keys and header text only.**
- `RecordingInfo` gains `sensor_model` and `std::optional<std::array<double,3>> imu_position_m`.
- `AddRecordingInfo` writes `sensor_model` right after `recording_format_version`, and writes `imu_position_m` only when set.
- The LAZ writer sets `system_identifier` from the profile. `generating_software` stays `dLidar recording format %d` and so reads 3.
- The existing `_l2` JSON keys keep their names, documented as "the sensor's clock", so tools that read format 2 keep working.
- The IMU position is the sensor's value (sensor frame) passed through the mount in `RecordSession`/`CaptureTiming` setup, by select-and-negate, so it is exact.

**D7. Mid-360S adapter structure mirrors the L2's** (single responsibility per file).
- `livox_packet_source.h`: `LivoxPacketSource::Next(HostTime&)` returns `kPoints`, `kImu`, `0` or `kEndOfStream`. `Packet()` returns the bytes of the current packet, a `const LivoxLidarEthernetPacket&` plus its length. `Prepare`, `Versions` and `Stop` work as for the L2.
- `livox_conversion.{h,cpp}`: `ParsePoints` and `ParseImu`, header-only knowledge of the packed structs.
  - **Point time:** `t_i = timestamp + i·time_interval·100/(dot_num−1)` ns, computed in int64 and then converted to seconds relative to the zero as `(double)(t_i − zero) * 1e-9`. That is the same expression shape as the L2's, with no FMA (`-ffp-contract=off`).
  - **xyz:** `(float)(mm * 0.001)` in double.
  - **range:** `sqrt` in double, then float.
  - **raw_range:** `min(llround(range_mm), 65535)`.
  - **alpha/theta:** `atan2(y,x)` and `atan2(z, hypot(x,y))`.
  - **ring** = 1, **sample_index** = i, **packet_seq** = `frame_cnt<<16 | udp_cnt`.
  - A packet whose `length` disagrees with `dot_num` × point size is skipped and counted.
  - **IMU seq:** unwrapping the u16 `udp_cnt` to u32. The unwrap is done by the sensor class, which owns the state.
  - `ParseImu` multiplies accel by 9.80665f because the port is SI (Liskov substitution: the same units as every sensor). The core divides back; the round trip may differ in the last float bit, which is irrelevant at the sensor's noise level and unaffected for the L2.
  - Packets are counted and skipped when their data type is other than 1 (points) or 0 (IMU), and point packets are counted and skipped when the length is wrong.
- `livox_packet_state.h`: one TSV row per point packet: `stamp_ns time_type data_type frame_cnt udp_cnt dot_num time_interval_100ns low_confidence skipped`.
- `livox_mid360s_sensor.{h,cpp}`: `LivoxMid360sSensor : LidarSensor, private PacketDiagnostics`, a counterpart of `UnitreeL2Sensor`.
  - `PacketTiming` record: `sensor_ns` = timestamp, `time_increment` = step in s (float), `point_num` = `dot_num`, types `kPacketTypePoints`/`kPacketTypeImu`.
  - `drop_low_confidence` drops points whose tag bits 0–1 or 2–3 equal 2.
- `synthetic_livox_source.{h,cpp}`: builds real packed packets from a model (rates, clock error and offset, batching, loss) or from a `packetsNNNN.bin` replay.
- `live_livox_source.{h,cpp}` (D8): the only file that includes `livox_lidar_api.h`.

**D8. Callback-to-pull bridge: a growable pool of fixed slots.**
- Each slot is 1,536 bytes (a Mid-360 packet is about 1,380 B) and holds the `HostTime` and a copy of the packet.
- The pool starts with 8,192 slots (12 MB, about 4 s at 2,083 pkt/s). The SDK callback takes the arrival time first, locks a mutex, takes a free slot (allocating a new block of 1,024 if none is free, with a one-time warning), memcpys and notifies.
- `Next()` waits up to 100 ms, pops the oldest slot into the "current" slot, and returns the previous current slot to the free list.
- The callback never waits on the consumer and never drops. Unbounded growth is the same policy as the chunk queue: dropping data would break the recording.
- Alternatives:
  - A lock-free SPSC ring: fixed size, so it would drop or block.
  - A `std::deque<std::vector<uint8_t>>`: one allocation per packet at 2 kHz.

**D9. Start-up is best effort for commands (SDK issue #125).** `LiveLivoxSource::Prepare` takes these steps:
1. Write the SDK config JSON (key `Mid360s`, from the settings) to `$XDG_RUNTIME_DIR/dlidar_livox.json`, or `/tmp` if that is unset. Init with that path and `host_ip`; never pass a null path (#135).
2. Wait up to 10 s for the info-change callback. Accept dev_type 35, and 9 (Mid-360) with a warning.
3. Send work mode Normal, enable IMU, set point data type Cartesian high, and query the firmware. Each is waited on for 2 s; a timeout gives one console warning and the start continues.
4. Wait for point packets of data type 1: 10 s without any packet, or 60 s without points, fails. The same limits as `LiveL2Source`.
5. Drain the backlog for 2 s.

`Stop()` sends work mode Sleep (best effort) and calls `LivoxLidarSdkUninit`. The SDK is a process-wide singleton: the source class allows one instance and `Open()` refuses a second one.

**D10. The SDK as a submodule, built with only its static library.**
- `external/Livox-SDK2` is pinned to a master commit with Mid-360S support, recorded in `docs/livox_mid360s.md`.
- CMake: `add_subdirectory(external/Livox-SDK2/sdk_core EXCLUDE_FROM_ALL)` if it configures stand-alone. Otherwise the root is added with `EXCLUDE_FROM_ALL`, so the samples are never built.
- Warnings from SDK sources are not our concern: `-w` is set on its target only.
- `dlidar_livox` links `livox_lidar_sdk_static` privately; only `live_livox_source.cpp` sees its headers.
- Alternative: a prebuilt library like Unitree's. Rejected by the user in favour of a source-pinned build.

**D11. Recording length stays chunk-based but settable** (`[device] target_chunks`, default 1500). The Mid-360S reaches 1500 chunks of 14,000 points in about 105 s; the manual says so. A seconds-based limit is deferred.

## Risks / Trade-offs

- **SDK commands time out (#125)** → D9 makes them optional. The firmware defaults are sampling with the IMU on. If the default point data type is not Cartesian high, the start fails with a clear message ("point packets of data type N; set it with Livox Viewer 2"), rather than recording nothing.
- **Frame or IMU-position sign wrong** → the bring-up checklist checks gravity and walls before the first real recording, and the IMU position is checked against the manual's figure. Both are settings or constants, not code changes.
- **`time_interval` semantics** (`/dot_num` in the ROS driver versus `/(dot_num−1)` on the wiki) → the conversion test pins our formula. Bring-up compares the stamp of packet k+1 with packet k's last point plus one step, using `packetstateNNNN.tsv`.
- **Data rate on a 1–2 GB Pi 5:** about 9.6 MB/s to the temp file and about 3× longer finalization → measure on the device's SD card. The pool (12 MB) and chunk queue fit in 1 GB. The backlog warning appears at 50 chunks (about 34 MB).
- **SDK threads and allocations on a 1 GB board** → the SDK's own buffers are small (MB). Check peak RSS at bring-up with `/usr/bin/time -v`.
- **Submodule pinned past the last tag** → the pinned hash is documented, and the SDK is updated deliberately. The adapter depends only on the stable packet structs and about 8 API calls.
- **Format 3 changes the L2 LAZ header and timing JSON** → `compare_recording_formats.sh` proves the standard fields, extra-byte values and IMU CSV are identical. dMapper reads neither the header identification nor the timing JSON.

## Migration Plan

1. Merging changes nothing for an existing L2 device without a settings file, apart from the format 3 additions.
2. `git submodule update --init --recursive` fetches the SDK.
3. To use the Mid-360S:
   - give the Pi an address in 192.168.1.0/24 (the L2 stays on 192.168.0.x);
   - write `~/.config/dlidar/dlidar.toml` with `sensor = "livox_mid360s"`;
   - run the bring-up checklist in `docs/livox_mid360s.md`.
4. Rollback: delete the settings file, or set `sensor = "unitree_l2"`. Older builds can still finalize format 1 and 2 temp files only; a format 3 leftover needs this build.

## Open Questions

- The default point data type and work mode of a new Mid-360S, and whether firmware V35.1.0112 fixes #125. These are answered at bring-up and only affect warnings.
- The physical mount axes. The default is `"+Z +Y -X"`, the same orientation as the L2, assuming the Mid-360S's −X side faces up. This is confirmed by the gravity check, and it is a settings value.
