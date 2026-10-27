# dLidar recording format

What a dLidar recording contains, field by field, for each supported sensor: the Unitree L2 and the Livox Mid-360S. Where the sensors differ, the tables have a column per sensor. The Mid-360S values are produced by its adapter (`src/adapters/livox_mid360s/`); see `docs/livox_mid360s.md`. It is written for the dMapper side (odometry, mapping and the planned intensity normalization). Source references are `file:line` in this repo.

**Effect on dMapper:** every format so far only adds data. The files dMapper reads (`lidarNNNN.laz`, `imuNNNN.csv`) keep their names, standard LAS fields, units, axes, time base and CSV text; format 3 adds only header identification and `timingNNNN.json` keys. A Mid-360S recording has the same files, units, axes and timestamp rules as an L2 recording.

**Recording format versions** (`kRecordingFormatVersion`, `src/domain/recording/recording_info.h`):

| Version | What changed |
|---|---|
| 1 | Everything before the LAZ extra bytes. It is not written anywhere: a file with no `recording_format_version` key is format 1. |
| 2 | LAZ extra bytes (`range`, `raw_range`, `sample_index`, `packet_seq`, `alpha`, `theta`), `packetstateNNNN.tsv`, the sensor and software versions in `timingNNNN.json`, the format in the LAZ `generating_software`, and a header on `points_tempNNNN.bin`. **Additions only:** every format 1 field keeps its value, type and position. |
| 3 | The sensor that made the recording: `sensor_model` (and, when known, `imu_position_m`) in `timingNNNN.json`, the LAZ `system_identifier`, extra-byte descriptions from the sensor (names, types and layout unchanged), and the sensor id in the `points_tempNNNN.bin` header. **Additions only:** for the same L2 packets, every LAZ point record (standard fields and extra bytes), the IMU CSV and the packet log are identical to format 2. |

Each sensor adapter describes its recordings with a `RecordingProfile` (`src/domain/recording/recording_profile.h`): sensor id, `sensor_model`, LAS system identifier, extra-byte descriptions and packetstate header. The L2's is `l2::kRecordingProfile` (`src/adapters/unitree_l2/l2_packet_state.h`), sensor id 1.

## Files of a recording

All files are in `/home/$USER/PointCloudDump/` and share the session number NNNN, the next free 4-digit number (`NextSessionIndex`, `src/adapters/storage/recording_files.h`).

| File | Since | Content |
|---|---|---|
| `lidarNNNN.laz` | 1 | Point cloud (see below). |
| `imuNNNN.csv` | 1 | IMU samples. **Unchanged in formats 2 and 3.** |
| `timingNNNN.json` | 1 | How the times were made, plus the recording keys (formats 2 and 3). |
| `packetsNNNN.bin` | 1 | Arrival time and sensor stamp of every packet (diagnostics, `bin/replay_packets`). Unchanged. |
| `packetstateNNNN.tsv` | 2 | One row per point packet with the sensor's own diagnostics columns (L2: state, calibration and angle parameters; Mid-360S: see below). |
| `points_tempNNNN.bin` | 1 | Exists only while recording; the crash-recovery source of the LAZ. |

dMapper reads every `.csv`, `.las` and `.laz` in the folder, so a recording has no other file with those extensions. A recording interrupted by a kill, crash or power loss is finalized on the next start into `recovered_<time>/`, with all its files.

## `lidarNNNN.laz`

LAS 1.2, point format 1, written through the LASzip DLL API (`WriteLazChunk`, `src/adapters/storage/laz_encoder.cpp`).

### Header

| Field | Value | Since |
|---|---|---|
| version / point format | 1.2 / 1 (`laz_encoder.cpp`) | 1 |
| point record length | 28 in format 1; **48** in format 2 (28 + 20 extra bytes) | 2 |
| scale x/y/z | 0.0001 (`laz_encoder.cpp`) | 1 |
| offset x/y/z | 0 (the LASzip default) | 1 |
| `number_of_point_records` (legacy 32-bit) | the point count | 1 |
| `number_of_points_by_return[0]` | the point count (every point has `return_number` 0, which doesn't match) | 1 |
| `global_encoding` | 1. This claims adjusted standard GPS time, but `gps_time` is not GPS time; see below. | 1 |
| `file_source_ID` | 4711 | 1 |
| `generating_software` | format 1: `LASzip DLL 3.4 r4 (231020)`; from format 2: **`dLidar recording format N`** with the format the recording was made in (`laz_encoder.cpp`) | 2 |
| `system_identifier` | format 3: the sensor's `RecordingProfile::las_system_identifier`, **`Unitree L2`** or **`Livox Mid-360S`**; empty before | 3 |
| VLRs | format 2: the standard Extra Bytes VLR (LASF_Spec, record 4), plus LASzip's own | 2 |

### Standard point fields (same in formats 1, 2 and 3)

| Field | Value | Source |
|---|---|---|
| X, Y, Z | The sensor frame remapped by the configured mount (`mount` in the settings, default `"+Z +Y -X"`: x = orig_z, y = orig_y, z = −orig_x; +X front, +Y left, +Z up). No other rotation and no lever arm. L2: its calibrated point model. Mid-360S: its Cartesian mm values / 1000. | `l2::ParsePoints`, Mid-360S `ParsePoints`, `CaptureTiming` |
| `intensity` | The raw 0–255 byte as sent (`intensities[j]`), not scaled. It goes byte → float → U16. | `l2::ParsePoints`, `laz_encoder.cpp` |
| `gps_time` | Seconds since the zero, which is the sensor stamp of the first point packet with points: `(stamp − zero)/1e9 + j·time_increment`. The packet stamp is the time of its first point. Mid-360S: the step is `time_interval·100 ns/(dot_num−1)`. The IMU `timestamp` column uses the same clock and zero, in ns. | `l2::ParsePoints`, Mid-360S `ParsePoints`, `laz_encoder.cpp` |
| `user_data` | Always 1 (the hard-coded `ring`). It carries no information. | `l2::ParsePoints`, `laz_encoder.cpp` |
| classification, point_source_ID, return_number, number_of_returns, scan_angle_rank, flags | Never set: always 0. | — |

Which points are written:
- **L2:** a point is dropped if `ranges[j]` is 0, if it is outside the packet's `range_min`/`range_max`, if the range is below 0.1 m or above 100 m, or if the intensity is below 5 (`l2::kDefaultFilter`, applied in `l2::ParsePoints` and again by `CaptureTiming`).
- **Mid-360S:** a point is dropped if its range is below 0.1 m (the blind zone) or above 100 m; no intensity floor. With `drop_low_confidence = true` also the points whose tag marks them low confidence (glued objects, rain/fog/dust).

### Extra bytes (from format 2)

These are stored in the record after the 28 standard bytes, in this order, and are described by the standard Extra Bytes VLR (`kLazExtraAttributes` and `PackLazExtraBytes`, `src/adapters/storage/laz_encoder.cpp`). Readers that don't use extra bytes see the format 1 fields unchanged. laspy 2.7 lists them by name and type (verified). Other readers of the standard VLR (PDAL, LAStools, CloudCompare) should do the same, but that was not tested.

| Name | Type | Byte | Unit | Meaning | Set at |
|---|---|---|---|---|---|
| `range` | float32 | 0 | m | The corrected range used for xyz: `range_scale * (ranges[j] + range_bias)`, as a float32 computed exactly as in the parser. | `l2::ParsePoints` |
| `raw_range` | uint16 | 4 | mm | `ranges[j]` as sent by the L2. | `:337` |
| `sample_index` | uint16 | 6 | — | `j`, the index of the point in its packet (0..`point_num`−1, 0..299 on the L2). Filtered points leave gaps, but the remaining points keep their own `j`. | `:338` |
| `packet_seq` | uint32 | 8 | — | The packet's `info.seq`. On the L2 it wraps at 1024 (measured). Use it with `packetstateNNNN.tsv`. | `:336` |
| `alpha` | float32 | 12 | rad | The alpha angle used for the point: `angle_min + alpha_angle_bias`, plus `angle_increment` once per sample, accumulated in float32 exactly as in the parser. | `:258, 334` |
| `theta` | float32 | 16 | rad | The theta angle used for the point: `com_horizontal_angle_start + theta_angle_bias`, plus `com_horizontal_angle_step` per sample, accumulated in float32 as in the parser. Not wrapped to [0, 2π). | `:260, 335` |

Their meaning on the **Mid-360S** (same names, types and bytes; descriptions from its `RecordingProfile`):

| Name | Unit | Meaning on the Mid-360S |
|---|---|---|
| `range` | m | Distance of the point from the sensor origin, `|xyz|` in the sensor frame, from the Cartesian mm values. |
| `raw_range` | mm | The same distance rounded to mm, saturated at 65535. |
| `sample_index` | — | `i`, the index of the point in its packet (0..95). |
| `packet_seq` | — | `frame_cnt << 16 | udp_cnt` of the packet, as sent. `udp_cnt` restarts every frame. |
| `alpha` | rad | Azimuth in the sensor frame, `atan2(y, x)`. |
| `theta` | rad | Elevation in the sensor frame, `atan2(z, hypot(x, y))`. |

- **Scale/offset:** LASzip's `laszip_add_attribute` always sets the scale and offset option bits. dLidar passes scale 1 and offset 0 for every attribute, so the values are the stored numbers in the units above. Readers that honour scale (laspy) return the integer attributes as floats of the same value.
- **Beam direction in the sensor frame (L2):** `orig = (cosθ·A − sinθ·B, sinθ·A + cosθ·B, C + a_axis_dist)` with `A = (−cosβ sinξ + sinβ cosξ sinα)·r + b_axis_dist`, `B = cosα cosξ·r`, `C = (sinβ sinξ + cosβ cosξ sinα)·r`. Here α = `alpha`, θ = `theta`, r = `range`, and β, ξ, a, b come from the point's packet row in `packetstateNNNN.tsv` (`l2::ParsePoints`). The vertical mount remap above then turns `orig` into X/Y/Z. `|xyz|` differs from `range` by at most `hypot(a_axis_dist, b_axis_dist)`.
- **Matching a point to its packet row:** points are written packet by packet in arrival order. A new packet starts where `packet_seq` changes or `sample_index` does not increase. `gps_time = stamp_ns/1e9 + sample_index·time_increment` holds exactly with that row's `stamp_ns` and `time_increment`.
- **Size:** the extra bytes compress much less than the standard fields. A replay of 190 s grew from 46 MB (format 1) to 109 MB (format 2).

## `packetstateNNNN.tsv` (from format 2)

The columns are the sensor's own (its `RecordingProfile::packet_state_header`). The Mid-360S's are listed in `docs/livox_mid360s.md`. The L2's:

Tab-separated, one header line, then one row per point packet received while recording, in arrival order. Packets that contributed no point are included. The rows are built by `MakePointPacketState` / `AppendPacketStateRow` (`src/adapters/unitree_l2/l2_packet_state.h`) for every point packet `UnitreeL2Sensor::Poll` returns, taken at each chunk with the recording zero. Floats are printed with 9 significant digits, which gives back the received float32 exactly. Integers are exact. A partial last row from a crash is cut off by recovery (`src/adapters/storage/recovery.cpp`).

Units and meanings come from the vendor's protocol header (`src/adapters/unitree_l2/vendor/unitree_lidar_protocol.h`). Where Unitree documents no unit, the column says "undocumented".

| Column | Unit | Meaning |
|---|---|---|
| `stamp_ns` | ns | The packet stamp minus the recording zero, on the same clock and zero as LAZ `gps_time` (×1e9) and the IMU `timestamp`. It is the time of the packet's first point (`j = 0`). |
| `l2_stamp_ns` | ns | The packet stamp as sent (`info.stamp`), absolute L2 clock. `stamp_ns = l2_stamp_ns − zero_l2_ns` (`timingNNNN.json`). |
| `seq` | — | `info.seq` (= LAZ `packet_seq`). It wraps at 1024 on the L2. A step of n > 1 (mod 1024) means n−1 lost point packets. |
| `point_num` | — | Points in the packet (300 on the L2). |
| `time_increment` | s | Time between consecutive points. |
| `scan_period` | s | As sent. |
| `range_min`, `range_max` | m | The packet's valid range limits (points outside are dropped). |
| `apd_temperature` | undocumented (°C likely) | `LidarInsideState`. |
| `apd_voltage` | undocumented (V likely) | `LidarInsideState`. |
| `laser_voltage` | undocumented (V likely) | `LidarInsideState`. |
| `imu_temperature` | undocumented (°C likely) | `LidarInsideState`. |
| `dirty_index` | undocumented | `LidarInsideState`. The SDK's "dirty percentage" is derived from it. |
| `packet_lost_up`, `packet_lost_down` | undocumented | `LidarInsideState`. |
| `sys_rotation_period`, `com_rotation_period` | undocumented (integer) | `LidarInsideState`: "up" and "down" motor rotation periods. |
| `range_scale` | 1 | Calibration: range [m] = `range_scale · (raw + range_bias)`. |
| `range_bias` | mm | Calibration. |
| `a_axis_dist`, `b_axis_dist` | m | Calibration: axis offsets of the sensor model. |
| `theta_angle_bias`, `alpha_angle_bias` | rad | Calibration: added to the scan angles. |
| `beta_angle`, `xi_angle` | rad | Calibration: mirror/axis tilt angles of the sensor model. |
| `angle_min`, `angle_increment` | rad | Alpha of the first sample and its per-sample step. |
| `com_horizontal_angle_start`, `com_horizontal_angle_step` | rad | Theta of the first sample and its per-sample step. |

The unit labels "likely" are guesses from the field names. The values are as sent.

## `timingNNNN.json`

All format 1 keys are unchanged (including its own `"format_version": 1`, the version of the timing report). Keys with `l2` in their name (`zero_l2_ns`, `imu_period_ns_l2`, `map_l2_per_host_ppm`) mean the sensor's own clock, whichever sensor recorded; the names are kept for older tools. Formats 2 and 3 add these keys at the top (`AddRecordingInfo`, `src/domain/recording/recording_info.h`; the versions are queried by the sensor at start-up, e.g. `LiveL2Source::Prepare`):

| Key | Since | Value |
|---|---|---|
| `recording_format_version` | 2 | 2, or 3 from format 3 |
| `sensor_model` | 3 | `"unitree_l2"` or `"livox_mid360s"`. |
| `imu_position_m` | 3 | Only if the sensor knows it (the Mid-360S does, the L2 does not): `[x, y, z]`, the IMU's position relative to the LiDAR origin in m, in the output frame (after the mount), 5 decimals. |
| `lidar_firmware_version` | 2 | From `getVersionOfLidarFirmware`, polled for up to 3 s before recording. `"unknown"` if the L2 sent no version packet in that time. |
| `lidar_hardware_version` | 2 | From `getVersionOfLidarHardware` (up to 1 s), or `"unknown"`. |
| `sdk_version` | 2 | From `getVersionOfSDK` (falls back to `unitree_lidar_sdk_VERSION`, 2.0.4). |
| `work_mode` | 2 | L2: the value sent with `setLidarWorkMode` (`work_mode` in the settings, default 0). −1 if the sensor has none. |

## `imuNNNN.csv` (unchanged)

Space-separated, with the header `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix`. Gyro in rad/s and acceleration in g, with the same axis remap as the points. `imuId` is 0. `timestamp` is ns on the sensor's clock with the same zero as the LAZ `gps_time`. `timestampUnix = timestamp + a constant per recording`. Each row is one IMU sample with its own sensor stamp (L2 about 890 Hz, Mid-360S 200 Hz); nothing is resampled. See `docs/imu_lidar_time_alignment.md`.

## `points_tempNNNN.bin` (crash recovery)

- **Format 3:** a 16-byte header: the magic `dLptsv3\n`, the record size as uint32 (48), and the sensor id as uint32 (`kPointsTempMagic`, `WritePointsTempHeader`, `src/adapters/storage/recording_files.h`). Then raw 48-byte `PointRecord` records (`src/domain/recording/point_record.h`).
- **Format 2:** the same with the magic `dLptsv2\n` and 4 zero bytes in place of the sensor id.
- **Format 1** (older builds): no header, raw 32-byte records (`PointDLidarV1`).

`DetectPointsTempLayout` (`src/adapters/storage/recording_files.h`) recognises all three. Recovery finishes each leftover as the sensor that recorded it: the profile with the header's sensor id (formats 1 and 2: the L2), whichever sensor the settings choose now; a sensor id the build does not know keeps the temp file. Each becomes a LAZ of the format it was recorded in (format 1: no extra bytes, LASzip `generating_software`; format 2: no `system_identifier`). A trailing partial record is ignored.

## Checking a format 3 recording against format 2

Replaying the first 60 s of `packets0001.bin` through `e94a925` (format 2) and format 3 with `tools/golden_replay.sh e94a925`:

- `imu0001.csv`, `packets0001.bin`, `packetstate0001.tsv` and the console log were byte-identical.
- `lidar0001.laz` differed in 11 header bytes only: `system_identifier` (`Unitree L2`) and the `2` → `3` of `generating_software`. Every point record, standard fields and extra bytes, was byte-identical.
- `timing0001.json` differed only in `recording_format_version` (2 → 3) and the new `sensor_model` line.

`tools/verify_recording_format.py base/rec new/rec` (with `~/venv-laspy`) passed every check, including every extra byte identical point for point (3 811 166 points).

## Checking a format 2 recording against format 1

The change was verified by replaying the same packets through the previous code (commit `d68191b`) and the new code. The replay used the real packet sequence of recording 0001 (L2 stamps, seq, host arrival times, `time_increment`) with synthetic packet contents, because the sensor was not available. The results:

- Point count, header bounds, X/Y/Z, `gps_time`, `intensity`, `user_data` and all other standard fields were identical point for point (11 286 510 points).
- `imuNNNN.csv` and `packetsNNNN.bin` were byte-identical.
- Every old `timingNNNN.json` key had the same value.
- The only new file was `packetstateNNNN.tsv`.
- On the extra bytes and the sidecar: `|xyz| − range` stayed within the axis offsets, `sample_index` was within 0..299, and `range` equalled `range_scale·(raw_range+range_bias)` exactly. `packet_seq` and `gps_time` matched the sidecar row of every point. The sidecar had one row per point packet of `packetsNNNN.bin`, and its seq gaps matched the packet log.

To repeat the format 1 check, run `tools/compare_recording_formats.sh [OLD_REV] [PACKETS_BIN] [SECONDS] [OUT_DIR]`. It builds the frozen `tools/replay_capture_format1.cpp` against `OLD_REV`'s `include/` and uses the working tree's `bin/replay_capture`, replays the packet log through both, and runs `tools/verify_recording_format.py OLD_DIR NEW_DIR`, which needs `laspy[lazrs]` and numpy. `bin/replay_capture` alone writes one replayed recording. For later format changes, compare the working tree with a revision of the current layout: `tools/golden_replay.sh REV PACKETS SECONDS OUT_DIR`, then `tools/verify_recording_format.py OUT_DIR/base/rec OUT_DIR/new/rec`.
