# dLidar recording format

What a dLidar recording of the Unitree L2 contains, field by field. It is written for the dMapper side (odometry, mapping and the planned intensity normalization). Source references are `file:line` in this repo.

**Recording format versions** (`kRecordingFormatVersion`, `include/datawriter.h:30`):

| Version | What changed |
|---|---|
| 1 | Everything before the LAZ extra bytes. It is not written anywhere: a file with no `recording_format_version` key is format 1. |
| 2 | LAZ extra bytes (`range`, `raw_range`, `sample_index`, `packet_seq`, `alpha`, `theta`), `packetstateNNNN.tsv`, the sensor and software versions in `timingNNNN.json`, the format in the LAZ `generating_software`, and a header on `points_tempNNNN.bin`. **Additions only:** every format 1 field keeps its value, type and position. |

## Files of a recording

All files are in `/home/$USER/PointCloudDump/` and share the session number NNNN, the next free 4-digit number (`include/datawriter.h:56-100`).

| File | Since | Content |
|---|---|---|
| `lidarNNNN.laz` | 1 | Point cloud (see below). |
| `imuNNNN.csv` | 1 | IMU samples. **Unchanged in format 2.** |
| `timingNNNN.json` | 1 | How the times were made, plus the recording keys (format 2). |
| `packetsNNNN.bin` | 1 | Arrival time and L2 stamp of every packet (diagnostics, `bin/replay_packets`). Unchanged. |
| `packetstateNNNN.tsv` | 2 | One row per point packet: state, calibration and angle parameters. |
| `points_tempNNNN.bin` | 1 | Exists only while recording; the crash-recovery source of the LAZ. |

dMapper reads every `.csv`, `.las` and `.laz` in the folder, so a recording has no other file with those extensions. A recording interrupted by a kill, crash or power loss is finalized on the next start into `recovered_<time>/`, with all its files.

## `lidarNNNN.laz`

LAS 1.2, point format 1, written through the LASzip DLL API (`WriteLazChunk`, `include/datawriter.h`).

### Header

| Field | Value | Since |
|---|---|---|
| version / point format | 1.2 / 1 (`datawriter.h:415`) | 1 |
| point record length | 28 in format 1; **48** in format 2 (28 + 20 extra bytes) | 2 |
| scale x/y/z | 0.0001 (`datawriter.h:417`) | 1 |
| offset x/y/z | 0 (the LASzip default) | 1 |
| `number_of_point_records` (legacy 32-bit) | the point count | 1 |
| `number_of_points_by_return[0]` | the point count (every point has `return_number` 0, which doesn't match) | 1 |
| `global_encoding` | 1. This claims adjusted standard GPS time, but `gps_time` is not GPS time; see below. | 1 |
| `file_source_ID` | 4711 | 1 |
| `generating_software` | format 1: `LASzip DLL 3.4 r4 (231020)`; format 2: **`dLidar recording format 2`** (`datawriter.h:442`) | 2 |
| VLRs | format 2: the standard Extra Bytes VLR (LASF_Spec, record 4), plus LASzip's own | 2 |

### Standard point fields (same in formats 1 and 2)

| Field | Value | Source |
|---|---|---|
| X, Y, Z | The L2's calibrated sensor frame, remapped for the vertical mount: x = orig_z, y = orig_y, z = −orig_x (+X front, +Y left, +Z up). No mounting rotation or lever arm. | `unitree_lidar_utilities.h:313` |
| `intensity` | The raw 0–255 byte as sent (`intensities[j]`), not scaled. It goes byte → float → U16. | `unitree_lidar_utilities.h:329`, `datawriter.h:477` |
| `gps_time` | Seconds since the zero, which is the L2 stamp of the first point packet: `(stamp − zero)/1e9 + j·time_increment`. The packet stamp is the time of its first point. The IMU `timestamp` column uses the same clock and zero, in ns. | `unitree_lidar_utilities.h:271, 331`, `datawriter.h:478` |
| `user_data` | Always 1 (the hard-coded `ring`). It carries no information. | `unitree_lidar_utilities.h:274`, `datawriter.h:479` |
| classification, point_source_ID, return_number, number_of_returns, scan_angle_rank, flags | Never set: always 0. | — |

Which points are written did not change. A point is dropped if `ranges[j]` is 0, if it is outside the packet's `range_min`/`range_max`, if the range is below 0.1 m (`kMinPointRangeM`) or above 100 m, or if the intensity is below 5 (`kMinPointIntensity`) (`unitree_lidar_utilities.h`, `capture_timing.h:15-19`).

### Extra bytes (format 2)

These are stored in the record after the 28 standard bytes, in this order, and are described by the standard Extra Bytes VLR (`kLazExtraAttributes`, `datawriter.h:343`; packing `PackLazExtraBytes`, `datawriter.h:353`). Readers that don't use extra bytes see the format 1 fields unchanged. laspy 2.7 lists them by name and type (verified). Other readers of the standard VLR (PDAL, LAStools, CloudCompare) should do the same, but that was not tested.

| Name | Type | Byte | Unit | Meaning | Set at |
|---|---|---|---|---|---|
| `range` | float32 | 0 | m | The corrected range used for xyz: `range_scale * (ranges[j] + range_bias)`, as a float32 computed exactly as in the parser. | `unitree_lidar_utilities.h:282, 333` |
| `raw_range` | uint16 | 4 | mm | `ranges[j]` as sent by the L2. | `:337` |
| `sample_index` | uint16 | 6 | — | `j`, the index of the point in its packet (0..`point_num`−1, 0..299 on the L2). Filtered points leave gaps, but the remaining points keep their own `j`. | `:338` |
| `packet_seq` | uint32 | 8 | — | The packet's `info.seq`. On the L2 it wraps at 1024 (measured). Use it with `packetstateNNNN.tsv`. | `:336` |
| `alpha` | float32 | 12 | rad | The alpha angle used for the point: `angle_min + alpha_angle_bias`, plus `angle_increment` once per sample, accumulated in float32 exactly as in the parser. | `:258, 334` |
| `theta` | float32 | 16 | rad | The theta angle used for the point: `com_horizontal_angle_start + theta_angle_bias`, plus `com_horizontal_angle_step` per sample, accumulated in float32 as in the parser. Not wrapped to [0, 2π). | `:260, 335` |

- **Scale/offset:** LASzip's `laszip_add_attribute` always sets the scale and offset option bits. dLidar passes scale 1 and offset 0 for every attribute, so the values are the stored numbers in the units above. Readers that honour scale (laspy) return the integer attributes as floats of the same value.
- **Beam direction in the sensor frame:** `orig = (cosθ·A − sinθ·B, sinθ·A + cosθ·B, C + a_axis_dist)` with `A = (−cosβ sinξ + sinβ cosξ sinα)·r + b_axis_dist`, `B = cosα cosξ·r`, `C = (sinβ sinξ + cosβ cosξ sinα)·r`. Here α = `alpha`, θ = `theta`, r = `range`, and β, ξ, a, b come from the point's packet row in `packetstateNNNN.tsv` (`unitree_lidar_utilities.h:288-302`). The VERTICAL remap above then turns `orig` into X/Y/Z. `|xyz|` differs from `range` by at most `hypot(a_axis_dist, b_axis_dist)`.
- **Matching a point to its packet row:** points are written packet by packet in arrival order. A new packet starts where `packet_seq` changes or `sample_index` does not increase. `gps_time = stamp_ns/1e9 + sample_index·time_increment` holds exactly with that row's `stamp_ns` and `time_increment`.
- **Size:** the extra bytes compress much less than the standard fields. A replay of 190 s grew from 46 MB (format 1) to 109 MB (format 2).

## `packetstateNNNN.tsv` (format 2)

Tab-separated, one header line, then one row per point packet received while recording, in arrival order. Packets that contributed no point are included. The rows are built by `MakePointPacketState` / `AppendPacketStateRow` (`include/packet_state.h:31, 60`) from `CaptureTiming::OnPoints` (`include/capture_timing.h:73`). Floats are printed with 9 significant digits, which gives back the received float32 exactly. Integers are exact. A partial last row from a crash is cut off by recovery (`datawriter.h:587`).

Units and meanings come from the vendor's protocol header (`include/unitree_lidar_protocol.h`). Where Unitree documents no unit, the column says "undocumented".

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

All format 1 keys are unchanged (including its own `"format_version": 1`, the version of the timing report). Format 2 adds these keys at the top (`AddRecordingInfo`, `datawriter.h:87`; queried in `QueryRecordingInfo`, `include/pcd_manager.h:111`):

| Key | Since | Value |
|---|---|---|
| `recording_format_version` | 2 | 2 |
| `lidar_firmware_version` | 2 | From `getVersionOfLidarFirmware`, polled for up to 3 s before recording. `"unknown"` if the L2 sent no version packet in that time. |
| `lidar_hardware_version` | 2 | From `getVersionOfLidarHardware` (up to 1 s), or `"unknown"`. |
| `sdk_version` | 2 | From `getVersionOfSDK` (falls back to `unitree_lidar_sdk_VERSION`, 2.0.4). |
| `work_mode` | 2 | The value sent with `setLidarWorkMode` (`kLidarWorkMode = 0`, `src/dLidar.cpp:4`). |

## `imuNNNN.csv` (unchanged)

Space-separated, with the header `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix`. Gyro in rad/s and acceleration in g, with the same axis remap as the points. `imuId` is 0. `timestamp` is ns on the L2 clock with the same zero as the LAZ `gps_time`. `timestampUnix = timestamp + a constant per recording`. Each row is one L2 IMU packet with its own stamp; nothing is resampled. See `docs/imu_lidar_time_alignment.md`.

## `points_tempNNNN.bin` (crash recovery)

- **Format 2:** a 16-byte header: the magic `dLptsv2\n`, the record size as uint32 (48), and 4 reserved bytes (`kPointsTempMagic`, `datawriter.h:245`). Then raw 48-byte `PointDLidar` records (`unitree_lidar_utilities.h`).
- **Format 1** (older builds): no header, raw 32-byte records (`PointDLidarV1`).

`DetectPointsTempLayout` (`datawriter.h:279`) recognises both. A format 1 temp file is still recovered and becomes a format 1 LAZ (no extra bytes, LASzip `generating_software`). A trailing partial record is ignored.

## Checking a format 2 recording against format 1

The change was verified by replaying the same packets through the previous code (commit `d68191b`) and the new code. The replay used the real packet sequence of recording 0001 (L2 stamps, seq, host arrival times, `time_increment`) with synthetic packet contents, because the sensor was not available. The results:

- Point count, header bounds, X/Y/Z, `gps_time`, `intensity`, `user_data` and all other standard fields were identical point for point (11 286 510 points).
- `imuNNNN.csv` and `packetsNNNN.bin` were byte-identical.
- Every old `timingNNNN.json` key had the same value.
- The only new file was `packetstateNNNN.tsv`.
- On the extra bytes and the sidecar: `|xyz| − range` stayed within the axis offsets, `sample_index` was within 0..299, and `range` equalled `range_scale·(raw_range+range_bias)` exactly. `packet_seq` and `gps_time` matched the sidecar row of every point. The sidecar had one row per point packet of `packetsNNNN.bin`, and its seq gaps matched the packet log.

To repeat this check after a later format change, run `tools/compare_recording_formats.sh [OLD_REV] [PACKETS_BIN] [SECONDS] [OUT_DIR]`. It builds `tools/replay_capture.cpp` against `OLD_REV`'s `include/` and against the working tree, replays the packet log through both, and runs `tools/verify_recording_format.py OLD_DIR NEW_DIR`, which needs `laspy[lazrs]` and numpy. `bin/replay_capture` alone writes one replayed recording.
