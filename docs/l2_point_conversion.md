# From L2 packet to LAZ x/y/z

How dLidar turns a Unitree L2 point packet into the coordinates in `lidarNNNN.laz`, whether the factory calibration is applied, and how this was checked against the vendor SDK. References are `file:line` in this repo.

**Short answer:** every point uses the unit's factory calibration (all 8 parameters), with the same formula as Unitree's SDK. The only extra step is a fixed axis rotation for the vertical mount. `tests/test_sdk_conversion.cpp` checks this against the compiled SDK.

## The path of one packet

1. **Receive.** The vendor SDK (`lib/<arch>/libunitree_lidar_sdk.a`, version 2.0.4) receives the UDP datagram, checks the header, the CRC32 of the data block and the tail, and returns `LIDAR_POINT_DATA_PACKET_TYPE` from `runParse()`. dLidar takes the **raw packet** with `getLidarPointDataPacket()` (`include/pcd_manager.h:285`). It does not use the SDK's `getPointCloud()`.
2. **Convert.** `CaptureTiming::OnPoints` (`include/capture_timing.h:67-83`) calls `parseFromPacketToPointCloud(cloud_, packet, kMinPointRangeM, 100, kMinPointIntensity)` (`capture_timing.h:77`). This function is in `include/unitree_lidar_utilities.h:230-342`. It is Unitree's own function from the SDK header, with the same math as the original (compare `git show 24bb20d:include/unitree_lidar_utilities.h`, lines 127-220). dLidar changed only the time field (double, L2 clock), the extra filters, the extra fields and the axis remap.
3. **Calibration model** (`unitree_lidar_utilities.h:238-302`), for sample `j` of the packet:
   - `r = range_scale · (ranges[j] + range_bias)` (`:282`). `ranges` is in mm, `range_bias` is in mm, and `range_scale` also converts mm to m (≈ 0.001).
   - `α = angle_min + alpha_angle_bias + j·angle_increment` (`:258`, `:276-277`)
   - `θ = com_horizontal_angle_start + theta_angle_bias + j·com_horizontal_angle_step` (`:260`, `:276-277`)
   - `A = (−cosβ sinξ + sinβ cosξ sinα)·r + b_axis_dist`, `B = cosα cosξ·r`, `C = (sinβ sinξ + cosβ cosξ sinα)·r` (`:296-298`), with β = `beta_angle` and ξ = `xi_angle` (`:238-245`)
   - `orig = (cosθ·A − sinθ·B, sinθ·A + cosθ·B, C + a_axis_dist)` (`:300-302`)

   The calculation is float32, as in the SDK.
4. **Axis remap, the only extra rotation.** `src/dLidar.cpp:8` sets `SensorOrientation::VERTICAL`, so `x = orig_z, y = orig_y, z = −orig_x` (`unitree_lidar_utilities.h:306-316`). This is a proper 90° rotation about Y (determinant +1), not a mirror flip. The IMU gets the same mapping (`:190-204`). There is no lever arm, no mounting rotation and no motion correction.
5. **Store.** The point is written to `points_tempNNNN.bin` as a float32 `PointDLidar` (`unitree_lidar_utilities.h:94-108`), then encoded to LAZ by `WriteLazChunk` (`include/datawriter.h:367`, coordinates at `:483-487`).

## Where the calibration comes from

The L2 sends the calibration in **every point packet**: `LidarPointData.param` (`include/unitree_lidar_protocol.h:162-163`), type `LidarCalibParam` (`unitree_lidar_protocol.h:116-128`):

| Field | Unit | Used at |
|---|---|---|
| `a_axis_dist` | m | `unitree_lidar_utilities.h:302` |
| `b_axis_dist` | m | `:296` |
| `theta_angle_bias` | rad | `:260` |
| `alpha_angle_bias` | rad | `:258` |
| `beta_angle` | rad | `:238-239` |
| `xi_angle` | rad | `:240-241` |
| `range_bias` | mm | `:282` |
| `range_scale` | 1 (×mm → m) | `:282` |

Every packet is converted with its own `param`, so no separate query is needed and the parameters can never be stale. In recording format 2, each packet's 8 values are also written as one row of `packetstateNNNN.tsv` (`include/packet_state.h:54`, `:73-75`, written at `datawriter.h:766`).

**The values of our unit are not known yet.** No real format 2 recording exists. `~/PointCloudDump/` holds only recording 0001, which is format 1 and has no `packetstate` file. The `packetstate0001.tsv` in `~/dLidar_format_samples/format2_new/` is a replay with **synthetic** values (`tools/replay_capture.cpp:45-52`), not the unit's. Any recording with the current build gives the real values in the `range_scale` to `xi_angle` columns of `packetstateNNNN.tsv`.

## Filtering

A sample is dropped (`unitree_lidar_utilities.h:279-289`) if:

- `ranges[j] < 1` (no return), as in the SDK;
- `r` is outside the packet's own `range_min`/`range_max`, as in the SDK;
- `r < 0.1 m` (`kMinPointRangeM`, `capture_timing.h:15`) or `r > 100 m`. The SDK uses 0-100 m;
- intensity `< 5` (`kMinPointIntensity`, `capture_timing.h:19`). The SDK has no intensity filter.

dLidar adds only the last two. There is no downsampling, voxel filter or outlier removal. The L2 firmware also removes points it judges to be dirt on the window. That is not visible to dLidar except as the dirty percentage that `CheckDirtyPercentage` warns about.

## Precision

| Stage | Resolution / error |
|---|---|
| Raw range | 1 mm (uint16 mm) |
| float32 model vs the same model in double | ≤ 0.5 mm up to 45 m (measured by the test). This is the SDK's own float math, and dLidar keeps it. |
| float32 storage in the temp file | ≈ 4 µm at 50 m |
| LAZ scale | 0.0001 m = 0.1 mm (`datawriter.h:417-419`), offset 0 |
| `gps_time` | double seconds from the first point packet |

All of these are far below the L2's own range noise (centimetre level).

## Test against the SDK: `tests/test_sdk_conversion.cpp`

Point packets with known contents are sent over loopback UDP into the **compiled vendor SDK** (`initializeUDP` on 127.0.0.1). Each packet has 300 random ranges (0-45 m), some empty samples and random intensities. Three calibration sets are used: none, small (mrad / mm) and large (tens of mrad, 2% scale). For each packet:

- the SDK's own `getPointCloud()` result is the reference;
- the packet the SDK parsed goes through dLidar's real path (`CaptureTiming::OnPoints`, VERTICAL, filters), and the remap is undone;
- the points are matched by sample index and their xyz and intensity are compared;
- the test checks that every point missing from dLidar's output is explained by `kMinPointIntensity` or `kMinPointRangeM`.

Result (`ctest --test-dir build -R sdk_conversion`), 12 packets, about 1160 points per set:

| Calibration | max \|SDK − dLidar\| | Effect of the calibration (largest move, per m of range) |
|---|---|---|
| none | 0 | 0 |
| small | 3.8 µm | 10.8 mm/m |
| large | 4.3 µm | 77 mm/m |

The µm differences are float rounding (dLidar is built `-O0`; at `-O2` they are exactly 0). They are 25× below the LAZ resolution.

Notes found on the way: the SDK's CRC covers only the data block, not the header. `closeUDP()` in SDK 2.0.4 recurses until the stack overflows (dLidar never calls it). The first cloud from `getPointCloud()` has absolute times in `PointUnitree.time`.

## How much error missing calibration would cause

dLidar applies the calibration, so this applies only to software that recomputes points from raw ranges without it. The error grows with range:

- angle biases (`theta_angle_bias`, `alpha_angle_bias`) and the tilts (`beta_angle`, `xi_angle`): about `r · angle`, so 1 mrad gives 1 mm per metre (10 mm at 10 m, 30 mm at 30 m);
- `range_bias`: a constant `range_bias · range_scale` along the beam (mm);
- `range_scale`: its relative deviation from 0.001, times `r`;
- `a_axis_dist`, `b_axis_dist`: a constant offset of their size (mm).

With mrad-sized parameters ("small" in the test), points move by up to about 11 mm per metre of range, so about 0.1 m at 10 m. The real figure needs the unit's values from a format 2 recording.
