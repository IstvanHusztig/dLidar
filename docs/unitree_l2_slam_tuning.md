# Unitree L2: SLAM Tuning Reference

Reference data for tuning the SLAM algorithm's `settings/lidar-vertical.toml` to the Unitree L2 as recorded by dLidar. It collects the sensor specs, the operating environment, the parameters we want to tune, and how dLidar's output matches up with them.

## Hardware Specifications (Unitree L2)

| Property | Value |
|---|---|
| Sensor type | 4D ToF solid-state, non-repetitive hemispherical scan |
| Effective point rate | 64,000 pts/s (sampling frequency 128 kHz) |
| Horizontal rotation | 5.55 Hz (~180 ms per full rotation) |
| Vertical mirror frequency | 216 Hz |
| Horizontal FOV | 360° |
| Vertical FOV | 90° (standard), up to 96° (NEGA mode) |
| Accuracy | ≤ 2 cm |
| Distance resolution | 4.5 mm |
| Minimum range / blind zone | 0.05 m |
| Maximum range | 30 m @ 90% reflectivity, 15 m @ 10% reflectivity |
| IMU | 6-axis (3-axis accel + 3-axis gyro) |
| IMU sampling / output rate | 1000 Hz sampling, 500 Hz output |
| Point density | Non-uniform, dense near the optical axis (non-repetitive pattern) |

## Environment & Platform

- **Environment:** cave and outdoor
- **Platform:** handheld
- **Mounting:** vertical (see [dLidar Output Conventions](#dlidar-output-conventions))

## Goals

- Minimize drift.
- Correctly compensate motion distortion (deskewing).
- Produce a clean map.

## Parameters to Tune in `lidar-vertical.toml`

1. **VoxelGrid / downsampling**
   - Leaf sizes for feature extraction and feature matching, chosen from the 64k pts/s density and the 0.05 m close-range capability.
2. **Distance culling (blind zone & max range)**
   - Min cutoff around 0.1–0.2 m, to reject self-reflections off the housing.
   - Max cutoff set to the realistic range (30 m at best, 15 m on dark surfaces).
3. **IMU & point cloud deskewing**
   - IMU noise parameters: accelerometer noise, gyroscope noise, and bias random walk for the 500 Hz output.
   - Timestamp sync: time offset and extrinsic calibration between the LiDAR and IMU frames.
4. **Point-LIO / Fast-LIO specifics**
   - Handling of non-repetitive scanning (N-scans vs. continuous-time processing).
   - Registration iteration count and covariance thresholds, consistent with ≤ 2 cm measurement noise.

## dLidar Output Conventions

These come from the current code and have to match the SLAM config.

| Aspect | dLidar behavior | Source |
|---|---|---|
| Axis frame | `SensorOrientation::VERTICAL` remaps **both** points and IMU from physical (+Z front, +X down, +Y left) to +X front, +Y left, +Z up. The LiDAR↔IMU rotation extrinsic should therefore already be close to identity. | `include/unitree_lidar_utilities.h` |
| Acceleration units | **g** (m/s² divided by 9.80665) | `parseFromImuPacket` |
| Gyro units | rad/s | `parseFromImuPacket` |
| Point time | `double` seconds, relative to a shared zero point (`GetGlobalTimeOffsetNs`) set by the first packet (IMU or point) | `parseFromPacketToPointCloud` |
| IMU timestamps | ns relative to the same zero point (`timestamp` column), plus Unix epoch ns (`timestampUnix` = `timestamp` + one constant offset) | `ImuTimestamper` in `include/imu_timestamper.h` |
| IMU sample spacing | Rebuilt from batched host-arrival stamps as the lower edge of the arrivals: `t_k = t_0 + k·T + o_k`, `t_k <= arrival_k`. T is fitted online (lower convex hull over 20 s, ~1.1947 ms / 837 Hz measured, not hard-coded). Output is delayed 0.5 s. The offset slews at most 0.25% of T per step (1% hard), and steps over lost samples are exactly n·T. No output during the first ~2 s until T settles (written afterwards), and samples before t=0 are dropped. Strictly increasing. | `ImuTimestamper` |
| Range filter at parse time | Sensor packet `range_min`/`range_max`, plus a **0.1 m** minimum applied by dLidar (`kMinPointRangeM`) to reject housing self-reflection. Max stays at the parser default of 100 m; the SLAM max-range cutoff (realistically 15-30 m depending on reflectivity) is not applied here. | `include/pcd_manager.h` |
| Intensity filter at parse time | Points with reflectivity below **5** (0-255 scale, `kMinPointIntensity`) are dropped as likely noise/multipath. | `include/pcd_manager.h` |
| LAZ precision | Scale factor 0.0001 m (0.1 mm), well below the 4.5 mm sensor resolution | `include/datawriter.h` |
| IMU CSV format | Space-separated: `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix` | `include/datawriter.h` |

| Lens dirtiness monitoring | Checked once at startup (`PrintDirtyPercentage`) and now also polled every 10 s during capture (`CheckDirtyPercentage`); logs a warning past **5%** dirty. Non-blocking, reuses the dirty index already cached from the most recently parsed packet. Relevant for handheld cave/outdoor use where the lens can fog or collect dust mid-run. | `include/pcd_manager.h` |

### Open points

- ~~**IMU rate mismatch / backward timestamps:**~~ Fixed: the old fixed 1.667 ms and later 2 ms interpolation steps did not match the real ~882 Hz rate, causing overlapping batches and backward timestamps. The SDK stamps packets with host arrival time (`use_system_timestamp`, default on), so `ImuTimestamper` now rebuilds even spacing. Recordings made before the fix can be repaired with `tools/repair_imu_timestamps.py`.
- **Device-side timestamps:** set `kUseSystemTimestamp = false` in `src/dLidar.cpp` to keep the L2's own clock for points and IMU (synced once at startup). Not yet verified on hardware.
- **Point packet base time:** point packets are also stamped at host arrival, so each packet's base time carries transport jitter. The writer warns if point times ever go backwards.
- ~~**Housing self-reflection / low-reflectivity noise:**~~ Fixed in dLidar: a 0.1 m minimum range and an intensity floor of 5 are now applied at capture time (see table above). Tune `kMinPointRangeM`/`kMinPointIntensity` in `include/pcd_manager.h` if real captures show either cutoff too aggressive or too lax.
- **Non-repetitive scan overlap (item 5):** the L2's non-repetitive Lissajous-style scan pattern means a stationary/slow-moving sensor keeps re-sampling nearly the same points near the optical axis, building up redundant density rather than new coverage. dLidar doesn't (and shouldn't) deduplicate this — it belongs in the SLAM's voxel-downsampling / feature-extraction stage. See the prompt below to have the SLAM side handle it.

## Prompt: Tune `lidar-vertical.toml` for Non-Repetitive Scan Overlap

Use this with the SLAM algorithm's assistant/config to address item 5 above.

```
You are an expert in LiDAR SLAM algorithms and point cloud processing.

I am using a Unitree LiDAR L2, a 4D ToF solid-state sensor with a NON-REPETITIVE
(Lissajous-style) hemispherical scan pattern, not a fixed set of rotating channels.
Horizontal rotation is 5.55 Hz (~180 ms/rotation), vertical mirror is 216 Hz, and
the effective point rate is 64,000 pts/s.

Because the scan is non-repetitive, when the sensor is stationary or moving slowly
(handheld, cave/outdoor exploration), successive rotations re-sample nearly the same
physical points near the optical axis instead of covering new area. Over a longer
capture this creates a strongly non-uniform point cloud: very high, redundant density
near the axis and sparse coverage at the edges of the FOV, which can bias feature
extraction/matching and waste compute in Point-LIO / Fast-LIO style pipelines.

Please review and adjust `settings/lidar-vertical.toml` in this project to properly
account for this non-repetitive scan pattern:

1. VoxelGrid / downsampling:
   - Confirm the leaf size is large enough, and/or configure any available
     time-windowed or angular-diversity-aware downsampling, so that samples are
     drawn across multiple sub-scans/rotations instead of just thinning whatever
     the sensor already over-samples near the axis.
2. Scan accumulation / registration window:
   - Recommend whether to accumulate multiple sub-scans (N-scans) before
     registration, or process in continuous-time mode, given the 180 ms rotation
     period and the non-repetitive pattern - and what N-scan count or time window
     best trades off coverage completeness against motion distortion for a
     handheld platform.
3. Feature extraction:
   - Adjust any point-density-dependent thresholds (e.g. minimum neighbor count,
     curvature/planarity thresholds) so they aren't skewed by the artificially
     high local density near the optical axis.
4. Any other lidar-vertical.toml parameters affected by scanning a target for
   longer than one rotation period with this specific non-repetitive pattern.

Output the concrete parameter values/changes to make in lidar-vertical.toml.
```
