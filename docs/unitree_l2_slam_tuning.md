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
| IMU timestamps | ns relative to the same zero point (`timestamp` column), plus Unix epoch ns (`timestampUnix`) | `GetOutputImuData` in `include/pcd_manager.h` |
| IMU sample spacing | New burst when the hardware stamp jumps by more than 1.5 ms; samples inside a burst are spaced by **2 ms (500 Hz)**, matching the spec | `GetOutputImuData` |
| Range filter at parse time | Sensor packet `range_min`/`range_max`, plus the parser defaults `0`–`100` m. No blind-zone cutoff is applied in dLidar, so the SLAM min-range filter is the one that matters. | `parseFromPacketToPointCloud` |
| LAZ precision | Scale factor 0.0001 m (0.1 mm), well below the 4.5 mm sensor resolution | `include/datawriter.h` |
| IMU CSV format | Space-separated: `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix` | `include/datawriter.h` |

### Open points

- ~~**IMU rate mismatch:**~~ Fixed: the interpolation step was 1.667 ms (600 Hz), which made IMU time drift against point time. It is now 2 ms (500 Hz), matching the spec. If deskewing still looks off, check the real rate from the hardware stamps.
