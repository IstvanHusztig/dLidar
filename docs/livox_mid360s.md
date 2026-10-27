# Livox Mid-360S in dLidar

How dLidar records the Livox Mid-360S, the second supported sensor next to the Unitree L2. To select it, set `sensor = "livox_mid360s"` in the settings file (`docs/manual.html`, "Settings"). The recording files are the same as for the L2, and their fields are described in `docs/recording-format.md`.

**Status:** built and tested against synthetic packets only. The sensor has not been connected yet; the bring-up checklist at the end confirms the assumptions marked *to confirm*.

## The sensor

| Item | Value | Source |
|---|---|---|
| Range | 0.1 m blind zone (0.1–0.2 m reference only), 40 m at 10 % reflectivity, 100 m cut-off | Mid-360S spec sheet |
| FoV | 360° horizontal, −7°…52° vertical | spec sheet |
| Points | 200,000 points/s, first return, 10 Hz frames; 96 points per UDP packet (~2,083 packets/s) | spec sheet, Livox protocol |
| IMU | ICM40609, 200 Hz at power-on (50/100/200/500 Hz settable); gyro rad/s, accel g | spec sheet, protocol |
| Time | Packet stamp in ns, `time_type` 0: since power-on, 1: PTP, 2: GPS. dLidar uses the stamps as they come (no sync needed: IMU and points share the clock). | protocol |
| Network | Sensor 192.168.1.1XX (XX: last two digits of the serial number), ports 56100–56500; host ports 56101–56501 | user manual |
| Power | 6.5 W typical, up to 14 W self-heating (−20…0 °C), 9–27 V | spec sheet |

## How dLidar reads it

`src/adapters/livox_mid360s/` (CMake `dlidar_livox`) is the only code that sees Livox-SDK2:

- **`LiveLivoxSource`** (`live_livox_source.cpp`), the only user of the SDK library:
  - **Bridge:** the SDK calls the point and IMU callbacks on its data thread. Each callback stamps the host arrival time first and copies the packet into a `LivoxPacketPool` (`livox_packet_pool.h`), which the capture thread reads in arrival order.
  - **The pool never drops:** it starts with 8,192 slots (~12 MB, ~4 s) and grows by 1,024 if the capture falls behind, with one console warning.
  - **Start-up (`Prepare`):**
    1. Wait up to 10 s for the sensor to announce itself. Device type 35 is the Mid-360S; a plain Mid-360 (type 9) is accepted with a warning.
    2. Set work mode normal, IMU on and point type Cartesian high, and query the firmware version.
    3. Wait for Cartesian point packets: no packet for 10 s, or no points within 60 s, fails.
    4. Drop the backlog for 2 s.
  - **Commands are best effort:** Livox-SDK2 issue #125 reports that commands time out on a Mid-360S while data still flows. A command without an answer only prints a warning, and only "no data" fails the start. After power-on the sensor samples with the IMU on.
  - **`Stop`** sets work mode sleep (best effort).
  - **SDK configuration:** written from the settings as JSON to `$XDG_RUNTIME_DIR` (or `/tmp`) and passed by path; `LivoxLidarSdkInit(nullptr, ...)` crashes (issue #135).
  - **Ctrl-C:** during the start-up it stops the start within ~10 ms, but `LivoxLidarSdkUninit` takes another ~1.1 s while the SDK joins its threads.
- **`LivoxMid360sSensor`** (`livox_mid360s_sensor.cpp`) is the `LidarSensor`:
  - IMU packets give one sample each. The 16-bit `udp_cnt` of the IMU packets is widened to a 32-bit seq, so lost IMU packets are counted.
  - Cartesian high point packets give points. Point packets of other data types are logged without points, and packets cut short or of an unknown type are skipped; both are counted.
- **`livox_conversion.cpp`:** the packet math, using only the packed structs of `livox_lidar_def.h`.
  - **Units:** xyz in m from the int32 mm values, intensity = reflectivity (0–255).
  - **Point time:** point i is at `stamp + i · time_interval · 100 ns / (dot_num − 1)`. *To confirm:* the Livox ROS driver divides by `dot_num`, while the protocol page defines `time_interval` as first-to-last point. The difference is < 1 % of 5 µs.
  - **Accel:** IMU acceleration g × 9.80665 → m/s², because the port is SI; the core converts back to g.
- **`SyntheticLivoxSource`:** replays a `packetsNNNN.bin` as Mid-360S packets (`bin/replay_capture LOG OUT 60 --sensor=livox_mid360s`).

### Frames and the IMU position

- **Shared axes:** points and IMU share the Mid-360S frame (+X towards the Livox logo). The IMU sits at **(+11.0, +23.29, −44.12) mm** from the LiDAR origin in that frame (user manual). dLidar writes it, mapped through the mount, as `imu_position_m` in `timingNNNN.json`. Neither the sensor nor dLidar applies a lever arm to the data.
- **Mount:** the default `mount = "+Z +Y -X"` assumes the Mid-360S is mounted like the L2: its +Z to the front and its −X up. *To confirm* at bring-up, then set in the settings file.

### Tag filter

Each point has a tag byte with three 2-bit confidence fields: bits 0–1 glued objects, bits 2–3 rain/fog/dust, bits 4–5 other (0 high, 1 medium, 2 low). By default every point is kept. `drop_low_confidence = true` drops the points with "low" in the glue or rain/fog/dust field. `packetstateNNNN.tsv` counts them per packet either way.

### `packetstateNNNN.tsv` columns

| Column | Meaning |
|---|---|
| `stamp_ns` | packet stamp − recording zero (same clock and zero as the LAZ `gps_time` and the IMU `timestamp`) |
| `livox_stamp_ns` | packet stamp as sent, absolute sensor clock |
| `time_type` | 0 since power-on, 1 PTP, 2 GPS; a change shows a lost or gained sync |
| `data_type` | 1 Cartesian high (recorded); 2, 3 are logged without points |
| `frame_cnt`, `udp_cnt` | as sent (`packet_seq` = `frame_cnt << 16 | udp_cnt`) |
| `dot_num` | points in the packet (96) |
| `time_interval_100ns` | first to last point, 0.1 µs |
| `low_confidence` | points tagged low confidence |

## What the recordings contain

- **Files and units:** the same files, units, axes and timestamp rules as the L2 (`docs/recording-format.md`), with `sensor_model` `livox_mid360s` and the LAS system identifier `Livox Mid-360S`.
- **IMU:** 200 Hz in the IMU CSV, against about 890 Hz on the L2.
- **Points:** ring (`user_data`) is always 1. The extra bytes hold the distance, the azimuth/elevation and the packet's frame/udp counters.
- **Default filter:** 0.1 m to 100 m, no intensity floor (`livox::kDefaultFilter`); tune it at bring-up.
- **Recording length:** 1,500 chunks of 14,000 points last about 105 s at 200,000 points/s (`target_chunks` in the settings).
- **Load:** the temp file grows by about 9.6 MB/s, about 3× the L2.

## Build

- **The SDK:** Livox-SDK2 (MIT) is the submodule `external/Livox-SDK2`, pinned to `c0796f0` (master of 2026-09-21, SDK 1.5.2; Mid-360S support is past tag v1.3.1). `git submodule update --init --recursive` fetches it.
- **Targets:** CMake builds only its `sdk_core` static library (`livox_lidar_sdk_static`, warnings off, into `build/lib`), linked privately into `dlidar_livox`.
- **Compiler:** GCC 12 (Debian 12 on the Pi 5) builds it. GCC 13+ needs `<cstdint>` includes the SDK lacks (issue #131).
- **Updating:** update the pin deliberately and re-run the tests. The adapter uses only the packet structs and about ten API calls.

## Resources (measured without the sensor)

`bin/replay_capture LOG OUT 1e9 100000 --sensor=livox_mid360s` on the dev Pi 5 (4 GB), writing to its SD card, with a generated 300 s packet log at Mid-360S rates (2,083 point packets/s, 200 Hz IMU), under `systemd-run --user --scope -p MemoryMax=900M`:

| Measured (2026-10-27) | Value |
|---|---|
| Points | 58.8 million (300 s) |
| Capture | 39.8 s of CPU-bound work for 300 s of data (7.5× real time; the adapter and storage are built `-O0` as in `bin/dLidar`) |
| Finalization | 95.5 s for the 300 s recording; `lidar0001.laz` 1.07 GB (temp file 2.8 GB) |
| Peak RSS | 529 MB, reached during capture, ~430 MB during finalization |
| SD card sustained write (`dd ... conv=fdatasync`) | 51.7 MB/s, 5× the Mid-360S's 9.6 MB/s |

The replay is not paced: it produces the 300 s of data in 40 s, faster than the card takes it, so the chunk queue backs up. That queue is the 529 MB, an upper bound. A live capture produces 9.6 MB/s, which the card writes with room to spare, so its queue stays short. Peak RSS of a live recording is measured at bring-up (checklist step 4).

## Network setup

Give the Pi an address in 192.168.1.0/24 on the Mid-360S's Ethernet link (default `host_ip = "192.168.1.50"`). The L2 stays on 192.168.0.x. If the Pi's Wi-Fi is also on 192.168.1.0/24, the SDK's discovery also reaches that network; use another subnet on one of the two links to avoid that.

## Bring-up checklist (when the sensor is connected)

1. **Network:**
   - `./bin/dLidar` with `sensor = "livox_mid360s"` prints "found Livox device type 35", the firmware (or a warning) and "Mid-360S streaming".
   - Write down which commands printed a "did not confirm" warning (issue #125).
2. **Frame and mount:**
   - With the device level, `imuNNNN.csv` shows accZ ≈ +1 g and accX, accY ≈ 0.
   - Nose down gives the expected accX sign, and left side down the expected accY sign.
   - A wall in front of the device appears at +X in the LAZ.
   - Set `mount` in the settings (and the default in `src/app/device_config.h` if it differs), then repeat.
3. **Timing:**
   - `timingNNNN.json` shows `imu_time_source` `sensor`, the auto checks passed, and `imu_period_ns_l2` ≈ 5,000,000.
   - In `packetstateNNNN.tsv`, the stamp of packet k+1 minus that of packet k ≈ `time_interval_100ns · 100 · 96 / 95` ns (within ~1 µs): this confirms the per-point time formula.
   - `bin/replay_packets packetsNNNN.bin` shows no unexpected gaps.
4. **Resources:** over a 5 min recording on a 1 GB Pi 5 (or the dev Pi 5 under `systemd-run --scope -p MemoryMax=900M`), record peak RSS (`/usr/bin/time -v`), CPU, and any backlog or "packet buffer grew" warnings here.

| Measured | Value |
|---|---|
| Commands without an answer (#125) | — |
| Mount | — |
| Peak RSS / CPU, 5 min | — |
| Finalization time, 5 min | — |
