# Proposal

## Why

A recording ends today after a fixed number of chunks (`target_chunks`, about 105 s on the Mid-360S, about 5 min on the L2) or on Ctrl-C over SSH. Neither fits a handheld scanner in a cave: a survey leg has no fixed length, and there is no laptop.

The device needs a push button that starts and stops recordings, and it must run without anyone starting a program. The Mid-360S arrives in about two weeks. Its IMU then runs at 500 Hz instead of the factory 200 Hz, for handheld SLAM with fast turns. That setting was deferred in `add-livox-mid360s-sensor`.

## What Changes

- **Service mode (new default):** `bin/dLidar` runs as a systemd service.
  - At start it recovers leftovers, opens the sensor and waits in Idle.
  - A short press of the button starts a recording; the next short press stops and finalizes it, and dLidar waits for the next press.
  - SIGTERM or SIGINT finalizes a running recording and exits.
  - The repository gets a unit file and install instructions.
- **Push button:** a new GPIO input adapter.
  - The button is wired from a GPIO pin to GND and uses the internal pull-up, debounced. The pin is GPIO 17 (header pin 11) by default, `[button] gpio` in the settings.
  - It uses the kernel's GPIO character-device API, so it needs no new library.
  - **5 s long press:** stops and finalizes any recording, then powers the Pi down. A polkit rule allows the service user to do that.
- **No length limit any more. BREAKING:**
  - `target_chunks` leaves the settings file. A file that still has it is rejected, with an error that names the button.
  - A recording ends only by button, signal, sensor failure (as today) or low disk space.
  - The core keeps an optional chunk limit for the replay tool.
- **Status LED:** a new GPIO output channel.
  - The LED is wired from a GPIO pin through a resistor to GND, active high: GPIO 27 (header pin 13) by default, `[led] gpio` in the settings.
  - **On:** a recording was started (from the press on, through the sensor start-up and the recording).
  - **Blinking (2 Hz):** the recording is being saved (finalized).
  - **Off:** idle.
  - **Three fast flashes, then off:** the recording failed.
  - It runs as an async `EventHub` channel, so a GPIO problem never affects a recording.
- **Low disk space stop:** when the free space on the recording disk drops below what finalizing the LAZ needs plus a margin, the recording stops and is finalized with a warning, instead of filling the SD card.
- **`--once`:** today's behaviour for tests: start one recording at once, stop on Ctrl-C, exit.
- **Mid-360S IMU:**
  - At start-up dLidar sets the IMU output rate and full-scale ranges. Defaults are 500 Hz, ±8 g and ±2000 °/s, from `imu_rate_hz`, `imu_accel_range_g` and `imu_gyro_range_dps` in `[livox_mid360s]`.
  - The command is best effort: it needs firmware V35.1.0112+, and an unanswered command leaves the sensor at 200 Hz with a warning.
- **Mid-360S between recordings:** the sensor stays open between recordings. Its packets are dropped while no recording runs, so the packet pool cannot grow, and each recording runs the start-up again.

## Capabilities

### New Capabilities
- `button-control`: the push button as an input channel. Wiring and debounce, short press to start or stop, long press to shut down.
- `status-led`: the status LED as an output channel. Wiring and what each light pattern means.
- `service-mode`: dLidar as a long-running device service. Start-up, waiting for commands, signals, the systemd unit, `--once`.

### Modified Capabilities
- `device-control`:
  - Recordings no longer end on a configured length.
  - A recording stops on low disk space.
  - The "CLI one-shot mode unchanged" requirement becomes `--once`.
- `device-settings` (defined in the unarchived change `add-livox-mid360s-sensor`, so ADDED requirements only): the `[button]` section, the IMU keys, and `target_chunks` is rejected.
- `livox-mid360s-sensor` (same; ADDED requirements only): IMU rate and range at start-up; no packets kept while idle.

## Impact

- **Code:**
  - New `src/adapters/gpio/` (button input and status LED), with press detection and the LED patterns testable without hardware.
  - New `src/app/` service loop in `main.cpp`.
  - Storage: a free-space check behind the `RecordingStorage` port.
  - Core: `RecordSession` without a mandatory chunk limit, and the low-space stop.
  - `LiveLivoxSource`: IMU range command, dropping packets while idle, and `Prepare`/`Stop` that can run again.
  - Settings: new keys, `target_chunks` removed.
- **System:** `systemd/dlidar.service`, `systemd/50-dlidar-poweroff.rules` (polkit), the service user in group `gpio`; installed by hand as documented.
- **Output and dMapper:**
  - The files and their format are unchanged; recordings are just of any length.
  - Mid-360S IMU CSVs have 500 rows/s instead of 200. dMapper reads the rate from the timestamps, but its Mid-360S settings must assume 500 Hz noise.
- **Runtime:** a 1 GB Pi 5 must hold long recordings. Memory does not grow with recording length: points stream to the temp file. Disk use is about 9.6 MB/s on the Mid-360S, and the low-space stop protects the card.
- **Out of scope:**
  - A buzzer, or LED patterns beyond the four above.
  - Bluetooth or phone control.
  - A recording length limit in seconds.
