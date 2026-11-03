# Design

## Context

See proposal.md (Why) and specs/ for the requirements. Facts from the code and the Pi that shape the design:

- **Already in the core:**
  - `ScannerController` (`src/application/device/scanner_controller.h`) runs recordings one after another (Idle → Preparing → Recording → Finalizing → Idle/Error). It is thread-safe for input channels: `StartRecording`, `StopRecording`, `Status`, `WaitIdle`.
  - Each recording calls `sensor.Prepare` and, at its end, `sensor.Stop` (`RecordSession`).
  - `main.cpp` is the only one-shot part: it starts one recording and waits.
- **The length limit:** it is `SessionConfig::target_chunks`, checked in the capture loop (`record_session.h:164`). The replay tool sets its own value (800 or a CLI argument), and the golden replay depends on it.
- **Storage threading:** the storage is used only on the writer thread. A failed `Open()` ends the recording in Error before any file exists.
- **Signals:** `src/app/signals.h` raises one stop flag that stays set. It is passed to the controller as an extra stop token, so after a signal every later recording would stop at once. That is right for a service that exits on a signal.
- **`LiveLivoxSource`:**
  - Its callbacks push every packet into the pool from `Open()` on. Nothing reads the pool between recordings, so it would grow by about 3 MB/s while idle.
  - `Stop()` runs only once (`stopped_`).
  - `OnInfoChange` keeps only the first device handle.
- **SDK IMU command:** Livox-SDK2 (`c0796f0`) has `SetLivoxLidarImuRange(handle, LivoxLidarImuOutRate, LivoxLidarAccelRange, LivoxLidarGyroRange, cb, data)`, for the Mid-360S/360L only. The rate enum is 0 = 200 Hz, 1 = 500 Hz, 2 = 100 Hz, 3 = 50 Hz.
- **GPIO on the Pi:**
  - The header GPIOs belong to the chip labelled `pinctrl-rp1` (its `/dev/gpiochipN` number differs between kernels).
  - `/dev/gpiochip*` is `root:gpio 0660`, and `vs2022` is not in group `gpio`.
  - libgpiod 1.6 is installed without its development headers. The kernel uAPI `<linux/gpio.h>` (v2: line requests with bias, edge events and a debounce attribute) is available.
- **Output channels:** `EventHub::AddAsync` gives an output channel its own thread and a bounded queue that drops the oldest event (`src/application/device/event_hub.h`). The planned LEDs were designed for it. `DeviceEvents::OnState` delivers every state change.
- **System:** systemd 252 and polkit (JavaScript rules in `/etc/polkit-1/rules.d`) on Debian 12.

## Goals / Non-Goals

**Goals:**
- The device works from power-on with the button alone, and every way a recording ends leaves complete files.
- The button and power-off logic can be tested without GPIO hardware.
- No change to recording output for the same packets: `tools/golden_replay.sh` stays identical, because the replay tool keeps its chunk limit.

**Non-Goals:**
- A buzzer, Bluetooth or phone control, or LED patterns beyond the four in the status-led spec.
- Several buttons, double clicks, or button-configurable actions.
- Installing the service automatically. It is a documented manual step.

## Decisions

**D1. Service mode lives in the composition root.**
- `main.cpp` gets two modes:
  - `--once`: today's flow (start, `WaitIdle`, exit).
  - The default: open sensor, storage and button, then loop until the signal flag or a shutdown request is set, sleeping 100 ms per iteration. On exit, stop and `WaitIdle` (the stop token already ends a recording on a signal), then `sensor->Stop()`.
- The core is unchanged apart from D4 and D5: the controller already supports many recordings.
- Alternative: a daemon class in `src/application`. Rejected, because the loop is wiring (signals, button, power-off), not a use case.

**D2. The GPIO adapter is `src/adapters/gpio/` (CMake `dlidar_gpio`): the button and the LED, split by job.** Both share `GpioLine`. Adapters may not include each other, so one adapter holds both.
- **`GpioLine`** (`gpio_line.{h,cpp}`): the only code that sees `<linux/gpio.h>`. Opened as an input (button) or as an output that starts low (LED).
  - It finds the chip whose label is `pinctrl-rp1`, falling back to `pinctrl-bcm2835` (Pi 4), via `GPIO_GET_CHIPINFO_IOCTL`.
  - It requests the line with `GPIO_V2_LINE_FLAG_INPUT | BIAS_PULL_UP | EDGE_RISING | EDGE_FALLING` and a 50 ms debounce attribute.
  - `Wait(timeout)` returns level changes with kernel timestamps. If the kernel rejects the debounce attribute, it requests without it and relies on D3's debounce.
- **`PressDetector`** (`press_detector.h`): pure, fed `(t, pressed)` changes and `Tick(t)`.
  - It debounces (a level must hold 50 ms).
  - It reports `Short` on a release within 5 s, and `Long` once the press has been held for 5 s, before the release. The release that follows a long press reports nothing.
- **`ButtonInput`** (`button_input.{h,cpp}`): owns the thread: `GpioLine` → `PressDetector` → actions.
  - The actions are two callbacks given by `main`: `on_short` and `on_long`. The adapter needs no core knowledge beyond them.
  - It stops with a flag within 100 ms.
- The press → command mapping (D3) is in `src/app/button_commands.h`, which is testable with a real `ScannerController` and fakes.
- Alternative: libgpiod. Rejected, because it is not installed as a development package, and the uAPI is about 60 lines.

**D9. Status LED as an async output channel (`src/adapters/gpio/status_led.{h,cpp}`).**
- **`StatusLed`** implements `DeviceEvents` and is registered with `hub.AddAsync`, so it gets its own queue and thread. `OnState` updates the wanted pattern.
- **Its own blink thread** sets the line every 50 ms from a pure `LedPattern` (`led_pattern.h`): state + time since the state began → on/off.
  - Preparing and Recording: on.
  - Finalizing: 250 ms on / 250 ms off.
  - Error: on/off at 100 ms three times, then off.
  - Idle: off.
- **Unit tests:** the pattern is a pure function, so they need no GPIO.
- **Failures:** a failed write prints one warning and the channel keeps trying. A line that cannot be opened at start disables the LED with a message.
- **At exit:** `main` turns the LED off, and the destructor also writes low.
- Alternative: drive the LED from `ButtonCommands`. Rejected, because the LED must also show failures, low-space stops and signals that do not come from the button.

**D3. Press → command mapping (`src/app/button_commands.h`).**
- **Short press:** read `controller.Status().state`:
  - Idle or Error: `StartRecording()`.
  - Preparing or Recording: `StopRecording()`.
  - Finalizing: print "press ignored: finalizing".
- **Long press:**
  1. `StopRecording()`, then `WaitIdle()` on the button thread, never on an event callback.
  2. Call the `PowerOff` function: `src/app/power.cpp` runs `systemctl poweroff` with fork/exec and waits for it.
  3. On a non-zero exit, print a warning naming `systemd/50-dlidar-poweroff.rules` and keep running.
- **Fakes in tests:** the power-off function is injected, so tests never power anything down.

**D4. No length limit in the device; the core keeps an optional one.**
- `SessionConfig::target_chunks` becomes `0 = no limit` (the loop condition is `target_chunks == 0 || produced < target_chunks`).
- `main` sets 0; `replay_capture` keeps 800 or its argument, so the golden replay is unchanged.
- The settings parser rejects `target_chunks`: "recordings are started and stopped with the button ([button]); remove target_chunks".
- `kTargetChunks` leaves `device_config.h`.

**D5. Low disk space behind the storage port.**
- `RecordingStorage` gains `virtual bool HasRoom(std::string &why) { return true; }`, called on the writer thread after each `Append`.
- `FileRecordingStorage` uses `statvfs` on its directory. It needs `points_temp size / 2 + 500 MB` free; the LAZ was 0.38× the temp file in the 300 s replay, and the CSV is already written. The free-space function is injectable for tests.
- When there is no room, `RecordSession` sets `low_space_` (writer thread) and sends a `StorageLowSpace` warning naming free and needed space. The capture loop then stops as on a stop command, so the recording is finalized and the state returns to Idle, not Error.
- `Open()` refuses with less than 500 MB free. That is the existing open failure, which ends in Error with no files.
- Alternative: check in the capture loop. Rejected, because the storage belongs to the writer thread.

**D6. The Mid-360S between recordings.**
- `LiveLivoxSource` gets `std::atomic<bool> accepting_` (false after `Open`). `OnPacket` returns at once while it is false.
  - `Prepare` sets it to true before waiting for data.
  - `Stop` sets it to false and calls `pool_.Clear()`.
  - The pool therefore never grows while idle.
- **Repeatable `Stop` and `Prepare`:**
  - `Stop` is no longer once-only: the `stopped_` flag is reset in `Prepare`, and `main`'s final `Stop` after `RecordSession`'s is harmless.
  - `Prepare` skips the 10 s search when a device is already known.
  - `OnInfoChange` always stores the latest handle, in case the sensor rebooted.
- **L2:** `Prepare` already restarts the rotation each time, and the SDK's socket drops packets nobody reads.

**D7. Mid-360S IMU setting.**
- `LiveLivoxConfig` gains `imu_rate_hz`, `imu_accel_range_g` and `imu_gyro_range_dps`, which `Prepare` maps to the SDK enums.
- `SetLivoxLidarImuRange` is sent after `EnableLivoxLidarImuData`, best effort with a 2 s wait and a warning on no answer. The warning says "firmware V35.1.0112+ needed; the sensor keeps its IMU setting".
- Settings validate the allowed values (`imu_gyro_range_dps` accepts the decimal values 62.5, 31.25 and 15.625).
- The recording format is unchanged: the measured rate already appears in `timingNNNN.json` (`imu_period_ns_l2`).

**D8. systemd and polkit files in `systemd/`.**
- **`dlidar.service`:**
  - `User=` is filled in at install, and `SupplementaryGroups=gpio`.
  - `After=`/`Wants=network-online.target`.
  - `ExecStart=<repo>/bin/dLidar`.
  - `Restart=on-failure`, `RestartSec=5`: also covers a link that is not up yet, when SDK init or the UDP bind fails at boot.
  - `KillSignal=SIGTERM`, `TimeoutStopSec=3600`: finalizing a 1 h Mid-360S recording takes about 20 min.
  - `Environment=XDG_RUNTIME_DIR=/run/user/%U` is not guaranteed for a system unit, so `RuntimeDirectory=dlidar` is set, and the Livox config path also tries `$RUNTIME_DIRECTORY`.
- **`50-dlidar-poweroff.rules`:** allows `org.freedesktop.login1.power-off` and `power-off-multiple-sessions` for the service user.
- **Documentation:** `docs/manual.html` ("Running as a service") lists the install commands: copy, set the user, `usermod -aG gpio`, `systemctl enable --now`.

## Risks / Trade-offs

- **LED in sunlight or behind the housing:** a small LED may be hard to see. → Pin and wiring are free; a brighter LED or a light pipe is a hardware choice.- **Press while Preparing:** the L2 start-up takes over 10 s and the Mid-360S a few seconds. A second press then stops the recording before it starts (Preparing → Idle, no files), which is the specified behaviour but can surprise. → Documented in the manual.
- **Power-off with an unfinished write:** the long press waits for `Idle` before powering off; a SIGTERM from the power-off itself finds no recording. A crash in between is covered by the leftover recovery.
- **GPIO debounce support:** the debounce attribute may be rejected by older kernels. → Falling back to software debounce in `PressDetector`, which always runs anyway.
- **Disk estimate:** `temp/2 + 500 MB` comes from one synthetic replay. Real scenes compress differently. → The margin is generous and the numbers are printed in the warning; tune it at bring-up.
- **Long recordings:** finalization takes about 0.32× the recording length on the Mid-360S, so a 1 h leg means about 20 min before the next recording can start. → Measured in 7.3; recorded in the manual. Splitting legs is the user's choice.

## Migration Plan

1. **Settings file:** if it has `target_chunks`, delete the line (dLidar says so).
2. **Install the service (once):**
   - add the user to group `gpio` (`usermod -aG gpio <user>`);
   - copy `systemd/dlidar.service` to `/etc/systemd/system/` with the user and repo path set, and `systemd/50-dlidar-poweroff.rules` to `/etc/polkit-1/rules.d/`;
   - run `systemctl enable --now dlidar`.
3. **Tests over SSH:** `bin/dLidar --once` keeps working as before.
4. **Rollback:** `systemctl disable --now dlidar`, then use `--once`.

## Open Questions

- Whether the Mid-360S firmware shipped with the sensor already supports `SetLivoxLidarImuRange` (V35.1.0112+). This is answered at bring-up; it only decides between 500 Hz and the warning.
