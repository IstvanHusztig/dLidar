# Tasks

After every group, run these on the Pi 5:
- `cmake --build build`
- `ctest --test-dir build --output-on-failure`
- `tools/golden_replay.sh`, which must stay identical: the replay tool keeps its chunk limit, so no output changes.

## 1. No length limit in the device

- [x] 1.1 `SessionConfig::target_chunks` gets `0 = no limit` (`record_session.h`; comment and loop condition). Remove `kTargetChunks` from `device_config.h`. `main` sets 0, and `replay_capture` keeps 800 or its argument. Add a `test_record_session.cpp` case: with `target_chunks = 0`, a FakeSensor script of 5 chunks records all 5, and the recording ends only at the end of the stream or on a stop. Verify with ctest `record_session` and an identical `tools/golden_replay.sh`.
- [x] 1.2 Settings: `target_chunks` is rejected with "recordings are started and stopped with the button ([button]); remove target_chunks" and the line number. Remove it from `DeviceSettings`, the manual's settings table and `CLAUDE.md` (Runtime Assumptions). Extend `test_settings.cpp`. Verify with ctest `settings`.

## 2. Low disk space stop

- [x] 2.1 Add `RecordingStorage::HasRoom(std::string &why)` (default true) to `src/application/ports/recording_storage.h`. `RecordSession` calls it on the writer thread after each `Append`. Without room, it sets `low_space_`, sends a `WarningKind::StorageLowSpace` warning with the text, and the capture loop stops as on a stop command. The recording is finalized and ends in Idle. `MemoryStorage` gets a settable `has_room`. Add a `test_record_session.cpp` case: room runs out at chunk 3, so 3 chunks are written, `Close` is called, the state is Idle and one warning is sent. Add a `ConsoleEvents` text for the warning. Verify with ctest.
- [x] 2.2 `FileRecordingStorage::HasRoom`: `statvfs` on its directory, needing `points_temp size / 2 + 500 MB`; `Open()` refuses below 500 MB free with a message. The free-space function is injectable. Extend `test_datawriter.cpp`: a fake free-space value below the need gives `HasRoom` false with both numbers in `why`, and below 500 MB `Open` fails and creates no file. Verify with ctest `datawriter`.

## 3. Mid-360S: IMU setting and idle packets

- [x] 3.1 Settings `[livox_mid360s]`: `imu_rate_hz` (50/100/200/500, default 500), `imu_accel_range_g` (4/8/16/32, default 8), `imu_gyro_range_dps` (2000/1000/500/250/125/62.5/31.25/15.625, integer or decimal, default 2000). Any other value fails with the allowed list. Pass them through `LiveLivoxConfig` (factory). Extend `test_settings.cpp`. Verify with ctest `settings`.
- [x] 3.2 `LiveLivoxSource::Prepare` sends `SetLivoxLidarImuRange` after `EnableLivoxLidarImuData`, mapped to the SDK enums, best effort. Without an answer it warns "firmware V35.1.0112+ needed; the sensor keeps its IMU setting". The printed start-up summary includes the IMU request. Verify that it builds, that the no-sensor run still ends in Error after about 10 s with no files, and that the enum mapping is a pure function with a unit test in `test_livox_sensor.cpp` (or a new `test_livox_imu_range.cpp`).
- [x] 3.3 Idle packets (design D6):
  - add `accepting_` (packets dropped while false; true from `Prepare`; `Stop` sets it false and clears the pool);
  - make `Stop` repeatable (reset in `Prepare`);
  - `Prepare` skips the search when a device is known;
  - `OnInfoChange` keeps the latest handle;
  - the config path uses `$RUNTIME_DIRECTORY`, then `$XDG_RUNTIME_DIR`, then `/tmp`.

  Factor the accept/drop gate into the pool (`LivoxPacketPool::SetAccepting`) so it is testable, and extend `test_livox_packet_pool.cpp`: pushes while not accepting are dropped and counted, `Clear` on stop, and no growth in 10 s of pushes while idle. Verify with ctest `livox_packet_pool`.

## 4. GPIO adapter: button and LED

- [x] 4.1 `src/adapters/gpio/press_detector.h`: debounce 50 ms, `Short` on a release within 5 s, `Long` at 5 s held (before the release), nothing on the release after a long press. Add `tests/adapters/gpio/test_press_detector.cpp` covering:
  - a clean short press;
  - bounces within 20 ms giving one press;
  - a glitch shorter than 50 ms giving nothing;
  - `Long` firing at 5.0 s exactly from `Tick`;
  - a release after a long press giving nothing;
  - two presses 100 ms apart giving two presses.

  Verify with ctest `press_detector`.
- [x] 4.2 `gpio_line.{h,cpp}` (the only `<linux/gpio.h>` user), as in design D2:
  - find the chip by label (`pinctrl-rp1`, else `pinctrl-bcm2835`);
  - request the v2 line with pull-up and both edges, debounce 50 ms, and on `EINVAL` retry without debounce;
  - `Wait(timeout_ms)` returns the level changes;
  - on failure, a clear error naming the device, `errno` and the `gpio` group / `[button] gpio`.

  Lines can be opened as input (pull-up, edges) or as output (initially low; `Set(bool)`). Add the CMake target `dlidar_gpio` and ctest `architecture`. Verify that on this Pi (user not in `gpio`), opening GPIO 17 fails with the permission message. With `sudo` (only if the user agrees), it opens and reads the idle level as released.
- [x] 4.3 `button_input.{h,cpp}`: a thread running `GpioLine` → `PressDetector` → `on_short`/`on_long` callbacks, with `Tick` every 100 ms and stopping within 100 ms. It takes the line through an interface (`ButtonLine`), so `tests/adapters/gpio/test_button_input.cpp` can drive it with a scripted fake line (short, long, stop while held). Verify with ctest.

- [x] 4.4 `led_pattern.h` (pure: state + ms since it began → on/off: Preparing/Recording on, Finalizing 250/250 ms, Error 3 × 100/100 ms then off, Idle off) and `status_led.{h,cpp}` (a `DeviceEvents` channel for `hub.AddAsync`, its own 50 ms thread, one warning on write failure, LED off in the destructor), driving an `LedLine` interface. Add `tests/adapters/gpio/test_status_led.cpp`:
  - the pattern at sample times for every state;
  - the channel with a recording fake line through Idle → Preparing → Recording → Finalizing → Idle (on, then blinking, then off) and → Error (3 flashes, off);
  - a failing fake line printing one warning and never blocking `OnState`.

  Verify with ctest.

## 5. Service mode

- [x] 5.1 `src/app/button_commands.h` (design D3): short press → start, stop or ignore by state; long press → stop, `WaitIdle`, then the injected power-off, with a warning when power-off fails. Add `src/app/power.cpp` (`systemctl poweroff` through fork/exec, its exit code returned). Add `tests/app/test_button_commands.cpp` with a real `ScannerController`, `FakeSensor`, `MemoryStorage` and a recording power-off fake. It checks:
  - start → stop → start → stop gives two recordings;
  - a press while Finalizing is ignored;
  - a long press while recording finalizes before power-off;
  - a power-off failure keeps the service running.

  Verify with ctest.
- [x] 5.2 `main.cpp`:
  - `--once` gives today's flow;
  - the default is service mode (design D1): settings, storage, recovery, sensor, button (an error exits before opening the sensor), then the loop until a signal or a shutdown;
  - a signal finalizes a running recording and exits 0.

  Add `[button] gpio` (0–27, default 17) and `[led] gpio` (0–27, default 27, not equal to the button's) to the settings, with tests. Open the LED in both modes (`--once` too) and register it with `hub.AddAsync`; a failure to open it is a message, not an exit; it is turned off at exit. Update the usage text. Verify:
  - `bin/dLidar --once --config <loopback Mid-360S file>` behaves as before (Error after about 10 s, no files);
  - `bin/dLidar` without GPIO access exits 2 with the permission message;
  - with a fake `ButtonLine` build flag or test, the loop is covered by 5.1.
- [x] 5.3 Add `systemd/dlidar.service` and `systemd/50-dlidar-poweroff.rules` (design D8), with the user and repo path as placeholders, and an install section in `docs/manual.html` (keep its "Wiring and pinout" section in step with the implemented pins and remove its "planned" note; "Running as a service": copy, `usermod -aG gpio`, `systemctl enable --now dlidar`, `journalctl -fu dlidar`, `--once`, the button and LED wiring, what each press does and each LED pattern means, the press-while-Preparing note, the finalization time per recording minute). Verify `systemd-analyze verify systemd/dlidar.service` (with the placeholders filled in a temp copy) reports no errors.

## 6. Documentation and integration

- [x] 6.1 Update `docs/architecture.md` (the button input channel, the LED output channel and service mode, the `RecordingStorage::HasRoom` port method, the `gpio` adapter), `docs/livox_mid360s.md` (IMU 500 Hz setting, idle dropping, a bring-up check for the IMU rate), `CLAUDE.md` (service mode, button, no chunk limit, `--once`) and the manual. Verify the docs' file paths exist and ctest `architecture` passes.
- [x] 6.2 Integration without hardware: a full clean build and ctest, then `tools/golden_replay.sh` identical. Run `replay_capture --sensor=livox_mid360s` with a fake low free-space value (test hook) and check that the recording is finalized with the warning.

## 7. With hardware (button wired, Mid-360S connected)

- [ ] 7.1 Install the service; after a reboot, press → recording (LED on), press → finalized (LED blinks, then off), repeat 3 times; consecutive numbered files. With the sensor unplugged, a press gives LED on, then 3 flashes and off.
- [ ] 7.2 Long press → the recording is finalized (LED blinks), then the Pi powers off (LED off); a leftover check at the next boot finds nothing.
- [ ] 7.3 Mid-360S IMU: `timingNNNN.json` shows an IMU period of about 2 ms (500 Hz) or the firmware warning; note which in `docs/livox_mid360s.md`.
- [ ] 7.4 Idle for 1 h with the Mid-360S streaming: the service's RSS is constant (`ps -o rss`); the next recording starts cleanly.
