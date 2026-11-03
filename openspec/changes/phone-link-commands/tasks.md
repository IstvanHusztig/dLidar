# Tasks

## 1. Start and stop in the application (D1, D2)

- [x] 1.1 `DeviceApi::Start()` and `Stop()` returning `CommandResult{accepted, state_seen, busy}` (`src/application/remote/device_api.h`). Extend `tests/application/test_device_api.cpp`:
  - with a fake `DeviceControl`: every state for both commands;
  - with `ScannerController` + `FakeSensor` + `MemoryStorage`: start from Idle and Error, `accepted:false` while recording, `busy` while finalizing (a slow `MemoryStorage`), a stop during Preparing writes no files.

  Verify with ctest `device_api`, and that golden replay is identical.

## 2. Protocol commands (D2, D3)

- [x] 2.1 Codec: the error code `busy`, an encoder for `{"accepted":…,"state":…}`. `Connection::Kind()` for `BleConnection` (`phone`) and the socket client (`local client`). `PhoneServer`: `recording.start` / `recording.stop` in the table, the answers, the `busy` error, the log lines.
- [x] 2.2 Tests: `test_phone_server.cpp` (answers, `busy`, unknown arguments, `hello` listing both, log lines with a fake kind); `test_phone_socket.cpp` (a `recording.start` over the socket starts a `FakeSensor` recording, `recording.stop` saves it, and the events follow). Verify with ctest `phone_server` and `phone_socket`.
- [x] 2.3 `docs/phone-protocol.md`: both commands with examples (accepted, nothing to do, `busy`), `busy` in the error table, `hello`'s example listing them. Verify that ctest `protocol_codec` accepts every new example line. Update `openspec/changes/phone-link-core/app-questions.md` Q32 to "Binding".

## 3. Forget and the pairing window (D4, D6, D7)

- [x] 3.1 `BluezPeripheral::ForgetPhones()` (asynchronous `RemoveDevice` for every bonded device, then the window, printing `Bluetooth: forgot N phone(s)`). `PhoneLink::OpenPairingWindow(bool forget)`. `BleOptions::on_pairing`, called on open, close and BlueZ loss.
- [x] 3.2 `SIGUSR2` → `ForgetRequested()` in `signals.h`, polled by `main.cpp` next to `SIGUSR1`. `tools/pair.sh --forget` sends `SIGUSR2`. Verify on the dev board: with the phone bonded, `tools/pair.sh --forget` prints the forgotten count in the journal, the phone is disconnected, and it pairs anew within the window.

## 4. Pairing button and LED (D5, D6, D8)

- [x] 4.1 Settings `[pairing_button] gpio` (default `kPairingButtonGpio = 22`, distinct from `[button]` and `[led]`), `dlidar.toml`, the manual's settings table. Extend `tests/app/test_settings.cpp`. Verify with ctest `settings`.
- [x] 4.2 `src/app/pairing_button_commands.h` (`OnShort` → open, `OnLong` → forget + open, a warning without a phone link) with `tests/app/test_pairing_button_commands.cpp`. `main.cpp` opens the line (a warning on failure), a second `ButtonInput`, not in `--once`. Verify with ctest `pairing_button_commands`, and that dLidar starts with the line busy (a warning, then runs).
- [x] 4.3 `LedOn(state, ms, pairing, pairing_ms)` and `StatusLed::SetPairing(bool)`; `main.cpp` binds `on_pairing` to it. Extend `tests/adapters/gpio/test_status_led.cpp` (the pattern while idle, the priority of recording, saving and failure, the pattern back after the failure flashes). Verify with ctest `status_led`.

## 5. On the dev board

- [x] 5.1 With the service and no sensor: `tools/phone_client.py recording.start` gives `accepted:true` and `phone_client.py --follow` shows preparing → error. `recording.stop` while preparing gives `accepted:true` and no files. `recording.start` while preparing gives `accepted:false`. The journal shows the `local client: …` lines. With the phone: the same through the app once its agent offers start and stop.
- [ ] 5.2 **Parked** (user, 2026-11-03: the button and LED are mounted when the lidar arrives). Pairing button (when mounted, GPIO 22 / pin 15 to GND / pin 20):
  - a short press opens the window, and the LED flashes twice a second;
  - a 5 s hold forgets the phones and opens the window;
  - a press during a recording changes nothing in it;
  - the record button's power-down is unchanged.

  Until it is mounted, check the same through `tools/pair.sh` and `tools/pair.sh --forget`, the LED pattern included.

## 6. Docs and integration

- [x] 6.1 `docs/manual.html` (wiring: the pairing button on pin 15, GND on pin 20; gestures; the LED pattern table; start/stop from the phone), `docs/architecture.md` (the commands as an input channel, `PairingButtonCommands`, `SetPairing`), `README.md`, `CLAUDE.md`. Send the app agent the two commands and `busy`. Verify that the manual and architecture.md agree.
- [ ] 6.2 Full ctest, `tools/golden_replay.sh` (identical), `tools/check_architecture.py --strict`, the architecture-reviewer agent on the tip before the push.
