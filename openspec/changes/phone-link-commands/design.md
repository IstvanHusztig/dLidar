# Design

## Context

See proposal.md (Why) and the specs for the requirements. What exists and shapes the design:

- **Control:**
  - `ScannerController` implements `DeviceControl` (`Status`, `StartRecording(state_out)`, `StopRecording`), thread-safe. `StartRecording` returns false unless Idle or Error and reports the state it saw; `StopRecording` only sets a flag.
  - `ButtonCommands` (`src/app/button_commands.h`) maps a record-button press to them by state and prints `button: …` lines.
- **The phone link** (`src/adapters/phone/`):
  - `PhoneServer` answers on the phone loop thread from a command table over `DeviceApi` (today `Status`, `Info`).
  - The codec knows the error codes `bad_request`, `unknown_command`, `unsupported_version`, `too_large`, `too_many_clients`. `busy` is reserved (`docs/phone-protocol.md`, app question 32).
  - `PhoneLink` starts everything; `OpenPairingWindow()` is thread-safe and goes through `BluezPeripheral` on the loop.
  - `tools/pair.sh --forget` removes the bonds with `bluetoothctl` before sending `SIGUSR1`.
- **GPIO** (`src/adapters/gpio/`):
  - `GpioLine::OpenButton(line)`, `ButtonInput(line, on_short, on_long)` with `PressDetector` (50 ms debounce, 5 s long press), on its own thread.
  - `StatusLed` (`hub.AddAsync`) drives the LED from `LedOn(state, ms)` in `led_pattern.h` every 50 ms.
- **Signals:** `SIGUSR1` sets `PairingRequested()`, which `main.cpp` polls every 100 ms (service mode and `--once`).
- **Pins:** GPIO 17 (record button, pin 11) and GPIO 27 (LED, pin 13) are taken. GPIO 22 (pin 15) is the next free general-purpose line. Pin 14 (GND) next to it already takes the LED, so its GND is pin 20.

## Goals / Non-Goals

**Goals:**
- A phone starts and stops recordings through the same `DeviceControl` path as the button, so a recording is the same whoever started it.
- One code path for "open the window" and one for "forget, then open", whatever triggers them: button, signal or script.
- Everything new is testable without hardware: the `DeviceApi` decisions, the server's answers, the LED pattern and the button mapping with fakes. The pairing button itself is checked once it is mounted.

**Non-Goals:**
- Per-phone permissions, an allow-list, or forgetting one phone (any bonded phone may command; forgetting removes all).
- Commands for Wi-Fi (phase 4), logs (phase 5), power-down from the phone, or recording options.
- Changing the record button or its 5 s power-down.

## Decisions

**D1. `DeviceApi::Start()` / `Stop()` decide; the adapter only encodes.**
- `Start()` returns `CommandResult{accepted, state_seen, busy}`:
  - `busy` when the state seen is `Finalizing`;
  - otherwise `accepted = StartRecording(&seen)`.

  The state check and the start are one call (`StartRecording` reports the state it saw atomically), so no race with the button.
- `Stop()`: `accepted` when the state seen is `Preparing` or `Recording`, then `StopRecording()`; otherwise `accepted:false`. `StopRecording` on the controller is a flag, so a stop racing a button press saves the recording once.
- Alternative: decide in `PhoneServer`. Rejected: the rule is device behaviour, testable with `FakeSensor` in `tests/application`, and the pairing button and a future input must not copy it.

**D2. The answer carries the state the command saw.**
- `{"accepted":true,"state":"idle"}` for a start from Idle; the events then say `preparing`.
- Alternative: the state after the command. Rejected: `StartRecording` does not report it atomically, and the events carry it anyway.
- `busy` is an error (`ok:false`, code `busy`, message "the last recording is still being saved; try again when it is idle"), as app question 32 announced.

**D3. Commands in the existing table, logged by the server.**
- `recording.start` and `recording.stop` (no arguments; unknown arguments are rejected as for every command) are entries in `PhoneServer`'s table. `hello` lists them automatically.
- Each prints one line on stdout: `phone: start`, `phone: start ignored, recording`, `phone: start refused, still saving`, and likewise for `stop`. `phone` is the client kind for BLE and `local client` for the socket.
- `Connection` gets `virtual const char *Kind() const`, implemented by `BleConnection` and the socket's `Client`.
- Alternative: log through `DeviceEvents`. Rejected: events describe the device, not who commanded it; the button logs the same way.

**D4. Forgetting lives in dLidar.**
- `BluezPeripheral::ForgetPhones()`:
  1. reads the bonded devices of the adapter (`GetManagedObjects`, Device1 `Paired`);
  2. calls `Adapter1.RemoveDevice` for each, asynchronously;
  3. when all have answered, prints `Bluetooth: forgot N phone(s)` and opens the window.
- `PhoneLink::OpenPairingWindow(bool forget)` is the one entry for the button, `SIGUSR1` and `SIGUSR2`.
- `tools/pair.sh --forget` sends `SIGUSR2` instead of calling `bluetoothctl`, so the forget follows the same path, and works with BlueZ's D-Bus policy for the service user.
- Removing a connected phone disconnects it, which `BleLink` already handles (`DeviceDisconnected`).
- Alternative: keep `bluetoothctl` in the script. Rejected: two code paths, and the pairing button needs it in-process anyway.

**D5. The pairing button reuses the GPIO adapter; the mapping is in the app.**
- `main.cpp` opens `GpioLine::OpenButton([pairing_button] gpio)`. On failure it prints a warning and continues (unlike the record button, which service mode needs).
- It creates a second `ButtonInput` with `PairingButtonCommands` (`src/app/pairing_button_commands.h`): `OnShort` → `open(false)`, `OnLong` → `open(true)`, through a `std::function<void(bool forget)>` bound to `PhoneLink::OpenPairingWindow`. With no phone link (another dLidar, BLE off) it prints a warning.
- `--once` does not open the pairing button, like the record button.
- No new adapter code: `ButtonInput` already runs on its own thread and calls callbacks.

**D6. The LED learns about the window from the phone link, wired in `main.cpp`.**
- `BleOptions` gets `std::function<void(bool open)> on_pairing`, called on the loop thread when the window really opens (after `Pairable` is true) and when it closes or BlueZ goes away.
- `main.cpp` keeps a `shared_ptr<StatusLed>` and binds `on_pairing` to `StatusLed::SetPairing(bool)`, a mutex-guarded flag plus its start time.
- `LedOn(state, ms, pairing, pairing_ms)`:
  - Preparing and Recording: on;
  - Finalizing: 2 Hz;
  - Error: its three flashes;
  - otherwise, while pairing: on when `pairing_ms % 1000` is in [0, 100) or [200, 300).
- Alternative: a `DeviceEvents::OnPairing`. Rejected: pairing is not device state, and the core would learn about Bluetooth.

**D7. `SIGUSR2` asks for forget-then-open.**
- `ForgetRequested()` in `signals.h`, polled next to `PairingRequested()`.
- The settings keep no pairing key.

**D8. Settings.**
- `[pairing_button] gpio` (0–27, default `kPairingButtonGpio = 22` in `device_config.h`). It must differ from `[button] gpio` and `[led] gpio`, and the error names the line.
- `dlidar.toml` lists it, and the manual's settings table and wiring section describe it (pin 15, GND on pin 20).

**D9. Tests.**
- `tests/application/test_device_api.cpp`, with a fake `DeviceControl` and with `ScannerController` + `FakeSensor`:
  - start from Idle and from Error;
  - start while Preparing and Recording (`accepted:false`);
  - start while Finalizing (`busy`);
  - stop in each state;
  - a stop during Preparing writes no files.
- `test_phone_server.cpp`: both commands' answers and the `busy` error shape, unknown arguments rejected, `hello` listing them, the log lines with a fake `Kind()`.
- `docs/phone-protocol.md`: examples for both commands and `busy`. `test_protocol_codec` checks every example line.
- `tests/adapters/gpio/test_status_led.cpp` (`LedOn`): the pairing pattern while idle, the priority of the recording, saving and failure patterns, and the pattern back after the failure flashes.
- `tests/app/test_pairing_button_commands.cpp`: short → open, long → forget + open, no phone link → warning.
- `test_settings`: the default, a valid line, the same line as the button or the LED, out of range.
- On the dev board:
  - `tools/phone_client.py recording.start` / `recording.stop` against the service, in service mode with no sensor (preparing → error) and with `--once`;
  - `tools/pair.sh --forget` with the phone;
  - the pairing button and the LED pattern once the button is mounted;
  - the app's start/stop once its agent has added them.

## Risks / Trade-offs

- **[A stray command stops a field recording]** → only bonded phones and local clients can send it, and bonds need the device in hand (button or shell). The stop saves normally, so nothing is lost.
- **[Two phones?]** → one phone at a time over BLE (`phone-ble`); a local client is the user on the Pi. Commands are serialized through `ScannerController`.
- **[The pairing button not mounted yet]** → it is optional; `tools/pair.sh` keeps working, and the hardware check waits like `push-button-recording` 7.x.
- **[Forgetting removes every bond, the owner's too]** → intended (one owner per scanner); the window opens right after, so the owner re-pairs in the same 2 minutes.
- **[`RemoveDevice` on a connected phone]** → BlueZ disconnects it first; `BleLink` sees the disconnect and advertises again.

## Migration Plan

- Rebuild and restart the service. The new setting has a default; `dlidar.toml` gains `[pairing_button]`.
- Wire the second button whenever available; until then nothing changes except the new commands.
- Rollback: the previous binary. Protocol v1 only gained commands, so a newer app degrades to "unknown command" on an older scanner.
