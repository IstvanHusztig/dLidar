# Proposal

## Why

The phone app can watch the scanner (`phone-link-core`, `phone-link-ble`) but not control it: a recording can only be started and stopped with the push button. In the field the phone is in the hand anyway, and the scanner may be mounted where the button is awkward to reach. Pairing still needs a shell on the Pi (`tools/pair.sh`), which a user in the field does not have; the roadmap foresaw a separate pairing button for that.

## What Changes

- **New commands `recording.start` and `recording.stop`** in protocol v1, for every connected client: bonded phones over BLE and local socket clients. Every phone that can connect is bonded, and bonds only come from the pairing window on the device, so no further permission is needed (user decision 2026-11-03).
  - They answer at once with `{"accepted":true|false,"state":…}`; the state events follow as for the button.
  - `accepted:false` means "nothing to do" (start while preparing or recording, stop while idle, error or finalizing); error `busy` means "not now" (start while the last recording is being saved). This is what app question 32 already promised.
  - `hello` lists the new commands. They are additions only: protocol v1 stays compatible.
  - The console and journal log each one (`phone: start`, `phone: stop`), as for the button.
- **A pairing button** on its own GPIO (default GPIO 22, header pin 15, to GND; `[pairing_button] gpio`). It reuses the push button's code (`GpioLine`, `PressDetector`):
  - a short press opens the 2-minute pairing window;
  - holding it 5 s forgets every bonded phone, then opens the window (user decision 2026-11-03).

  The button is optional: without it (not wired, line busy) dLidar warns and runs. The record button keeps its meaning; a 5 s hold there still saves and powers down.
- **Forgetting phones moves into dLidar.** The signal `SIGUSR2` (and `tools/pair.sh --forget`) removes the bonds and opens the window, the same path as the button's long press; `SIGUSR1` (`tools/pair.sh`) keeps opening it. Nothing outside dLidar has to call `bluetoothctl` any more.
- **The status LED shows an open pairing window**: two short flashes every second while it is open, unless a recording is running or being saved, which keep their patterns (user decision 2026-11-03).
- **Docs:** `docs/phone-protocol.md` (the commands, `busy`), the manual (pairing button wiring, gestures, LED), `docs/architecture.md`, `CLAUDE.md`. The app agent gets the commands.

## Capabilities

### New Capabilities
- `phone-commands`: the commands a connected client may give that change the device: starting and stopping a recording, their answers and errors, and how they are logged.
- `pairing-button`: the pairing button: wiring and setting, short and long press, forgetting bonded phones, and the LED pattern while the pairing window is open.

### Modified Capabilities
- `phone-ble`: the requirement "Pairing window" now names the pairing button and `SIGUSR2` (forget, then open) besides `SIGUSR1`.

## Impact

- **Code:**
  - `src/application/remote/device_api.h`: `Start()` and `Stop()` over `DeviceControl`, returning `{accepted, state}` or busy (planned in `phone-link-core` design D3).
  - `src/adapters/phone/`:
    - the two commands in `PhoneServer`'s table, with codec support for the `busy` error;
    - `BluezPeripheral::ForgetPhones()`;
    - `PhoneLink::OpenPairingWindow(forget)` and a pairing-window callback.
  - `src/adapters/gpio/`: `led_pattern.h` gets the pairing pattern; `StatusLed::SetPairing(bool)`.
  - `src/app/`: `PairingButtonCommands`; `main.cpp` wires the second button, `SIGUSR2` and the LED callback; settings `[pairing_button] gpio` with its default in `device_config.h`; `signals.h` (`SIGUSR2`).
  - `tools/pair.sh --forget` sends `SIGUSR2`.
  - No change to `src/domain`, `RecordSession` or `ScannerController`.
- **Recordings and dMapper:** no effect. A recording started from the phone is the same as one started with the button, and golden replay stays identical.
- **The app:** two new commands to offer (start and stop buttons, `busy` handling). Nothing existing changes.
- **Hardware:** a second push button between GPIO 22 and GND (optional until it is mounted; `tools/pair.sh` keeps working).
