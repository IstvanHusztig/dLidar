# Proposal

## Why

The Android app (milestone 1 finished, tested on a Galaxy S25 Ultra) speaks protocol v1 over Bluetooth LE, but dLidar serves the protocol only on a local Unix socket (`phone-link-core`). Today the phone only finds `tools/ble_spike`, a throwaway echo service, and only while someone starts it by hand. The scanner must be visible to the phone and answer the real protocol whenever `bin/dLidar` runs, from boot on, so the user can see the scanner's state, recordings and warnings in the field.

## What Changes

- **BLE GATT transport for the phone link** (`src/adapters/phone/`): dLidar registers the GATT service the spike proved, with the same UUIDs:
  - service `b06b0001-1294-4457-85c4-ef6a46d0c127`;
  - RX `b06b0002-…` (write, write-without-response, encrypted only);
  - TX `b06b0003-…` (notify, encrypted only).

  It advertises the 128-bit service UUID in the advertisement packet itself, with the name `dLidar` in the scan response (where BlueZ puts it). It is a second transport on the existing phone loop, next to the socket, serving the same `PhoneServer`. The app needs no protocol change.
- **One phone at a time.** Advertising stops while a phone is connected, and a second phone that connects anyway is disconnected. Over BLE it does not get `too_many_clients`: in BlueZ's notification mode every subscribed phone would receive it.
- **Every request needs the bond** (user decision 2026-11-02). RX and TX require an encrypted link. An unbonded phone gets an ATT error and Android starts pairing, which succeeds only while the pairing window is open.
- **Paced notifications:** messages are split into MTU−3 pieces of the connection's own MTU, each message in a fresh notification. Output is sent at a measured rate with a per-connection cap, so a slow link backs up into the server's event holding, never into bluetoothd's memory.
- **The pairing window moves onto the phone loop and into the phone adapter.** `src/adapters/bluetooth/` is merged into `src/adapters/phone/`. Opening it no longer blocks dLidar's start (review note on `8e33edf`). It opens on request: the signal `SIGUSR1` (`tools/pair.sh`), which replaces the setting `[phone] pairing_at_start` (user decision 2026-11-03).
- **Robust to Bluetooth trouble:** no adapter, bluetoothd stopped or restarted, Bluetooth switched off. dLidar warns once, records as usual, and registers again when BlueZ comes back. Only asynchronous D-Bus calls run on the loop, so a hung bluetoothd stalls neither the socket clients nor dLidar's start.
- **Settings:** `[phone] ble` (default `true`) and `[phone] name` (default `"dLidar"`, 1–20 printable ASCII characters).
- **Service:** `systemd/dlidar.service` gets `Wants=`/`After=bluetooth.service` and `SupplementaryGroups=bluetooth`. It is installed and enabled at boot on the dev board, so the phone finds the scanner after power-on. The `ble-spike` unit is stopped and disabled, and `tools/ble_spike` deleted, only after the user confirms.
- **Measurement moved here from `phone-link-core` 1.2:** the notification throughput at MTU 517, which sets the pacing rate. The BLE link across a Wi-Fi switch of the Pi stays in phase 4 (`phone-link-wifi`).
- **Docs:** `docs/phone-protocol.md`, the BLE transport section (second phone disconnected, encrypted TX, MTU before the first write). Also `docs/manual.html`, `docs/architecture.md` and `CLAUDE.md`.

## Capabilities

### New Capabilities
- `phone-ble`: the phone link over Bluetooth LE. It covers advertising and discovery, the GATT service and its security, how protocol messages map onto writes and notifications, the one-phone rule, the pairing window, and behaviour when Bluetooth is missing or restarts.

### Modified Capabilities
<!-- None in openspec/specs: the protocol (`phone-api`) is still in the unarchived change `phone-link-core`, and this change does not alter it; the BLE-specific second-phone rule is stated in `phone-ble`. -->

## Impact

- **Code:**
  - New in `src/adapters/phone/`:
    - `ble_link` (pure state machine: connection, MTU, pacing, one-phone rule);
    - `bluez_peripheral` (D-Bus glue: GATT application, advertisement, agent, name watch);
  - `PairingWindow` moves into `src/adapters/phone/` and becomes asynchronous on the phone loop; `src/adapters/bluetooth/` and the `dlidar_bluetooth` target are removed.
  - `PhoneLink::Start` takes the BLE options.
  - `src/app/main.cpp`: no pairing thread; the window is opened through `PhoneLink`.
  - `src/app/settings.*`, `src/app/device_config.h`: the new keys.
  - No change to `src/domain` or `src/application`.
- **Dependencies:** none new (GIO is already used).
- **System:** BlueZ ≥ 5.66 (`bluetooth.service`). The Pi must not carry `dtoverlay=disable-bt` (hardware note from 2026-11-02).
- **Recordings and dMapper:** no effect. The recording files, their format and the console output of a recording stay byte-identical (golden replay). Nothing new goes into the output folder.
- **The app:** no protocol change. Over BLE a second phone is disconnected instead of receiving `too_many_clients` (correction of app question 26); TX needs the bond too.
