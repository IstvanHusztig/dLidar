# Proposal

## Why

The handheld scanner is to be operated from an Android phone over Bluetooth LE. The phone shows status, information, logs and errors, then starts and stops recordings and sets the Pi's Wi-Fi. Today the only ways in and out are the button, the LED and the console over SSH.

Before the app can be built, dLidar needs three things:
- a fixed **API** the app developer builds against;
- an event loop that every transport shares;
- proof that BlueZ on the Pi 5 behaves the way the protocol assumes.

This change is the first step of the phased plan: spike and core, then BLE, then commands with pairing security, then Wi-Fi, then logs. It is read-only, so it cannot disturb a recording.

## What Changes

- **BLE spike (throwaway):** a minimal GATT echo service with advertising, used to measure MTU, notification behaviour with two phones, reconnects and whether the link survives a Wi-Fi network switch before the protocol is frozen. Its findings go into design.md. The spike code is not kept.
- **Phone protocol v1 (`docs/phone-protocol.md`):** the contract for the Android app.
  - **Framing:** newline-delimited compact JSON.
  - **request** `{"v":1,"id":N,"cmd":"…","args":{…}}`.
  - **response** `{"id":N,"ok":true,"result":{…}}` or `{"id":N,"ok":false,"error":{"code":"…","message":"…"}}`. There is exactly one per request, in request order.
  - **event** `{"event":"…","seq":N,"data":{…}}`, with a `dropped` count when events were left out.
  - **Rules:**
    - commands, events, fields and codes are only ever added;
    - clients ignore what they do not know;
    - unknown `args` are rejected;
    - for operations whose result comes later, an event names the request (`op`).
  - It is transport-independent: the same bytes go over BLE (next phase) and over the local socket.
- **Read-only commands:**
  - `hello`: protocol version, command and event lists, limits;
  - `status.get`: state, chunks and points, whether the last recording was saved, output directory, free space;
  - `info.get`: dLidar version, sensor model, recording format version, protocol version.
- **Events pushed to every connected client:**
  - `state` on each state change;
  - `warning` with kind and text;
  - `progress` at most once per second;
  - `summary`, the IMU timing summary at the end of a recording.
- **One event loop:** a single thread runs the phone link (GLib main loop). Every transport runs on it, so the BLE transport of the next phase plugs into the same loop.
- **Local socket transport:** `dlidar.sock` (mode 0600) in the service's runtime directory `/run/dlidar`, else `$XDG_RUNTIME_DIR`. It is used for tests and for debugging over SSH. Without a usable directory it prints a warning and dLidar runs on.
- **`tools/phone_client.py`:** send a command, print the response, or follow events.
- **New dependencies:**
  - nlohmann/json as a git submodule in `external/`;
  - GLib/GIO (system packages, already installed);
  - both used only by the new phone adapter.

## Capabilities

### New Capabilities
- `phone-api`: the message protocol between dLidar and a phone app. It covers:
  - message shapes, ordering and versioning;
  - the commands and their results, the events, the error codes and limits;
  - how clients connect and are kept from affecting recordings.

  Later phases add commands, transports and security to this capability.

### Modified Capabilities
None. The phone channel uses the existing `device-control` contracts (control interface, status delivered to every output channel) without changing them.

## Impact

- **Code:**
  - New `src/application/remote/`: `DeviceApi` with typed methods over narrow `DeviceControl` and `StorageSpace` interfaces.
  - New `src/adapters/phone/` (CMake `dlidar_phone`): codec, framing, the loop server, the event mailbox, the socket transport.
  - `ScannerController` implements `DeviceControl`, and `FileRecordingStorage` implements `StorageSpace`. Recording behaviour is unchanged.
  - Wiring in `src/app/main.cpp`.
- **Dependencies:** `external/json` (nlohmann/json, pinned), and `libglib2.0-dev` for the build (installed on the dev board).
- **Output and dMapper:** none. Recording files and the console output are unchanged.
- **Runtime:**
  - one extra thread (the loop) and small bounded buffers;
  - at most 4 socket clients, a 4 KB request limit, a byte cap on each client's output;
  - negligible on a Pi 5 with 2 GB.
- **Out of scope (later phases):**
  - the BLE transport (`phone-link-ble`);
  - start/stop with pairing security (`phone-link-commands`);
  - Wi-Fi setup (`phone-link-wifi`);
  - logs (`phone-link-logs`).
