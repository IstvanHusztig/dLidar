# Tasks

From group 2 on, after every group, run on the Pi 5:
- `cmake --build build`
- `ctest --test-dir build --output-on-failure` (including `architecture`)
- `tools/golden_replay.sh`, which must stay identical

## 1. BLE spike (throwaway, design D1)

- [x] 1.1 `tools/ble_spike/` (CMake option `DLIDAR_BLE_SPIKE`, off by default): a GIO program that registers an LE advertisement and one GATT service with an RX characteristic (write, write-without-response) and a TX characteristic (notify), and echoes each received line. Enable `bluetooth.service` on the dev board. Verify that nRF Connect on Android sees the device, writes a line and receives the echo.
- [x] 1.2 Measure with the spike, running as the service user under systemd:
  - MTU and the largest notification;
  - throughput;
  - two centrals with `PropertiesChanged` versus `AcquireNotify`;
  - reconnect after leaving range;
  - bonding;
  - the BLE link while the Pi switches to another Wi-Fi network (`nmcli connection up`; no scan is ever made by the Pi).

  Write the results into design.md "Spike Findings", with any change to D2/D4 they force (and, if one does, to the spec). Verify that the findings section answers each point.

## 2. Dependencies, interfaces and API

- [x] 2.1 Add nlohmann/json v3.11.3 as submodule `external/json` (an INTERFACE include target). `pkg_check_modules` for `gio-2.0 gio-unix-2.0`. Add the `dlidar_phone` target. Add the version header generated at build time (D7). Update `CLAUDE.md` (Build & Run: submodules, `libglib2.0-dev`). Verify that a clean configure and build succeed, and that touching a commit changes `dlidar_version.h` without a reconfigure.
- [x] 2.2 Ports `device_control.h` (`ScannerController` implements it) and `storage_space.h` (`FileRecordingStorage` implements it: `UINT64_MAX` becomes empty). Verify that the existing ctests pass unchanged and that golden replay is identical.
- [x] 2.3 `src/application/remote/device_api.h` (`DeviceApi`, `DeviceInfo`, `StatusView`, `kProtocolVersion`) and `tests/application/test_device_api.cpp` (status in idle/recording/error with `last_ok`, `free_bytes` known and unknown, the info fields). Verify with ctest `device_api`.

## 3. Codec and framing

- [x] 3.1 `src/adapters/phone/line_framer.h` and `tests/adapters/phone/test_line_framer.cpp`:
  - a line split across 3 feeds;
  - 2 lines in one feed;
  - exactly 4096 bytes accepted, 4097 reported `too_large` once;
  - the next line is recovered;
  - reset.

  Verify with ctest `line_framer`.
- [x] 3.2 `src/adapters/phone/protocol_codec.{h,cpp}`: request decoding (no exceptions; `v`/`id`/`cmd` types; id range; unknown args per command), the encoders for responses, errors (with `supported`) and events (`seq`, `dropped`, `op`), and the enum string tables. Add `tests/adapters/phone/test_protocol_codec.cpp` covering every error path, UTF-8, no raw newline, every `WarningKind` named, and every output under 16384 bytes. Verify with ctest `protocol_codec`.
- [x] 3.3 Write `docs/phone-protocol.md` v1 from the spec and D2:
  - framing and limits, the three message shapes;
  - ordering, `seq`/`dropped`, results that come later (`op`);
  - error codes;
  - every command and event with one example;
  - evolution rules for clients;
  - "call `status.get` after connecting";
  - the transports (the socket now, BLE next).
  - the answers in `app-questions.md` (the app team's questions, 2026-11-02).

  Verify that every example line is accepted by the codec tests (paste them in as test vectors), and that the field names match the spec.

## 4. Loop, server, events and socket

- [x] 4.1 `Connection` interface, `PhoneServer` (command table, `seq`, `last_status`, progress timer), `Mailbox` and `PhoneEvents` (D4, D5). Add `tests/adapters/phone/test_phone_server.cpp` with a fake `Connection` and a fake clock:
  - order and `seq`;
  - progress at most 1 Hz and flushed before `finalizing`;
  - `dropped`, with the latest state kept;
  - `status.get` never older than the events sent;
  - pipelined responses in order.

  Verify with ctest `phone_server`.
- [x] 4.2 `PhoneLoop` (a private `GMainContext` thread, invoke, flush up to 200 ms, stop) and `SocketTransport` (D4, D6):
  - the path lookup without `/tmp`;
  - leftover-file probing, then bind → chmod 0600 → listen;
  - non-blocking I/O with a 64 KiB cap per connection;
  - 4 clients, `too_many_clients` for a fifth;
  - unlink on close.

  Add `tests/adapters/phone/test_phone_socket.cpp` covering the D10 list. Verify with ctest `phone_socket`, and that the test run leaves no thread or socket file behind.

## 5. Wiring, tools and docs

- [ ] 5.1 `src/app/main.cpp`: build `DeviceInfo` from `ProfileFor(settings)`, `kRecordingFormatVersion` and the version header, then wire everything inside the controller block (D6), with a warning on failure. Verify on the dev board:
  - the console output of `bin/dLidar --once` without a sensor is byte-identical to the previous build's;
  - in service mode, `/run/dlidar/dlidar.sock` (or `$XDG_RUNTIME_DIR/dlidar.sock` when run by hand) exists with mode 0600 and is gone after a clean exit;
  - a second dLidar warns and leaves the first one's socket alone.
- [ ] 5.2 `tools/phone_client.py` (D9). Verify against `bin/dLidar` in service mode:
  - `hello`, `info.get` and `status.get` print their responses;
  - `--follow` shows preparing → … → idle or error for a button-started recording;
  - with `--once`, `--wait 5 --follow` shows the warning and the error of a start without a sensor.
- [ ] 5.3 Docs:
  - `docs/architecture.md`: the phone link as an input and output channel example; `src/application/remote/`, the new ports and `src/adapters/phone/` in "What lives where"; the GLib loop in the data flow; Deferred updated (the BLE transport still to come);
  - `docs/manual.html`: the socket, `tools/phone_client.py`, `libglib2.0-dev`;
  - `CLAUDE.md`: an Architecture bullet for the phone adapter.

  Verify that the manual and architecture.md agree, and that `tools/check_architecture.py --strict src` passes.

## 6. Integration

- [ ] 6.1 Run the full ctest suite, `tools/golden_replay.sh` (identical), and the `--once` console comparison from 5.1. Run the architecture-reviewer agent on the branch. Verify that everything passes and the reviewer reports PASS.

## Workflow follow-up

- Propose `phone-link-ble` (phase 2) after this change is merged. It can merge into this change if the spike shows the BLE transport is small.
- Archive this change after review.
