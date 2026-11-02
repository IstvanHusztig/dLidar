# Design

## Context

See proposal.md (Why) and specs/phone-api/spec.md for the requirements. Facts from the code and the Pi that shape the design:

**Control and events**
- `ScannerController` (`src/application/device/scanner_controller.h`):
  - `Status()` is a mutex copy;
  - `StartRecording` and `StopRecording` return at once;
  - it lives in an inner block of `main.cpp` (`src/app/main.cpp:155-176`), after the hub, which outlives it.
- In service mode the loop calls `StopRecording` + `WaitIdle` before that block ends, so a SIGTERM's finalization (up to about 20 min) completes inside the block.
- `EventHub::AddAsync` gives a channel its own thread and a 64-event drop-oldest queue (`src/application/device/event_hub.h`). No event is published before the first `StartRecording`, so channels can be added inside the controller block.
- A chunk is at most 14000 points (`chunk_assembler.h:19`). On the Mid-360S that means about 14 `OnProgress` calls per second.

**Free space and sensor identity**
- `FileRecordingStorage` computes free space with a statvfs lambda (`UINT64_MAX` = unknown). The `RecordingStorage` port has no free-space query.
- Sensor identity: `ProfileFor(settings)` (`src/app/sensor_factory.h`) gives the profile:
  - the id: `unitree_l2`, `livox_mid360s`;
  - `sensor_model`: `Unitree L2`, `Livox Mid-360S`.
- `kRecordingFormatVersion` is in `src/domain/recording/recording_info.h:17`. The build has no version string.

**Pi and BlueZ**
- The service has `RuntimeDirectory=dlidar` (`/run/dlidar`, which is `$RUNTIME_DIRECTORY`). An SSH user's `$XDG_RUNTIME_DIR` is `/run/user/UID`, a different place.
- `libglib2.0-dev` (2.74) is installed. On the dev board BlueZ's D-Bus policy already allows any user `send_destination="org.bluez"`. `bluetooth.service` is inactive there.
- BlueZ's D-Bus GATT server has two ways to send notifications:
  - `PropertiesChanged` on `Value`, which goes to every subscribed central;
  - `AcquireNotify`, which gives an fd.

  Writes arrive as `WriteValue(value, {device, mtu, offset, type})` calls, not as a byte stream on an fd. This is what the spike (D1) measures.
- `tools/check_architecture.py` allows any `adapters/<x>/` with no change.

## Goals / Non-Goals

**Goals:**
- One event loop and one connection abstraction that the BLE transport of the next phase plugs into without restructuring.
- The protocol document is frozen only after the BLE spike has confirmed its assumptions.
- The whole path is testable without a phone: unit tests plus a socket end-to-end test.
- A client can never hold up a recording, and memory per client is bounded.

**Non-Goals:**
- The production BLE transport, pairing, and authentication. The socket is protected by its mode and its directory.
- Commands that change state, logs, Wi-Fi.
- A general RPC framework.

## Decisions

**D1. A BLE spike comes first, and is throwaway.**
- `tools/ble_spike/` is a standalone GIO program, built only with `-DDLIDAR_BLE_SPIKE=ON` and never linked into dLidar. It has:
  - an LE advertisement;
  - one service with an RX characteristic (write, write-without-response) and a TX characteristic (notify) that echoes lines.
- Run it as the service user under systemd and measure with nRF Connect:
  - the negotiated MTU and largest notification;
  - notification throughput, and the cost of a 300-byte message at MTU 23 and at MTU 247;
  - what two connected centrals receive with `PropertiesChanged` and with `AcquireNotify`;
  - reconnect after the phone leaves range;
  - what happens to bonding;
  - whether the BLE link survives the Pi switching Wi-Fi networks (`nmcli connection up`). Wi-Fi and BLE share one radio. The Pi never scans for networks: the phone sends the SSID and password.
- **Outcome:** a "Spike findings" section appended to this design. Then the protocol document is frozen.
- **Already decided, unless the spike disproves it:**
  - one BLE central at a time (advertising stops while connected);
  - a message always starts at a fresh notification.
- Alternative: build the socket core first and find out about BlueZ in phase 2. Rejected by the review: the protocol would be frozen before the riskiest part was known.

**D2. Protocol v1 (the contract is `docs/phone-protocol.md`).**
- **Request/response with client ids**, plus events with a per-connection `seq` and `dropped`. The spec gives the message shapes.
- **Results that come later** (needed by `wifi.connect` in phase 4) are defined now and unused in v1. The command answers `{"ok":true,"result":{"accepted":true}}`, and a later `result` event carries `"op":<request id>` at top level and `ok` plus `result`/`error` like a response. It goes only to the asking connection and is never coalesced away.
- **`status.get` also returns `warnings` and `summary`** (the mailbox already keeps both), so an app that reconnects can recover them (app question 16, 17).
- **Error codes:** `bad_request`, `unknown_command`, `unsupported_version`, `too_large` and `too_many_clients`. Later codes include `busy` for "device busy", in phase 3.
- **Rejected alternatives:**
  - JSON-RPC 2.0 verbatim: more bytes per BLE message, numeric error codes, no gain for the app.
  - CBOR or protobuf: a schema toolchain on both sides, and harder to debug with nRF Connect and `socat`.
  - One typed GATT characteristic per field: no replies, a new UUID per field, and Android caches GATT tables.
  - A length prefix: it never gets back in step after a lost fragment, while a newline does.

**D3. The application layer has typed methods and narrow interfaces. JSON exists only in the adapter.**
- `src/application/ports/device_control.h`: `DeviceControl { Status(); StartRecording(DeviceState*); StopRecording(); }`. `ScannerController` implements it publicly. Same signatures, no behaviour change.
- `src/application/ports/storage_space.h`: `StorageSpace { std::optional<uint64_t> FreeBytes() const; }`. `FileRecordingStorage` implements it with its existing lambda, mapping `UINT64_MAX` to empty. statvfs on a fixed directory is thread-safe.
- `src/application/remote/device_api.h`:
  - `DeviceApi(DeviceControl&, const StorageSpace*, DeviceInfo)` with `Status() -> StatusView` (`DeviceStatus` + `free_bytes`) and `Info() -> DeviceInfo`.
  - `DeviceInfo` holds `version`, `sensor`, `sensor_model` and `recording_format`; `main.cpp` fills it from `ProfileFor(settings)`.
  - `kProtocolVersion = 1`.
  - Phase 3 adds `Start()` and `Stop()` returning `{accepted, state}`.
- **The adapter's command table** maps a name to a handler: decode and validate the args (unknown names rejected), call the `DeviceApi` method, encode the result. Adding a command means one table entry and one method.
- Alternative: a `Command` enum with `Request`/`Response` variants in the application layer. Rejected: every command would touch five places, for no testability gain.

**D4. One GLib main loop thread owns the phone link (`src/adapters/phone/`, CMake `dlidar_phone`, private links to `nlohmann_json` and `gio-2.0` through pkg-config).**
- **`PhoneLoop`:**
  - a private `GMainContext` on one thread, started and stopped by the adapter;
  - stop: a quit and a join, after the flush in D6.
- **`Connection`, push-based:**
  - The server calls `Send(bytes)` and `Close()`.
  - The transport calls the server's `OnOpen(conn)`, `OnBytes(conn, data)` and `OnClosed(conn)`, all on the loop thread.
  - The Unix socket implements it in this change, and the BLE characteristic pair implements it in phase 2. There is no fd in the interface.
- **`PhoneServer` (on the loop), for each connection:**
  - A `LineFramer`, reset when the connection opens.
  - Handlers run inline. They are cheap: a mutex copy, statvfs, a struct copy.
  - Each response is encoded and `Send`.
  - Events carry a `seq` per connection.
- **`SocketTransport`:**
  - non-blocking `AF_UNIX` fds watched with `g_unix_fd_add`;
  - a write buffer per connection, capped at 64 KiB. Past the cap the connection is closed (spec: bounded per client), and responses are never dropped silently;
  - 4 connections at most; a fifth gets `too_many_clients` and is closed.
- **Pure parts, unit tested without GLib:**
  - `protocol_codec.{h,cpp}`: nlohmann only here; parsed without exceptions (`json::parse(s, nullptr, false)`) after the size check.
  - `line_framer.h`.
- Alternative: two blocking threads per client. Rejected:
  - phase 2 needs this loop anyway;
  - blocking `send` makes bounded memory and teardown hard;
  - runtime cost is not the reason (both are negligible on 2 GB).

**D5. Events: hub → coalescing mailbox → loop.**
- `PhoneEvents : DeviceEvents` is registered with `hub.AddAsync`, so the recording threads only touch the hub queue.
- It writes into a `Mailbox` (mutex) and wakes the loop with one `g_main_context_invoke` if none is pending. The mailbox holds:
  - the latest `DeviceStatus` from `OnState`, plus an ordered list of state transitions, at most 8;
  - the latest progress;
  - the last 16 warnings;
  - the last summary;
  - and counts what it dropped.
- **On the loop:**
  - `PhoneServer` takes the mailbox, updates its `last_status`, and sends to each connection in order: states, warnings, summary.
  - Progress is sent at most once per second per connection, by a 1 s timer that fires when there is new progress. A pending progress is always sent before a `finalizing` state.
  - `dropped` goes on the next event.
- **`status.get`** answers from `last_status`: the latest state and progress the loop has seen, plus live `free_bytes`. It is never older than events already sent, because both are produced on the loop in order (spec "Status never older…").
  - `last_status` starts from `DeviceControl::Status()` when the server is created.
  - It may lag the controller by milliseconds. Accepted: it is consistent with the event stream.

**D6. Wiring and lifetime in `main.cpp`.**
- **Inside the controller block, after the controller:**
  1. build `DeviceInfo` and `DeviceApi`;
  2. create `PhoneServer` + `SocketTransport` + `PhoneLoop`;
  3. `hub.AddAsync(phone_events)`;
  4. start the loop. A failed socket prints `WARNING: no phone socket (<reason>)`, and dLidar continues.
- **On leaving the block, after `WaitIdle`** (service mode and `--once` both wait):
  1. flush: the loop runs up to 200 ms until the mailbox and the write buffers are empty;
  2. close the connections;
  3. unlink the socket;
  4. stop the loop.
- The hub may still deliver to `PhoneEvents` after the loop stops: `PhoneEvents` holds the mailbox by `shared_ptr`, and the mailbox ignores input once closed.
- **Socket path:** `$RUNTIME_DIRECTORY`, else `$XDG_RUNTIME_DIR`, else no socket. There is no `/tmp` fallback, because another user could squat on the path. The steps:
  1. a leftover file: try `connect`. If something answers, warn "another dLidar is running" and skip; otherwise `unlink`;
  2. `bind`;
  3. `chmod(path, 0600)`;
  4. `listen`. A connect before `listen` fails, so there is no window with the wrong mode.
- The replay tools do not wire the phone link. On success the socket prints nothing.

**D7. Version string generated at build time.**
- A CMake custom target runs a `cmake -P` script that writes `build/generated/dlidar_version.h` (`git describe --always --dirty`, `"unknown"` without git). The file is rewritten only when the text changed.
- Only `main.cpp` includes it.
- Alternative: configure time. Rejected: it goes stale with Ninja.

**D8. Dependencies.**
- nlohmann/json v3.11.3 as submodule `external/json`, used as an `INTERFACE` include target (no install, no tests). Alternative: the Debian package. Rejected: not pinned, and other submodules set the precedent.
- GIO through `pkg_check_modules(GIO REQUIRED gio-2.0 gio-unix-2.0)`, linked only into `dlidar_phone`.

**D9. `tools/phone_client.py` (standard library only).**
- `phone_client.py [--socket PATH] [--wait S] CMD [JSON_ARGS]` prints the response. `--follow` prints events.
- It looks for the socket in `/run/dlidar/dlidar.sock`, then `$XDG_RUNTIME_DIR/dlidar.sock`.
- `--wait` retries the connect for S seconds, for a dLidar that is just starting.

**D10. Tests.**
- `tests/application/test_device_api.cpp`: status in each state (through `ScannerController` with `FakeSensor`/`MemoryStorage`), `free_bytes` known and unknown (a fake `StorageSpace`), the info fields.
- `tests/adapters/phone/test_protocol_codec.cpp`: shapes, every error path, unknown args, the id range, `"v":2` with `supported`, UTF-8 text, no raw newline, every `WarningKind` named.
- `tests/adapters/phone/test_line_framer.cpp`: lines split and merged, 4096 versus 4097, recovery after `too_large`.
- `tests/adapters/phone/test_phone_server.cpp`: `PhoneServer` + `Mailbox` with a fake `Connection` and a fake clock, no sockets. It checks:
  - `seq` and order;
  - progress at most 1 Hz, flushed before `finalizing`;
  - `dropped` set;
  - the latest state kept;
  - `status.get` never older than the events sent.
- `tests/adapters/phone/test_phone_socket.cpp`: the real loop and socket in a temp directory used as `$RUNTIME_DIRECTORY`. It checks:
  - pipelined requests;
  - events during a `FakeSensor` recording;
  - a fifth client gets `too_many_clients`;
  - a client that sends but never reads is disconnected at the cap, while another client gets everything and the `MemoryStorage` contents equal a run without clients;
  - a disconnect mid-line;
  - a leftover file is replaced, and a live socket is left in place;
  - the flush at shutdown delivers `idle`.

## Risks / Trade-offs

- **[The spike shows notifications cannot be addressed or are too slow]** → one central at a time is already assumed. If throughput is poor, phase 2 negotiates a larger MTU and the protocol keeps its messages small. Decided before the protocol doc is frozen.
- **[GLib in the process]** → it is confined to `dlidar_phone` and runs on its own `GMainContext`, never the default one, so it cannot interfere with other code.
- **[`last_status` lags the controller by milliseconds]** → this is the price of the ordering guarantee. `free_bytes` stays live.
- **[A client disconnected at the 64 KiB cap]** → it only happens to a client that does not read. The app reconnects and calls `status.get`.
- **[Just Works pairing later has no MITM protection]** → out of scope here. Phase 3 opens a pairing window with a separate pairing button and keeps an allow-list (roadmap).
- **[Golden replay does not exercise the phone link]** → it stays a regression gate. The isolation evidence is the socket test, where a stalled client leaves the `MemoryStorage` contents identical, plus a manual check that the console output of `bin/dLidar --once` is unchanged.

## Migration Plan

- This is purely additive.
- **Deploy:** `git submodule update --init --recursive` (new `external/json`), rebuild, restart the service. The socket appears at `/run/dlidar/dlidar.sock`.
- **Rollback:** the previous binary. Nothing persistent is written.

## Spike Findings

Measured 2026-11-02 on the dev Pi 5 (BlueZ, `bluetooth.service`) with a Galaxy S25 Ultra (Android 15), nRF Connect and the app's milestone-1 build; spike run as user `vs2022` under systemd (`ble-spike` unit).

1. **MTU:** the phone asks for 517 and BlueZ accepts it; `WriteValue` reports `mtu=517`. Splitting by MTU-3 of the connection works; the app writes pieces of up to 514 bytes.
2. **Notifications:** `PropertiesChanged` on `Value` (the default mode) delivers every echo; `AcquireNotify` was not needed. Phase 2 keeps `PropertiesChanged`.
3. **Two centrals:** not measured (the user has one phone). The one-central rule stands: advertising stops while a phone is connected, and a second connection gets `too_many_clients`; phone-link-ble tests that with a fake second connection.
4. **Reconnect:** 10+ disconnects and reconnects in a row (app restarts, Bluetooth off/on) between 22:10 and 22:48; each time `Connected` went false/true, the phone turned on notifications again (`StartNotify`) and echoes worked, with no spike restart.
5. **Bonding:** with `--secure` (RX `encrypt-write`) the phone paired Just Works on its first write while the pairing window was open (`[phone] pairing_at_start`), and stays `Paired`/`Bonded` across reconnects without a new prompt. The adapter's default is `Pairable: yes`; dLidar's pairing window sets it false when it closes, and BlueZ may set it true again when the default agent changes, so the window sets it false after unregistering its agent. Decision (user): every request needs the bond (RX `encrypt-write`).
6. **Wi-Fi switch:** moved to `phone-link-wifi` (phase 4), where the Pi switches networks; it does not affect the protocol.
7. **Throughput (`flood`):** moved to `phone-link-ble`, measured with the real transport; v1 messages are small (below 1 KiB), so it does not affect the protocol.

Consequences for the protocol: none. The framing (a message starts in a fresh notification, MTU-3 pieces), one central at a time and the 4096/16384 limits stand.
