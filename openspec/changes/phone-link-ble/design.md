# Design

## Context

See proposal.md (Why) and specs/phone-ble/spec.md for the requirements. Facts that shape the design:

**The phone link today (`phone-link-core`)**
- `PhoneLoop` runs a private `GMainContext` on its own thread. `PhoneServer` (a `ConnectionHandler`) serves every `Connection` on it. `SocketTransport` is the only transport.
- The server holds events for a connection whose `Pending()` exceeds 8 KiB, and the transport closes it past its cap (64 KiB for the socket).
- `PhoneLink::Start` starts all of it. It fails as a whole when another dLidar answers on the socket.

**The pairing window**
- `PairingWindow` (`src/adapters/bluetooth/`) has its own thread and context.
- It makes synchronous BlueZ calls with 5 s timeouts, and `main.cpp` waits for its start-up. The review of `8e33edf` (note 6) flagged that a hung bluetoothd delays dLidar's start.
- It sets `Pairable` false only after unregistering its agent, because BlueZ may set the adapter pairable again when the default agent changes.

**BlueZ on the dev board (5.66, Debian/RPi build)**
- The D-Bus policy allows any user `send_destination="org.bluez"` (`<policy context="default">`). Upstream BlueZ allows only root and group `bluetooth` (gid 108 exists here; the service user is not in it).
- The adapter's default is `Pairable: yes`: any phone could pair while dLidar does not manage it.
- `ExchangeMTU` is the default, 517.
- Server characteristic flags include `encrypt-write` and `encrypt-notify` (server only).
- `WriteValue` options carry `device`, `mtu`, `offset` and `type`. `StartNotify`/`StopNotify` carry no device. `AcquireNotify` carries `device` and `mtu`.

**Spike findings (`phone-link-core` design)**
- MTU 517 is accepted.
- `PropertiesChanged` notifications reach the phone.
- More than 10 reconnects worked without a restart.
- Bonding through the pairing window works and survives reconnects.
- Not measured: two centrals, throughput.

**Architecture rules**
- An adapter may not include another adapter (`tools/check_architecture.py`).
- The BLE transport implements `phone::Connection`, so it must live in `src/adapters/phone/`.

## Goals / Non-Goals

**Goals:**
- The BLE logic (connection identity, MTU, splitting, pacing, the one-phone rule, advertising on/off) is a pure state machine, unit-tested with a fake peripheral and a fake clock. The D-Bus glue is thin and only translates.
- Nothing on the phone loop blocks: every BlueZ call is asynchronous, so a hung bluetoothd affects neither the socket clients nor dLidar's start.
- One owner of everything BlueZ (GATT application, advertisement, agent, pairable state), on the phone loop.

**Non-Goals:**
- Commands that change the device, the bond allow-list and the pairing button (phase 3). Wi-Fi, and BLE during a Pi Wi-Fi switch (phase 4).
- Several phones at once, BLE privacy (resolvable private addresses), long writes (prepared writes with an offset).
- A D-Bus-level test harness: the glue is verified on the dev board with the phone and `btmon`.

## Decisions

**D1. One phone adapter owns BlueZ: `src/adapters/bluetooth/` merges into `src/adapters/phone/`.**
- New files:
  - `ble_link.{h,cpp}`: the pure state machine (D2) and its `BleConnection`, the `Connection` the server sees;
  - `bluez_peripheral.{h,cpp}`: the D-Bus glue for the GATT application, the advertisement, the agent, the adapter's `Pairable`, the `org.bluez` name watch and the Device1 signals;
- `PairingWindow` becomes part of `bluez_peripheral`: the agent and `Pairable` belong to the same BlueZ session.
- The `dlidar_bluetooth` target is removed.
- Alternative: keep a `bluetooth/` adapter and let `main.cpp` connect the two through a callback interface. Rejected: two owners of one D-Bus session, two threads, and the cross-adapter rule would force a port into the application layer for pure plumbing.

**D2. `BleLink`: a pure state machine behind a `BlePeripheral` port.**
- **Inputs**, from the glue on the loop thread:
  - `DeviceConnected(dev)`, `DeviceDisconnected(dev)`;
  - `NotifyStarted()`, `NotifyStopped()`;
  - `Write(dev, bytes, mtu, offset)`, which returns accept or a reject reason;
  - `BluezUp()`, `BluezDown()`;
  - `Tick(now)`.
- **Outputs**, through `BlePeripheral`:
  - `Notify(piece)`;
  - `Disconnect(dev)`;
  - `SetAdvertising(bool)`;
  - `WakeAt(t)`.
- **Toward the server**, as `ConnectionHandler` calls on its `BleConnection`: `OnOpen`, `OnBytes`, `OnWritable`, `OnClosed`.
- Unit tests drive it with a fake peripheral, a fake clock and either a fake handler or the real `PhoneServer`.
- Alternative: logic inside the D-Bus callbacks. Rejected: untestable without BlueZ, and the spike showed how easily the state goes wrong across reconnects.

**D3. Notifications by `PropertiesChanged` on TX `Value`, not `AcquireNotify`.**
- The spike measured it, and BlueZ keeps each device's subscription state itself.
- `AcquireNotify` gives one socket for one subscriber. BlueZ's behaviour with a second subscriber is unmeasured, and the socket must be acquired again on every reconnect.
- Consequences:
  - a notification cannot be addressed, so it reaches every subscribed device. Hence D5's one-phone rule by disconnecting, and no `too_many_clients` over BLE;
  - there is no backpressure from BlueZ, hence D4.
- Revisit (a design change, not a silent switch) only if task 1's measurement shows `PropertiesChanged` losing or reordering pieces.

**D4. Paced output with a bounded queue.**
- `BleConnection::Send` splits a message into pieces of MTU−3 bytes, the first piece always fresh, and queues them.
- `Pending()` is the queued byte count; past the cap (64 KiB, as for the socket) the connection closes and the device is disconnected.
- A loop timer drains the queue at a fixed rate R in 10 ms ticks (a token bucket allowing a burst of one tick).
  - R = **24 KiB/s**, under half of the measured lossless rate at MTU 517 (54 KiB/s, see Measurements).
  - Protocol v1 needs little: events are below 1 KiB/s, and the largest response, a `status.get` with 16 warnings, is about 11 KiB, about 0.5 s at 24 KiB/s.
- `OnWritable` fires when the queue empties. Then the server's held state and progress go out, and a slow link drops events (`dropped`), never chunks.
- Alternative: emit unpaced, as the spike did. Rejected: bluetoothd queues every signal, so a phone out of range would grow bluetoothd's memory without bound.

**D5. Which device is the connection, and the one-phone rule.**
- `BleLink` tracks connected devices from Device1 `Connected` changes. dLidar subscribes to ObjectManager `InterfacesAdded` and Device1 `PropertiesChanged` on `org.bluez`, and reads `GetManagedObjects` when BlueZ appears.
- The first connected device becomes the phone; advertising stops at once.
- Any further device that connects while one is tracked is disconnected (`Device1.Disconnect`). Its writes are rejected with `org.bluez.Error.NotPermitted`.
- The connection opens on `StartNotify` while a phone is tracked, and closes on `StopNotify`, the phone's `Connected` false, or `BluezDown`.
- Advertising resumes when the tracked phone disconnects and no other device stays connected.
- A write from the phone while notifications are off is rejected with `NotPermitted`: its answer could not be delivered. The app always subscribes first.
- **Idle phones are disconnected after 30 s.** A tracked phone without an open connection (not subscribed) for 30 s is disconnected (`Device1.Disconnect`). The clock starts when it connects, when BlueZ comes up with it already connected, and when notifications go off. Found in task 5.1: Android kept the LE link to the bonded Pi through dLidar restarts after the app's "Disconnect", so dLidar never advertised and the app's scan found nothing. First set to 10 s, which cut off Android's pairing prompt (capture 12:13: Security Request, then dLidar's disconnect 12 s later); 30 s leaves time to answer it.
- Alternative: `AcquireNotify` for its device id. Rejected in D3. Alternative: let BlueZ advertise during a connection. Rejected: the second phone would find the scanner.

**D6. MTU per connection.**
- The MTU is 23 until a `WriteValue` from the phone reports `mtu`. Every later write updates it, and the next message is split by the new value.
- Pieces already queued keep their size: they are at most the old MTU−3, still valid.

**D7. Writes.**
- Accepted from the phone with `offset` 0; any other offset gets `org.bluez.Error.InvalidOffset` (no long writes: the app's pieces are at most MTU−3).
- The bytes go to `OnBytes`, so the server's `LineFramer` joins them. The D-Bus reply is sent at once, after the server has handled the bytes.

**D8. Security flags.**
- RX: `encrypt-write`, `write-without-response` (the spike's secure mode, which the phone used).
- TX: `notify`, `encrypt-notify`, so an unbonded phone cannot even subscribe.
- When BLE starts, dLidar sets the adapter `Pairable` false, overriding BlueZ's default yes. Only the pairing window (D10) makes it pairable.
- **dLidar guards `Pairable`.** Any BlueZ client that registers an agent can switch pairing on (task 1.2 saw it happen, from a second, older dLidar's pairing window). So dLidar watches Adapter1 `Pairable` and sets it false again whenever it turns true outside its pairing window. The first time per start it prints one warning: `another program made Bluetooth pairable; switched off (pairing only in the pairing window)`.
- On the target image nothing else registers an agent, so the guard should never fire.
- Task 2 checks with the phone that an unbonded phone gets an ATT security error on both characteristics.

**D9. Advertisement.**
- `LEAdvertisement1` with:
  - `Type` peripheral;
  - `ServiceUUIDs` [service UUID];
  - `LocalName` = `[phone] name`;
  - `Discoverable` true (LE General Discoverable flag).
- Measured (task 1.2, `btmon`): BlueZ 5.66 puts the 128-bit UUID list (18 bytes) in the advertisement data and the `LocalName` in the scan response, even with room left. With `Discoverable` the flags field (3 bytes) joins the UUID: 21 of 31 bytes. The app filters by UUID and Android scans actively, so it also gets the name. The settings cap the name at 20 characters (the scan response holds 29).
- dLidar registers the advertisement while no phone is connected and BlueZ is up, and unregisters it when a phone connects.
- **The adapter's `Alias` is set to the name too.** A connected or bonded Android phone shows the GAP Device Name, which BlueZ serves from the adapter's alias, by default the hostname (`lsh` on the dev board). Found in task 5.1: the phone showed `dLidar` while scanning and `lsh` once bonded. BlueZ keeps the alias after dLidar stops; on a dedicated scanner that is intended.

**D10. Pairing window on the loop, never blocking.**
- `OpenPairingWindow(120 s)` is asynchronous and returns at once. The `main.cpp` thread no longer waits.
- On the loop it:
  1. registers an `Agent1` (`NoInputNoOutput`, accepting every request) and requests it as the default agent;
  2. sets `PairableTimeout` 0 and `Pairable` true;
  3. after 120 s on a loop timer, unregisters the agent, then sets `Pairable` false (order from the `phone-link-core` finding).
- Device1 `Paired` becoming true prints `Bluetooth: paired with <alias>`.
- If BlueZ is down when the window is due, it opens when BlueZ appears, with the remaining time.
- `pairing_at_start` with `ble = false` prints a warning and opens nothing.
- The console messages are unchanged.

**D11. Asynchronous D-Bus only, and recovery.**
- The system bus is obtained with `g_bus_get` from the loop thread (the context is thread-default there, so GDBus dispatches our callbacks on the loop).
- Every BlueZ call is `g_dbus_connection_call` with a callback.
- BlueZ comes and goes with `g_bus_watch_name_on_connection("org.bluez")`:
  - appeared: find the adapter (the first `Adapter1`; none is a warning, and `InterfacesAdded` is watched), set `Pairable` false, `RegisterApplication`, then `RegisterAdvertisement`;
  - vanished: `BluezDown` closes the connection; the GATT and advertisement objects stay exported for the next registration.
- Each cause prints one warning (no adapter, registration failed, BlueZ gone), and its recovery prints one line.
- GDBus starts one worker thread per process for the bus. It lives until exit, so the socket test's thread count is measured with BLE off.

**D12. Start-up and the second dLidar.**
- `PhoneLink::Start(api, socket_dir, ble_options)`:
  - When the socket finds another dLidar answering, the whole link stays off, BLE included. Two GATT applications with the same UUIDs would confuse the phone.
  - Any other socket failure (no runtime directory) still starts BLE.
  - BLE failures never stop the link: they warn and recover (D11).
- Shutdown order:
  1. the flush (the final state also reaches the phone, paced);
  2. the advertisement and application are unregistered, and the agent too if the window is open;
  3. `Pairable` false;
  4. the socket closes;
  5. the loop stops.

  Unregistering is best effort, bounded by the 200 ms flush plus 300 ms. BlueZ drops our objects anyway when the process exits.

**D13. Settings.**
- `[phone] ble` (bool, default `true`) and `[phone] name` (string, 1–20 printable ASCII characters, default `"dLidar"`).
- Their defaults live in `device_config.h` with `kPairingAtStart`.
- `dlidar.toml` lists them.

**D14. Service unit.**
- `systemd/dlidar.service` gets `Wants=bluetooth.service`, `After=bluetooth.service network-online.target` and `SupplementaryGroups=gpio bluetooth`. The group covers images whose BlueZ policy is upstream's.
- `RuntimeDirectory=dlidar` gives `/run/dlidar/dlidar.sock`.
- Installed on the dev board as `vs2022` from `/home/vs2022/dLidar`, then enabled. `ble-spike` is stopped and `tools/ble_spike` deleted only after the user confirms.

**D15. Tests.**
- `tests/adapters/phone/test_ble_link.cpp` (pure, fake peripheral and clock, the real `PhoneServer` where useful):
  - splitting at MTU 23, 247 and 517, a fresh notification per message;
  - the MTU learned from writes;
  - the pacing rate, `Pending`, `OnWritable` and the cap;
  - open and close on notify and disconnect, `seq` restarting from 1;
  - a second device disconnected, its writes rejected, the first unaffected;
  - advertising off while connected and on after it;
  - BlueZ down and up;
  - a write before subscribing rejected;
  - offset ≠ 0 rejected.
- `test_settings`: `ble`, and `name` length and characters.
- On the dev board with the Galaxy S25 Ultra and the app (task 4):
  - discovery after boot;
  - the bonded reconnect;
  - an unbonded phone refused;
  - the pairing window;
  - `status.get` and events during a `--once` recording;
  - restarting `bluetooth.service`;
  - out of range during a recording;
  - `btmon` for the advertisement contents.
- Regression: golden replay, and the `--once` console output identical to the previous build. With Bluetooth working the link prints nothing.

## Measurements

**1.1 Throughput (2026-11-03, Galaxy S25 Ultra, nRF Connect, MTU 517, spike in secure PropertiesChanged mode):**
- The spike hands floods to BlueZ in milliseconds (`flood 200 300`, `500 1000`, `100 4096`: 3.6 to 7.1 MB/s). BlueZ queues all of it, unpaced, hence D4.
- On the air (`btmon -t` during `flood 500 1000`): 500 messages became 1000 notifications (514 + 488 bytes), all sent in order, none dropped by BlueZ.
- The radio confirmed the 500,000 bytes over 8.99 s (1248 "Number of Completed Packets" events): **about 54 KiB/s**. The longest gap between confirmations was 63 ms.
- Pacing rate R = 24 KiB/s (D4).

**1.2 Unbonded phone (2026-11-03, binary `btmon -w` capture, bond removed on both sides, `Pairable` false):**
- Subscribing to TX (`encrypt-notify`) made the Pi send an SMP Security Request. Android showed its pairing prompt by itself, and Just Works pairing completed. So `encrypt-notify` protects the events and Android pairs on its own.
- **Pairing was possible although `Pairable` had been set false.** At 10:34:55, a minute after it was switched off, bluetoothd sent the controller `Set Bondable: Enabled` and `Set IO Capability: NoInputNoOutput`, which is an agent registering. Exactly 120 s later (10:36:56) the same pair of commands switched it off again. Cause, found afterwards: a `bin/dLidar` (an older build with the threaded `PairingWindow`) was started by hand from a VS Code terminal at 10:34:55, evidently with `pairing_at_start = true`; its 120 s window explains both commands. So it was dLidar's own pairing window, not a foreign program. (A first suspicion, the desktop taskbar's Bluetooth plugin `wf-panel-pi`, was wrong.)
- Lesson: every program that registers a BlueZ agent can make the adapter pairable. The `Pairable` guard (D8) stays as defence in depth against other agents (a desktop Bluetooth applet, `bluetoothctl`'s default agent, a leftover spike).
- The earlier "write failed: not connected" in nRF Connect came from a connection that had already dropped. It is not a security error.

**1.2 Advertisement (`btmon` while bluetoothd re-registered the spike):** the advertisement data holds only the 128-bit service UUID list (18 bytes); the name `dLidar` (8 bytes) is in the scan response. The spike sets no `Discoverable`, so no flags field (D9).

**Group 4 on the dev board, without the phone (2026-11-03, a test instance with spare pins and its own socket directory):**
- At start the adapter is set not pairable. The objects are exported with TX `notify` + `encrypt-notify` and the name `dLidar`.
- Advertising is off while a phone is connected (the bonded Galaxy over LE), and starts at once when it disconnects.
- `systemctl restart bluetooth`: advertising again after 0.28 s, one warning ("bluetoothd is gone") and one line when it is back.
- A `Pairable` set from outside is switched off again with the guard's warning. bluetoothd's own pairable-at-start is switched off quietly.
- Pairing window: pairable for 120 s with "started" and "ended" printed, then not pairable. `pairing_at_start` with `ble = false` warns and opens nothing.
- `rfkill block`, at start or while running: one "switched off" warning, the socket works; after `unblock`, "on again" and advertising resumes.
- Start-up to "Ready" with bluetoothd stopped: 1015/1023 ms with BLE versus 1013/1008 ms without. Note: D-Bus activation restarts a stopped bluetoothd within 40 ms on this image.
- SIGTERM: the advertisement is unregistered (`ActiveInstances` 0) and the socket file removed.
- No classic link can stand in for the phone: the adapter is not `connectable` over BR/EDR (`btmgmt info`), so a bonded phone reaches the Pi over LE only.

## Risks / Trade-offs

- **[Notifications go to every subscriber]** → a second phone is disconnected as soon as BlueZ reports it connected. In the moment before that, a bonded second phone that subscribes very fast could receive some notifications. Bonds come only through the user's pairing window, so this is a bonded phone of the user's. Phase 3's allow-list can disconnect earlier, from `Device1` `Connected` before services resolve.
- **[Another BLE device on the Pi]** (for example a keyboard) counts as "connected" and stops advertising → the dev and target Pis have none. If one appears, filter by the devices that connect over LE or that subscribe; noted for later.
- **[The pacing rate is a guess until measured]** → task 1 measures it first. A rate too low only makes large responses slower, never wrong.
- **[`encrypt-notify` behaviour on the phone]** → task 2 checks it with an unbonded phone. If Android does not pair on a CCC write error, RX `encrypt-write` still protects every request, and TX carries only events, no secrets.
- **[The name only in the scan response]** → a passive scan sees the UUID but no name. The app scans actively and filters by UUID, so it does not depend on the name.
- **[Shutdown with a hung bluetoothd]** → unregistering is bounded (D12). The process exit releases everything in BlueZ.
- **[GDBus worker thread]** → one per process, expected; documented in the tests.
- **[Another program switches pairing on]** (seen once: a second dLidar's pairing window) → the `Pairable` guard (D8) switches it off again within one D-Bus round trip and warns once. Before it reacts, a pairing attempt could slip in. Two dLidars never serve BLE at once (D12), and nothing else registers an agent on the target.
- **[The pairable state left on after a crash]** → dLidar sets `Pairable` false at every start (D8). A crash inside the window leaves the adapter pairable until the next start, or until bluetoothd restarts, which reverts it to `main.conf`. Accepted, and noted in the manual.

## Migration Plan

1. `git pull`, rebuild (no new dependency).
2. The service: install the updated unit (D14), `systemctl daemon-reload`, `enable --now dlidar`.
3. After the user confirms: `systemctl stop ble-spike` (a transient unit, gone after a reboot anyway) and `git rm -r tools/ble_spike` with the `DLIDAR_BLE_SPIKE` option.

Rollback: `[phone] ble = false` in `dlidar.toml`, or the previous binary. The bonds stay in BlueZ; nothing else persists.

