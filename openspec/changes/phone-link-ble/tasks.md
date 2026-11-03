# Tasks

## 1. Measure first (spike, design D3/D4/D8/D9)

- [x] 1.1 Throughput with `tools/ble_spike --secure` (PropertiesChanged mode) and the Galaxy S25 Ultra at MTU 517 (nRF Connect or the app's echo test): `flood 200 300`, `flood 500 1000`, `flood 100 4096`. From the spike log and the phone, note the bytes per second that arrive with every `seq=` present and in order. Set the pacing rate R (D4) to about half the lossless rate, and write the numbers into design.md ("Measurements"). Verify that the section names the rate and the losses seen, if any.
- [x] 1.2 In the spike's secure mode give TX the flags `notify`, `encrypt-notify`. With the phone's bond removed and the pairing window closed, check what Android does on subscribing to TX and on writing RX. Note whether Android starts pairing on its own in each case. Check with `sudo btmon` that the spike's advertisement packet (not the scan response) holds the flags, the 128-bit service UUID and the name. Write both results into design.md ("Measurements"), and adjust D8/D9 if one fails. Verify with the btmon excerpt quoted in the section.

## 2. Settings and service unit (D13, D14)

- [x] 2.1 `[phone] ble` (bool) and `[phone] name` (1–20 printable ASCII characters). Defaults `kBleEnabled = true` and `kBleName = "dLidar"` go in `src/app/device_config.h`, the parser in `src/app/settings.cpp`, both keys in `dlidar.toml` and in the `docs/manual.html` settings table. Extend `tests/app/test_settings.cpp`: the defaults, a valid name, an empty name, 21 characters, a non-ASCII byte, a control character, and `ble = 1`. Verify with ctest `settings`.
- [x] 2.2 `systemd/dlidar.service`: `Wants=bluetooth.service`, `After=bluetooth.service network-online.target`, `SupplementaryGroups=gpio bluetooth`. Verify with `systemd-analyze verify` on a copy with USER/REPO filled in.

## 3. BleLink, the pure state machine (D2, D4–D7)

- [x] 3.1 `src/adapters/phone/ble_link.{h,cpp}`:
  - the `BlePeripheral` port (`Notify`, `Disconnect`, `SetAdvertising`, `WakeAt`);
  - `BleLink` with the inputs of D2;
  - the `BleConnection` (`Send` split into MTU−3 pieces with the first piece fresh, `Pending`, the 64 KiB cap, `Close` disconnecting the phone);
  - the token-bucket drain at rate R in 10 ms ticks;
  - the one-phone rule and advertising on/off (D5);
  - the MTU from writes (D6);
  - write rejection for another device, notifications off, and offset ≠ 0 (D7).

  No GLib or D-Bus in these files.
- [x] 3.2 `tests/adapters/phone/test_ble_link.cpp` with a fake peripheral and a fake clock, the D15 list:
  - splitting at MTU 23, 247 and 517, a fresh notification per message, and the spec's 3000-byte example (13 pieces at MTU 247);
  - the MTU learned from writes;
  - the pacing rate over 10 s, `Pending`, `OnWritable` when the queue empties, and the cap closing the connection and disconnecting;
  - open on notify, close on stop-notify, disconnect and BlueZ down, then `seq` from 1 after a reconnect (with the real `PhoneServer`);
  - a second device disconnected and its writes rejected, the first one's stream unaffected;
  - advertising off on connect and on after the disconnect;
  - the write rejections;
  - a held state delivered after a slow link drains.

  Verify with ctest `ble_link`.

## 4. BlueZ glue, pairing window, wiring (D1, D8–D12)

- [x] 4.1 `src/adapters/phone/bluez_peripheral.{h,cpp}`, asynchronous D-Bus only, on the phone loop:
  - the system bus via `g_bus_get` from the loop thread;
  - the `org.bluez` name watch, finding the adapter, `InterfacesAdded`;
  - the GATT application objects and `RegisterApplication`, with the flags of D8;
  - the advertisement (D9), registered and unregistered as `BleLink` asks;
  - `Pairable` false at start, and the guard: Adapter1 `Pairable` turning true outside the pairing window is set false again, with one warning per start (D8);
  - Device1 `Connected` and `Paired` signals into `BleLink`; `WriteValue`, `StartNotify` and `StopNotify` into `BleLink` with the D-Bus error names of D5/D7;
  - TX `Value` `PropertiesChanged` for `Notify`; `Device1.Disconnect`;
  - one warning per cause, and one line on recovery.
- [x] 4.2 Move the pairing window into `bluez_peripheral` (D10): asynchronous, the agent registered only while it is open, the order unregister → `Pairable` false, the remaining time when BlueZ appears late, the same console messages. Delete `src/adapters/bluetooth/` and the `dlidar_bluetooth` target. Verify that `main.cpp` no longer waits on any Bluetooth call (start-up time with `bluetooth.service` stopped equals the time with BLE off, within 0.2 s).
- [x] 4.3 `PhoneLink::Start(api, socket_dir, ble)` with the BLE options and the start-up rules of D12, the shutdown order, and `OpenPairingWindow`. `src/app/main.cpp` passes `settings.phone` (ble, name, pairing_at_start). With `ble = false` and `pairing_at_start = true`, a warning and no window. Verify that ctest `phone_socket` (BLE off) still passes with no thread left behind, and that `tools/check_architecture.py --strict src` passes.

## 5. On the dev board with the phone (spec scenarios)

- [x] 5.1 Run `bin/dLidar` by hand (button not pressed; L2 absent) with `ble = true`. Check with the Galaxy S25 Ultra and the app, and `btmon` where noted:
  - discovered by the UUID filter, the advertisement contents (btmon);
  - the bonded phone connects, `hello`/`status.get`/`info.get` are answered, and the phone's MTU appears in the debug log;
  - advertising stops while connected and resumes within 2 s after the disconnect;
  - reconnect with `seq` from 1;
  - `sudo systemctl restart bluetooth` re-advertises within 10 s with one warning;
  - with the bond removed, the write is refused with the window closed, and pairing works with `pairing_at_start = true` (the messages printed);
  - with `bluetooth.service` stopped, dLidar starts with one warning and the socket works;
  - `busctl set-property … Pairable b true` from outside, with the window closed, is switched back to false with the warning.

  Write a short protocol of the results into design.md ("Measurements").
- [x] 5.2 `bin/dLidar --once` without a sensor (the only recording flow on the dev board until the Mid-360S arrives), with the phone following events: the `preparing` state, the `sensor_failed` warning and the `error` state arrive on the phone. Move the phone out of range and back during `--once`, and check that `status.get` after the reconnect is correct. Verify that the console output equals a run with `ble = false`.

## 6. Docs, service, cleanup, integration

- [x] 6.1 Docs:
  - `docs/phone-protocol.md`, the BLE section: second phone disconnected (no `too_many_clients` over BLE), TX encrypted too, MTU 23 until the first write, the pacing rate;
  - `openspec/changes/phone-link-core/app-questions.md`, Q26 corrected;
  - `docs/manual.html` ("Phone link": BLE on by default, the name, how to find and pair, what a crash inside the pairing window leaves; the settings table);
  - `docs/architecture.md` (the phone adapter with BLE, `bluetooth/` gone, Deferred);
  - `CLAUDE.md` (the phone bullet).

  Send the app agent the BLE changes (Q26, encrypted TX, MTU before the first write). Verify that the manual and architecture.md agree.
- [ ] 6.2 Install `systemd/dlidar.service` on the dev board (USER `vs2022`, REPO `/home/vs2022/dLidar`) and the polkit rule, set `JustWorksRepairing = confirm` in `/etc/bluetooth/main.conf` (D14) and restart `bluetooth`, then `systemctl enable --now dlidar`. Check that a phone unpaired on its side pairs again inside the window without removing the bond on the Pi, and is refused outside it. Reboot, and check that the phone finds `dLidar` without anyone logging in, and that `/run/dlidar/dlidar.sock` answers `tools/phone_client.py status.get`. Verify with `systemctl is-enabled dlidar` and the phone.
- [ ] 6.3 Only after the user confirms: stop and disable the `ble-spike` unit if it exists, delete `tools/ble_spike/` and the `DLIDAR_BLE_SPIKE` option. Update `phone-link-core` design D1 ("deleted"). Verify that a clean configure and build pass.
- [ ] 6.4 Run the full ctest suite, `tools/golden_replay.sh` (identical), and the `--once` console comparison with the previous build. Run the architecture-reviewer agent on the branch. Verify that everything passes and that the reviewer approves the tip before the push.
