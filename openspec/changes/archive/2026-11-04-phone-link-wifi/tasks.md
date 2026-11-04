# Tasks

## 1. Port and use case (D2)

- [x] 1.1 `src/application/ports/wifi_manager.h`: the port `WifiManager` (`Link()`, `Connect(network, limit, done)`) and the types `WifiNetwork`, `WifiSecurity`, `WifiResult`, `WifiOutcome`, `WifiLink`, `WifiState`. Then `src/application/remote/wifi_setup.h`: `WifiSetup` with `Check`, `Begin` (`Accepted`/`Busy`), `Status()` (`connecting` while running, `last`) and `kWifiConnectLimit = 60 s`.
  - Verify with a new ctest `wifi_setup` (`tests/application/test_wifi_setup.cpp`, fake `WifiManager`): every argument rule and boundary from D9, `busy`, `connecting`, `last` after each outcome, `done` forwarded once.
  - Verify that `tools/check_architecture.py --strict` passes.

## 2. Protocol and phone server (D7)

- [x] 2.1 Codec: encoders for the `wifi.connect` answer (`accepted`, `timeout_s`), the `result` event payloads (ok with `ssid`/`ip`; errors `wrong_password`, `not_found`, `timeout`, `failed`) and the `wifi.status` answer (`state`, `ssid`, `ip`, `last`). Make sure no existing `bad_request` message echoes an argument's value. Verify with ctest `protocol_codec`.
- [x] 2.2 `PhoneServer`:
  - the `wifi.connect` and `wifi.status` commands, offered only with a `WifiSetup`;
  - the requester and id remembered;
  - `OnWifiResult` (log line, `result` event to the requester even when backlogged, nothing if it closed).

  `PhoneLink::Start` takes a `WifiSetup *` and routes `done` through `PhoneLoop::Invoke`. Verify with ctest `phone_server` and `phone_socket`, covering the D9 cases: requester only, backlogged requester, closed requester, `hello` with and without, and the marker password absent from captured stdout and every sent byte.
- [x] 2.3 `docs/phone-protocol.md`:
  - sections for `wifi.connect` and `wifi.status`, with examples (joined, `wrong_password`, `busy`, `bad_request` for `psk`, the status during an attempt and after one);
  - the result error codes and `timeout_s`;
  - the "Results that come later" text no longer says "a later phase";
  - `hello`'s example lists both commands.

  Verify that ctest `protocol_codec` accepts every new example line. Then send the draft sections to the app agent and record its answer in `openspec/changes/phone-link-core/app-questions.md`.

## 3. NetworkManager adapter (D1, D3–D6)

- [x] 3.1 CMake target `dlidar_network` (`src/adapters/networkmanager/`, no new library):
  - `CommandRunner` with the `posix_spawn` runner (`LC_ALL=C`, pipes, timeout, `Cancel()` kills the child, never logs the argument vector);
  - pure helpers: `OutcomeFor(exit_code, err)`, `ProfilesToReplace(profiles, ssid, new_uuid)`, and the parsers for the terse link and profile output.

  Verify:
  - a new ctest `nmcli_wifi` (`tests/adapters/networkmanager/test_nmcli_wifi.cpp`) covers the D5 table with the board's texts, `ProfilesToReplace` against the dev board's profile set (never `lidar`, never an access point, never another SSID) and the parsers;
  - the real runner runs `nmcli -t -f GENERAL.STATE device show wlan0` on the board and returns its output.
- [x] 3.2 `NmcliWifi`:
  - its worker thread;
  - the `Link()` cache (a 5 s refresh plus after each connect; `unavailable` without `nmcli`, NetworkManager or `wlan0`);
  - `Connect` as in D3: in-memory add with a generated UUID, activation, save by `modify`, replace, delete and restore on failure or timeout, `done` within `limit`;
  - the permission hint;
  - `Stop()` cancelling and cleaning up.

  Verify with ctest `nmcli_wifi` and a scripted runner for every D3 path from D9, with the argument vectors checked against D4 and the password only in the `add`. The read-only check against the board's NetworkManager (`Link()` gives `connected`, `TP-Link_B8CB`, the board's IP) is part of ctest `nmcli_wifi`. D3's open point (does `modify` write an in-memory profile to disk?) needs the polkit rule, so it moved to 5.2.

## 4. Wiring and permission (D8)

- [x] 4.1 `main.cpp`:
  - creates `NetworkManagerWifi` and `WifiSetup`;
  - passes the setup to `PhoneLink::Start`;
  - stops the adapter before the phone link at shutdown (no result after the link is gone).

  `systemd/40-dlidar-network.rules` (`USER` placeholder, the two actions). The manual's install block gets the copy line, and "Running as a service" explains the rule. Verify:
  - full ctest;
  - `tools/golden_replay.sh` identical;
  - dLidar starts and stops cleanly with NetworkManager running;
  - without a system bus it still starts, and `wifi.status` reports `unavailable`. To check this, stop the service, then run `DBUS_SYSTEM_BUS_ADDRESS=unix:path=/nonexistent ./bin/dLidar --once` and query it with `tools/phone_client.py`.

## 5. On the dev board (D10)

Driven from the app by the user, the journal captured to a file. Before each step, take the profile inventory (`nmcli -t -f NAME,TYPE,UUID con show` and the file names in `/etc/NetworkManager/system-connections/`); after each step, compare it and search the journal (dLidar and NetworkManager) for the marker password.

- [x] 5.1 Restart the service with the new build.
  - Check that `tools/phone_client.py wifi.status` reports `connected`, `TP-Link_B8CB` and the board's IP, and that `hello` lists both commands.
  - Without the rule: `wifi.connect` from the app ends in `failed`, with the rule named, and the inventory is unchanged.

  **Done 2026-11-03:** service restarted 22:15 with 340cf5b; `hello` lists both commands, `wifi.status` gave connected, TP-Link_B8CB, 192.168.1.106. The no-rule join was run over the local socket instead of the app (the app could not scan yet): `failed` with the rule named, inventory unchanged, marker in no output.
- [x] 5.2 Install the rule (sudo, with the user's go-ahead). First, as the service user and without changing any network:
  - add a `save no`, `autoconnect no` profile for a made-up SSID and never activate it;
  - run `nmcli connection modify uuid <U> connection.autoconnect yes`, and check that its file appears in `/etc/NetworkManager/system-connections/` (D3's open point);
  - delete it, and check that the inventory is unchanged.

  Then send a made-up SSID with a marker password from the app:
  - the outcome is `not_found` or `timeout`;
  - the board is back on `TP-Link_B8CB` within 60 s;
  - the inventory is unchanged;
  - the marker is in no log.

  Record which outcome came, and after how long (design Open Questions).

  **Done 2026-11-03:** the first install as `50-` did not grant `settings.modify.system`: Debian's `49-polkit-pkla-compat.rules` (NetworkManager's `.pkla`, `sudo` group, not a local session) answered no first. Renamed to `40-dlidar-network.rules`; then both rights are yes. The probe confirmed D3: `save no` writes no file, the `modify` writes it, the delete leaves the inventory unchanged. The app cannot type an unseen SSID, so the made-up SSID went over the local socket: `not_found` after 28.5 s (not a timeout), back on TP-Link_B8CB, inventory unchanged, no marker in the journal.
- [x] 5.3 Send `TP-Link_B8CB` with a wrong marker password:
  - the outcome is `wrong_password`;
  - the board is back on it, and the profile is unchanged (same UUID);
  - the marker is in no log.

  Record the device state reason (D5).

  **Done 2026-11-03:** from the app: `wrong_password` after 17 s; the board back on TP-Link_B8CB with the same UUID; no marker in dLidar's journal (the NetworkManager journal is not readable for this user); the BLE link stayed up.
- [x] 5.4 Join the phone's hotspot (WPA3 if offered, else WPA2):
  - the outcome is `connected`, with its IP;
  - a new profile is saved next to the old ones, and `lidar` and `LidarStation` are unchanged.

  Then send `TP-Link_B8CB` with its real password from the app: the board returns, one `TP-Link_B8CB` profile remains, and the hotspot profile is still saved. Record whether the BLE link survived each join (the app's log, the journal's Bluetooth lines).


  **Done 2026-11-03:** no hotspot was available (one phone), so the success path was a join of TP-Link_B8CB with its real password from the app: `connected`, 192.168.1.106 after 8.6 s; exactly one TP-Link_B8CB profile left (new UUID, on disk, autoconnect yes; its old autoconnect priority 5 became the default 0, the documented loss of custom settings); LidarStation, preconfigured, lidar unchanged; BLE stayed up. WPA3 and a move to another network are untested.
## 6. Docs and integration

- [x] 6.1 Docs:
  - `docs/manual.html`: Wi-Fi setup from the app, the outcomes, what is saved and restored, the rule, and that a same-SSID profile is replaced;
  - `docs/architecture.md`: the `WifiManager` port, `WifiSetup`, the networkmanager adapter, the target in the layer table; drop Wi-Fi from Deferred;
  - `README.md` and `CLAUDE.md`.

  Fill in D5 and the Open Questions with the board results. Verify that the manual and `architecture.md` agree.
- [x] 6.2 Full ctest, `tools/golden_replay.sh` (identical), `tools/check_architecture.py --strict`, and the architecture-reviewer agent on the tip before the push.

## Workflow follow-up

- Archive the change once the board checks (group 5) are done and the reviewer approved the tip.
