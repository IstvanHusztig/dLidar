# Design

## Context

See proposal.md (Why) and `specs/phone-wifi/spec.md` for the requirements. What exists and shapes the design:

- **The phone link** (`src/adapters/phone/`):
  - `PhoneServer` answers on the phone loop thread from a command table. Every response must come within milliseconds (`docs/phone-protocol.md`, "Ordering and ids"), so no handler may wait for the system.
  - The later-result event already exists in the contract ("Results that come later", requirement in `phone-link-core`'s `phone-api`) and in the codec (`EncodeResultEvent`, unused so far). `wifi.connect` is its first user.
  - `PhoneLink` owns the loop thread; `PhoneLoop::Invoke` runs work on it from any thread.
- **Layers:** an adapter never includes another adapter (`tools/check_architecture.py`), so `main.cpp` connects the Wi-Fi adapter and the phone link through the application layer.
- **The dev board** (measured 2026-11-03):
  - NetworkManager 1.42.4 manages `wlan0` (brcmfmac).
  - `nmcli -f GENERAL device show wlan0` reports `GENERAL.STATE`, `GENERAL.REASON` (NetworkManager's numeric reason with its text), `GENERAL.CON-UUID` and `IP4.ADDRESS`. `nmcli connection add` takes `save no`.
  - Profiles:
    - `TP-Link_B8CB` (Wi-Fi, active, priority 5);
    - `LidarStation` (Wi-Fi infrastructure, priority 10);
    - `preconfigured` (Wi-Fi);
    - `lidar` (Ethernet `eth0`, manual 192.168.0.2/24: the sensor).
  - The service user `vs2022` has `settings.modify.system` = no and `network-control` = auth, and is not in `netdev`. A system service has no login session, so NetworkManager's session-based defaults never apply to it. `nmcli` asks NetworkManager over D-Bus, so the same polkit checks apply: a rule is needed, like the existing `50-dlidar-poweroff.rules`.
  - The developer reaches the board over SSH on that Wi-Fi (192.168.1.102 → .106).

## Goals / Non-Goals

**Goals:**
- Join a network the phone names and report a precise outcome. Never leave the scanner worse off after a failed attempt.
- The password reaches `nmcli` and nothing else that dLidar emits.
- The whole connect sequence (add, activate, save, replace, restore, timeout) is testable without NetworkManager.

**Non-Goals:**
- Scanning or listing networks on the Pi (the app scans), hidden networks, SSIDs that are not UTF-8, WEP, enterprise (802.1X) networks, static IP settings.
- Removing or listing saved networks (the app's wish for later: `wifi.forget {ssid}` and a list), priorities between them, a hotspot on the Pi: possible later commands.
- A settings switch to turn Wi-Fi setup off: `[phone] ble = false` turns off the phone link, and a local client already has a shell.
- Wi-Fi state events: `wifi.status` after a reconnect is enough for v1.

## Decisions

**D1. NetworkManager through `nmcli`, run behind a command-runner interface.**
- Every NetworkManager action is one `nmcli` run with an argument vector: no shell, so an SSID needs no quoting.
- Environment `LC_ALL=C`: messages in English, fixed text.
- The output read is `-t`/`-g` terse fields only.
- `CommandRunner::Run(argv, timeout)` → `{exit_code, out, err}`, with a `Cancel()` for shutdown:
  - the production runner uses `posix_spawn` with pipes;
  - tests use a scripted fake, so the whole sequence runs in a unit test.
- The password is an argument of `nmcli connection add`, so a local user could read it in the process list while that command runs. The scanner has one user, so this is accepted (user, 2026-11-03). The runner never logs an argument vector; a failure message names the `nmcli` subcommand only.
- Alternative: NetworkManager's D-Bus API through GIO. Rejected:
  - about twice the code;
  - testable only against a real or fake bus;
  - its one advantage, keeping the password off the command line, does not matter on a one-user device.

**D2. Port, use case and adapter.**
- `src/application/ports/wifi_manager.h`, the port `WifiManager`:
  - `WifiLink Link() const`: `{state, ssid, ip}`, a cached value that never blocks;
  - `void Connect(const WifiNetwork &, std::chrono::seconds limit, std::function<void(WifiResult)> done)`: `done` is called exactly once, on any thread, within `limit`.

  `WifiNetwork` is `{ssid, security, psk}`. `WifiResult` is `{outcome, ip, message}`, the outcome being `Connected`, `WrongPassword`, `NotFound`, `Timeout` or `Failed`.
- `src/application/remote/wifi_setup.h`, the use case `WifiSetup` (thread-safe), next to `DeviceApi`:
  - `Check(args)`: the argument rules of the spec, pure, returning the offending argument's name. Its messages never contain the value.
  - `Begin(network, done)` returns `Accepted` or `Busy`:
    - it holds one attempt at a time;
    - it calls the port with `kWifiConnectLimit = 60 s`;
    - `last` is set to (ssid, pending) when it starts, and to (ssid, outcome) when `done` comes, before `done` is forwarded.
  - `Status()`: the port's link, with state `connecting` while its own attempt runs, plus `last`. `pending` tells a phone that reconnected that *its* join still runs. `connecting` alone can also be NetworkManager joining a saved network by itself (app wish, 2026-11-03).
  - A `wifi.connect` for the network the scanner is already on does a full join (a short drop). It does not short-cut on "already connected": checking that the password matches would mean reading the saved secret.
- `src/adapters/networkmanager/` (CMake `dlidar_network`, no new library) implements the port as `NmcliWifi` over a `CommandRunner`. It has pure parsing helpers.
- Alternative: put the rules in `PhoneServer`. Rejected: they are device rules, and another input channel (a web page, a USB key) would have to copy them.

**D3. A connect is an in-memory attempt, saved only on success.** `NmcliWifi::Connect`, on the adapter's thread:
1. **Remember.**
   - Read the profile to come back to: `nmcli -g GENERAL.CON-UUID device show wlan0`, empty if none.
   - Read the saved profiles: `nmcli -t -f UUID,TYPE connection show`, then `nmcli -g 802-11-wireless.ssid,802-11-wireless.mode connection show <uuid>` for each Wi-Fi profile. These are the input for D4.
2. **Add in memory.** Run `nmcli connection add type wifi ifname wlan0 con-name <ssid> ssid <ssid> connection.uuid <U> save no`, plus `wifi-sec.key-mgmt wpa-psk|sae wifi-sec.psk <psk>` unless `open`.
   - dLidar generates `U` (from `/proc/sys/kernel/random/uuid`), so it never has to parse a message.
   - The profile lives only in NetworkManager's memory.
3. **Activate.** Run `nmcli --wait <limit − 15> connection up uuid <U> ifname wlan0`. It does not rescan: NetworkManager uses its own scan list.
4. **Exit 0 (connected):**
   - read `IP4.ADDRESS` (`nmcli -g IP4.ADDRESS device show wlan0`, the address without the prefix);
   - save the profile to disk with `nmcli connection modify uuid <U> connection.autoconnect yes`: a modify without `--temporary` writes it;
   - delete each profile that D4 selects (`nmcli connection delete uuid <old>`).

   The outcome is `Connected`.
5. **Any other exit:**
   - take the outcome from the exit code and message (D5);
   - delete `<U>`, which also deactivates it;
   - run `nmcli --wait 10 connection up uuid <prev>` if there was a previous profile.

   A missing or slow restore is logged; NetworkManager's autoconnect is the backstop.

The limits keep `done` within `limit`: activation gets `limit − 15 s`, the rest is the cleanup. `busy` lasts until the cleanup is done.

Why: a wrong password or a typo never replaces a working profile, and a scanner reached over Wi-Fi comes back on its own (spec "A failed attempt changes nothing").

Alternative: add the profile to disk first, and delete it on failure. Rejected: a crash or power loss in between leaves a broken saved profile, which NetworkManager would autoconnect forever. Whether a modify really writes an in-memory profile is checked on the board (task 3.2). If it does not, the fallback is to delete `<U>` and add it again with `save yes`, while connected.

**D4. Which profiles may be touched.** The pure helper `ProfilesToReplace(profiles, ssid, new_uuid)` selects only profiles that:
- are of type `802-11-wireless`;
- have mode `infrastructure` (or empty, which NetworkManager treats as infrastructure);
- have exactly the same SSID;
- are not the new one.

It never selects Ethernet profiles such as `lidar`, access point profiles, or other SSIDs. dLidar only deletes the attempt `<U>`, and the selected profiles after `<U>` is saved. The restore only *activates* the remembered profile; it never changes it. The tests check every argument vector the adapter runs against this rule.

**D5. Outcomes from `nmcli`.** The pure helper `OutcomeFor(exit_code, err)`, in the C locale:

| `nmcli connection up` result | Outcome |
|---|---|
| exit 0 | `Connected` |
| exit 3 (the `--wait` ran out) | `Timeout` |
| exit 4, message names the reason "Secrets were required, but not provided" (`NO_SECRETS`, 7) or a supplicant disconnect (8) | `WrongPassword`: dLidar registers no secret agent, so a rejected key ends in one of these |
| exit 4, "The Wi-Fi network could not be found" (`SSID_NOT_FOUND`, 53) | `NotFound` |
| any step: "Not authorized" or "Insufficient privileges" | `Failed`, with "dLidar may not change the network: install systemd/40-dlidar-network.rules" |
| `nmcli` missing, "NetworkManager is not running", no `wlan0` | `Failed`, with that reason |
| anything else | `Failed`, with `nmcli`'s first error line, which never contains the password |

The exact texts are pinned by the tests from the board's NetworkManager 1.42 (task 5.x measures the wrong-password reason of the Pi's chip). If a text does not match, the outcome falls back to `Failed` with the message: safe, only less precise. The spec does not change.

**D6. The adapter's thread.**
- `NmcliWifi` owns one worker thread. It runs one job at a time: a connect, or a link refresh.
- The link refresh runs `nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show wlan0`:
  - every 5 s while no connect runs;
  - after every connect.

  It fills the cache that `Link()` reads under a mutex, so `wifi.status` never waits. `nmcli` missing, NetworkManager not running or no `wlan0` give `unavailable`. One `nmcli` run every 5 s costs a few milliseconds of CPU, during a recording too.
- `Stop()`, at shutdown:
  - cancels the running `nmcli`, through the runner, which kills the child;
  - deletes an unsaved attempt `<U>` (best effort, 1 s);
  - joins the thread. `done` is not called.

  An in-memory profile left behind by a kill would vanish at the next reboot anyway.
- Alternative: run `nmcli` asynchronously with `GSubprocess` on a GLib loop. Rejected: no GLib is needed for a sequence of blocking steps on a thread of its own.

**D7. In the phone server.**
- `PhoneLink::Start` gets a `WifiSetup *` (null: the commands are not offered, and `hello` does not list them).
- `wifi.connect` handler:
  1. `Check` the arguments: an error is `bad_request` naming the argument.
  2. `Begin`: `Busy` becomes the error `busy`.
  3. Answer `{"accepted":true,"timeout_s":60}` and print `phone: wifi.connect Home (wpa2-psk)` (the client's `Kind()`).
- The server remembers the requesting connection and id. `done` runs on the adapter thread; `PhoneLink` passes it through `PhoneLoop::Invoke` to `PhoneServer::OnWifiResult`, which:
  - prints `Wi-Fi: connected to Home, 192.168.1.50` or `Wi-Fi: Home: wrong password`;
  - sends the `result` event, if that connection still exists. The event goes out even when the connection is backlogged ("never left out"): the transport's byte cap still protects memory.

  A closed connection only clears the remembered pair.
- `wifi.status` handler: `WifiSetup::Status()` encoded as `{"state","ssid","ip","last"}`.

**D8. Permission: a polkit rule.**
- `systemd/40-dlidar-network.rules` returns YES for the user `USER` on two actions:
  - `org.freedesktop.NetworkManager.settings.modify.system`: add, modify and delete profiles;
  - `org.freedesktop.NetworkManager.network-control`: activate and deactivate.

  It is named `40-` so it runs before Debian's `49-polkit-pkla-compat.rules`: NetworkManager's `.pkla` answers `no` for `settings.modify.system` to the `sudo` group outside a local session, which ended the check before a `50-` rule (found on the board, 2026-11-03).
  It has the same install pattern as the power-off rule (replace `USER`, copy it to `/etc/polkit-1/rules.d/`), and the manual's install block gains the line.
- Alternative: the `netdev` group. Rejected: Debian's NetworkManager defaults grant it to *active sessions*, which a service never has.

**D9. Tests.**
- `tests/application/test_wifi_setup.cpp`, with a fake `WifiManager`:
  - every argument rule, including the boundaries (8 and 63 characters, 64 hex, 65 characters, a non-hex 64-character password, 32 and 33 bytes of SSID, `psk` with `open`);
  - `busy` while an attempt runs;
  - `connecting` in `Status()`;
  - `last` after each outcome;
  - `done` forwarded once.
- `tests/adapters/networkmanager/test_nmcli_wifi.cpp`, with a scripted `CommandRunner`:
  - every D3 path: connected with an older same-SSID profile replaced, wrong password, not found, timeout, permission denied, `nmcli` missing, no previous profile, a failed restore;
  - `OutcomeFor` with the board's texts;
  - `ProfilesToReplace` against the board's profile set;
  - the link parsing for each state.

  Every argument vector run is checked: `lidar`'s and other profiles' UUIDs never appear in a `modify`, `delete` or `down`. The password appears only in the `add`. Nothing the adapter prints contains it.
- `test_phone_server.cpp`:
  - both commands' answers;
  - `bad_request` and `busy` shapes;
  - the result going only to the requester, even when it is backlogged;
  - a closed requester (no crash, the result only logged);
  - `hello` listing the commands only when a `WifiSetup` is given;
  - a marker password sent in every outcome, checked absent from captured stdout and from every byte sent to every client.
- `test_protocol_codec`: every new example line in `docs/phone-protocol.md`.
- On the board: D10.

**D10. Board tests around the SSH link.** Joining another network moves the dev board off the developer's Wi-Fi.
- The tests are driven from the app, by the user, while dLidar's journal is captured to a file. Bash commands on the Pi keep running; only the VS Code window reconnects.
- Order, safest first:
  1. `wifi.status`.
  2. Without the rule: `failed` naming it; nothing changes.
  3. Install the rule (sudo, with the user's go-ahead).
  4. `not_found`: a made-up SSID. The board drops off Wi-Fi for at most 60 s, then returns.
  5. `wrong_password` for the current network: the same drop. Afterwards the profile is unchanged.
  6. Join the phone's hotspot (WPA3 if the phone offers it, else WPA2): the board moves there. Then `wifi.connect` back to `TP-Link_B8CB` from the app over BLE, and the board returns.
- After each step:
  - the profile inventory (`nmcli -t -f NAME,TYPE,UUID con show`, and the file names in `/etc/NetworkManager/system-connections/`) is compared with the one before;
  - the journal (dLidar and NetworkManager) is searched for the marker password;
  - the app's log shows whether the BLE link survived.

## Risks / Trade-offs

- **[BLE and Wi-Fi share the Pi 5's radio; the link may drop while joining]** → measured in D10. The app reconnects through the bond and reads `wifi.status` (`last`). The spec does not promise that the link survives.
- **[`nmcli`'s texts or fields change in another NetworkManager version]** → only terse fields and C-locale messages are read; the tests pin the 1.42 texts; an unmatched text degrades to `Failed` with the message. The target image is the same Raspberry Pi OS as the dev board.
- **[The password is visible in the process list while `nmcli connection add` runs]** → accepted: one user on the device (D1).
- **[The Wi-Fi subnet overlaps the sensor's Ethernet subnet]** (the Mid-360S defaults to 192.168.1.x, like many home networks) → the sensor keeps working, because its route on `eth0` has the lower metric (100 vs 600), but it is confusing. The `connected` log line prints the IP; the Mid-360S bring-up checklist already moves one of them.
- **[`not_found` may only show as `timeout`]** NetworkManager does not request a scan for a non-hidden SSID it has not seen, so it may wait for its next periodic scan → the spec allows `timeout` for an SSID that is not in range. The app picked the network from a fresh scan, so this is rare.
- **[WPA3-SAE on the Pi's brcmfmac chip]** SAE support depends on the firmware → test with a WPA3 hotspot (D10). If it fails, the outcome is `failed` with NetworkManager's reason, and transition-mode networks still work as `wpa2-psk`.
- **[A same-SSID profile made by hand loses its custom settings]** (a static IP, for example) when replaced → documented in the manual. Such setups are not the scanner's case.
- **[The password is in `/etc/NetworkManager/system-connections/`]** → that is NetworkManager's store, root-only (0600). Without it, the scanner could not join at boot.
- **[A stray `wifi.connect` moves a remotely administered scanner]** → only bonded phones and local users can send it, and a failure restores the previous network. A success is what was asked for.

## Migration Plan

1. Rebuild, install the polkit rule (manual, "Running as a service"), and restart the service. Without the rule, everything else works and `wifi.connect` reports `failed` with the hint.
2. Send the app agent the draft protocol sections before they are frozen. The app shows the commands only when `hello` lists them.
3. Rollback: the previous binary. The rule then grants permissions nobody uses; remove it with `sudo rm /etc/polkit-1/rules.d/40-dlidar-network.rules`. Saved profiles stay, as ordinary NetworkManager profiles.

## Board results (2026-11-03)

- Wrong WPA2 password: `wrong_password` after 17 s (the D5 text mapping held). Wrong WPA3: untested (no WPA3 network).
- An SSID not in range: `not_found` after 28.5 s, well inside the limit.
- Success (rejoining the same network with its password): 8.6 s; the replaced profile's autoconnect priority 5 became 0.
- The BLE link survived every join; the app saw no drop.
- The polkit rule had to be `40-` (see D8).
