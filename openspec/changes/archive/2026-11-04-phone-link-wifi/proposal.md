# Proposal

## Why

To put the scanner on a Wi-Fi network today, someone needs a shell on the Pi, or must edit the SD card. In the field (a cave entrance, a site office, a phone hotspot), the phone is the only screen the user has. The phone link can already watch and command the scanner over Bluetooth LE (`phone-link-core`, `-ble`, `-commands`), so it is the natural place to set up Wi-Fi.

The roadmap fixed the shape of this phase (user decisions, 2026-11-02 and 2026-11-03):
- The Pi never scans: the app scans and lists the networks, the user picks one and types the password, and the app sends the network name, password and security type.
- A new network is **added** to the saved ones.

## What Changes

- **New command `wifi.connect`** `{ssid, security, psk}`, with `security` one of `wpa2-psk`, `wpa3-sae` or `open` (no `psk`):
  - It answers at once with `{"accepted":true,"timeout_s":60}`. The result comes later as event `result`, as `docs/phone-protocol.md` "Results that come later" already defines.
  - The result is either `connected` with the network name and the scanner's IP address, or one of the errors `wrong_password`, `not_found`, `timeout` and `failed` (a catch-all with a readable message).
  - A second `wifi.connect` while one runs answers `busy`. Bad arguments answer `bad_request` and name the argument:
    - a network name that is empty or longer than 32 bytes;
    - an unknown security type;
    - a password that is not 8–63 printable ASCII characters or 64 hex digits;
    - a password given for an open network.
- **New command `wifi.status`**:
  - It answers at once with the Wi-Fi state: `connected`, `connecting`, `disconnected` or `unavailable` (no Wi-Fi device, or NetworkManager not running).
  - It also gives the network name and the IP address.
  - It gives the outcome of the last `wifi.connect`. A phone whose Bluetooth link dropped while the Pi was joining reconnects and reads the outcome here: the result event is lost with the link.
- **A failed attempt changes nothing.** dLidar saves the network only when the connect succeeds. After a wrong password, a missing network or a timeout:
  - the attempt is removed;
  - the Wi-Fi network the Pi was on before is joined again.

  A wrong password therefore never strands a scanner that is reached over Wi-Fi.
- **Saved networks:**
  - Every network joined this way is saved and kept across reboots, next to the existing ones. NetworkManager joins whichever saved network is in range.
  - A network sent again with a new password replaces the saved one with the same name.
  - dLidar never changes any other network profile: not the sensor's Ethernet (`lidar`), not other Wi-Fi networks, not a hotspot profile.
- **dLidar drives NetworkManager through `nmcli`**, as the roadmap planned. It does not use `nmcli device wifi connect`, which rescans: it adds the profile and activates it, so the Pi never scans.
- **Passwords are never logged or echoed.** They appear in no console line, no journal line, no error message and no event. They do reach `nmcli`'s command line, which is acceptable because the scanner has one user (user, 2026-11-03).
- **Permission:** a new polkit rule `systemd/40-dlidar-network.rules` lets the service user add and activate NetworkManager connections, the same pattern as the power-off rule. Without it, `wifi.connect` ends in `failed`, with a message that names the rule.
- **Logs:** each request and result is a console line, e.g. `phone: wifi.connect Home (wpa2-psk)` and `Wi-Fi: connected to Home, 192.168.1.50`. These lines carry the network name only, never the password.
- **Docs:**
  - `docs/phone-protocol.md`: both commands, the result codes and the timeout;
  - the manual: installing the rule, and setting up Wi-Fi from the app;
  - `docs/architecture.md`, `CLAUDE.md` and `README.md`.

  The app agent gets the draft contract before it is frozen.

## Capabilities

### New Capabilities
- `phone-wifi`: Wi-Fi setup by a connected client:
  - the `wifi.connect` and `wifi.status` commands;
  - their arguments, answers, later results and errors;
  - how networks are saved, replaced and restored after a failure;
  - what may never be touched or logged;
  - the permission dLidar needs.

### Modified Capabilities

None. The main specs (`device-control`, `phone-ble`, `sensor-integration`) keep their requirements. `wifi.connect` is the first user of the "Results that come later" requirement, which `phone-link-core` already defined for exactly this phase.

## Impact

- **Code:**
  - `src/application/ports/wifi_manager.h`: the port.
  - `src/application/remote/wifi_setup.h`:
    - argument rules;
    - one attempt at a time;
    - the timeout;
    - the last outcome.
  - A new adapter `src/adapters/networkmanager/`, CMake target `dlidar_network`, with no new library. It has its own thread, which runs `nmcli` behind a small command-runner interface, so tests replace NetworkManager with scripted answers.
  - `src/adapters/phone/`: `PhoneServer` gets the two commands, and the result event goes back to the connection that asked. `PhoneLink::Start` takes the Wi-Fi setup.
  - `src/app/main.cpp` wires it.
  - No change to `src/domain`, `RecordSession`, `ScannerController` or the storage adapter.
- **Recordings and dMapper:** no effect. Wi-Fi setup is allowed during a recording: it never touches the sensor's Ethernet, and the recording does not use Wi-Fi. Golden replay stays identical.
- **System:**
  - a new polkit rule to install, a step added to the manual's installation;
  - NetworkManager 1.42 on the dev board; the Wi-Fi profiles go to `/etc/NetworkManager/system-connections/`, readable by root only.
- **The app:** two new commands in the existing message format; nothing existing changes.
- **Dev board testing:** the dev board is reached over SSH on Wi-Fi, so joining another network moves it off the developer's network. The board tests are planned around that (design, Risks).
