# Spec Delta

## Purpose

Lets a connected client (a bonded phone over BLE, or a local socket client) put the scanner on a Wi-Fi network the phone found: the `wifi.connect` and `wifi.status` commands, how networks are saved and restored, and what is never touched or logged.

## ADDED Requirements

### Requirement: wifi.connect command
`wifi.connect` with `ssid`, `security` and, except for `open`, `psk` SHALL join that Wi-Fi network without the scanner scanning for networks itself. It SHALL answer at once with `{"accepted":true,"timeout_s":T}`, T being the most seconds the attempt takes; the outcome SHALL follow as one `result` event (see "wifi.connect result").

#### Scenario: Join a network
- **WHEN** a client sends `{"v":1,"id":12,"cmd":"wifi.connect","args":{"ssid":"Home","security":"wpa2-psk","psk":"secret123"}}` and the network is in range with that password
- **THEN** it receives `{"id":12,"ok":true,"result":{"accepted":true,"timeout_s":60}}`, then within 60 s `{"event":"result","seq":N,"op":12,"ok":true,"result":{"ssid":"Home","ip":"192.168.1.50"}}`, and the scanner is on that network

#### Scenario: Open network
- **WHEN** a client sends `wifi.connect` with `"security":"open"` and no `psk` for an open network in range
- **THEN** the scanner joins it and the result reports `ssid` and `ip`

### Requirement: wifi.connect arguments checked
`wifi.connect` SHALL answer `bad_request`, naming the argument, and change nothing when:
- `ssid` is missing, empty or over 32 bytes;
- `security` is not `wpa2-psk`, `wpa3-sae` or `open`;
- `psk` is missing for `wpa2-psk` or `wpa3-sae`;
- `psk` is given for `open`;
- `psk` is neither 8–63 printable ASCII characters nor 64 hexadecimal digits.

#### Scenario: Short password
- **WHEN** a client sends `wifi.connect` with `"security":"wpa2-psk"` and a 7-character `psk`
- **THEN** it receives `bad_request` naming `psk`, and the message does not contain the password

#### Scenario: Unknown security
- **WHEN** a client sends `"security":"wep"`
- **THEN** it receives `bad_request` naming `security`

### Requirement: wifi.connect result
The `result` event SHALL be `ok:true` with `ssid` and `ip` (the scanner's IPv4 address on that network, `null` if it got none) when the scanner joined. Otherwise it SHALL be `ok:false` with one of these error codes:
- `wrong_password`: the network rejected the password;
- `not_found`: the network was not in range;
- `timeout`: no outcome within `timeout_s`;
- `failed`: any other reason, with a readable message.

#### Scenario: Wrong password
- **WHEN** a client sends `wifi.connect` for a WPA2 network in range with a wrong password
- **THEN** the result is `ok:false` with code `wrong_password`

#### Scenario: Network not in range
- **WHEN** a client sends `wifi.connect` for a name that no network in range has
- **THEN** the result is `ok:false` with code `not_found`, or `timeout` if the network could not be ruled out within `timeout_s`

### Requirement: The result goes to the requester
The `result` event SHALL go only to the connection that sent the `wifi.connect`. It SHALL never be left out, even when that connection is behind on its events. If that connection is gone, the result SHALL only be logged.

#### Scenario: Only the requester gets the result
- **WHEN** two local clients are connected and one sends `wifi.connect`
- **THEN** only that client receives the `result` event, and the other client's event `seq` is unaffected

### Requirement: One attempt at a time
While a `wifi.connect` is running, another `wifi.connect` from any client SHALL answer the error `busy` and SHALL NOT affect the running attempt.

#### Scenario: Second connect
- **WHEN** a `wifi.connect` is running and a client sends another one
- **THEN** it receives `ok:false` with code `busy`, and the first attempt's result still arrives

### Requirement: wifi.status command
`wifi.status` (no arguments) SHALL answer at once with:
- `state`: `connected`, `connecting`, `disconnected` or `unavailable`;
- `ssid` and `ip`: the current network and address, `null` when not connected;
- `last`: the most recent `wifi.connect` since dLidar started (`ssid` plus `result`: `pending` while it runs, then `connected` or an error code), or `null`.

`unavailable` means there is no Wi-Fi device, or NetworkManager cannot be reached.

#### Scenario: After a lost link
- **WHEN** the BLE link drops while a `wifi.connect` runs, and the phone reconnects after the attempt ended
- **THEN** `wifi.status` reports `last` with that network and its outcome, so the phone can show it without the lost `result` event

#### Scenario: During an attempt
- **WHEN** a client sends `wifi.status` while a `wifi.connect` runs
- **THEN** `state` is `connecting`, `last` is `{"ssid":…,"result":"pending"}`, and the answer comes without waiting for the attempt

### Requirement: A network is saved only when joined
A network SHALL be saved, and kept across reboots, only after a `wifi.connect` to it succeeded. It SHALL be saved next to the already saved networks. A saved infrastructure Wi-Fi profile with the same network name SHALL be replaced by the new one.

#### Scenario: A second network
- **WHEN** the scanner has `Home` saved and joins `Site` through `wifi.connect`
- **THEN** both `Home` and `Site` are saved, and after a reboot the scanner joins whichever is in range

#### Scenario: New password for a saved network
- **WHEN** `Home` is saved and a `wifi.connect` to `Home` with a new password succeeds
- **THEN** one `Home` profile remains, with the new password

#### Scenario: The network the scanner is on
- **WHEN** the scanner is on `Home` and a `wifi.connect` to `Home` comes, with the same or another password
- **THEN** the scanner joins again in full (a short Wi-Fi drop), and on success one `Home` profile remains with the password sent

### Requirement: A failed attempt changes nothing
After a `wifi.connect` ends in any error, nothing the attempt created SHALL remain saved. The saved networks SHALL be as before. The Wi-Fi network the scanner was on before the attempt SHALL be joined again, if it is still in range.

#### Scenario: Wrong password while reached over Wi-Fi
- **WHEN** the scanner is on `Home` and a client sends `wifi.connect` for `Site` with a wrong password
- **THEN** the result is `wrong_password`, no `Site` profile is saved, the `Home` profile is unchanged, and the scanner is back on `Home`

#### Scenario: Wrong password for the current network
- **WHEN** the scanner is on `Home` and a client sends `wifi.connect` for `Home` with a wrong password
- **THEN** the result is `wrong_password`, the saved `Home` profile keeps its working password, and the scanner is back on `Home`

### Requirement: Other connections are never touched
`wifi.connect` SHALL only add, replace or activate infrastructure Wi-Fi profiles on the Wi-Fi device. It SHALL NOT change, deactivate or delete:
- the sensor's Ethernet connection;
- a Wi-Fi profile with another network name;
- an access point (hotspot) profile.

#### Scenario: During a recording
- **WHEN** a recording runs from the sensor's Ethernet and a client joins another Wi-Fi network
- **THEN** the recording continues, its data is unaffected, and the Ethernet connection is unchanged

### Requirement: Passwords are never disclosed
A password SHALL NOT appear in any console or journal line, error message, response or event. Logs and messages SHALL name the network only.

#### Scenario: Password in nothing dLidar emits
- **WHEN** clients send `wifi.connect` with a marker password, in every outcome (joined, wrong password, bad arguments, busy)
- **THEN** the marker appears in none of dLidar's output lines and none of the messages sent to any client

### Requirement: Wi-Fi requests and results are logged
Each `wifi.connect` SHALL print one console line naming the client kind, the network and the security type (for example `phone: wifi.connect Home (wpa2-psk)`). Its outcome SHALL print one line, for example `Wi-Fi: connected to Home, 192.168.1.50` or `Wi-Fi: Home: wrong password`.

#### Scenario: Journal shows the outcome
- **WHEN** a phone joins the scanner to `Home` while dLidar runs as the service
- **THEN** `journalctl -u dlidar` shows the request line and the `connected` line with the address

### Requirement: Missing permission is reported
When the system refuses dLidar the right to add or activate a network connection, `wifi.connect` SHALL end in `failed`. The message SHALL name the missing polkit rule (`systemd/50-dlidar-network.rules`). Nothing SHALL change.

#### Scenario: Rule not installed
- **WHEN** the service runs without the polkit rule and a client sends `wifi.connect`
- **THEN** the result is `ok:false`, code `failed`, with a message naming the rule, and the saved networks are unchanged

### Requirement: Commands are listed
`hello` SHALL list `wifi.connect` and `wifi.status` among its commands, and the protocol SHALL stay version 1: these are additions only.

#### Scenario: hello
- **WHEN** a client sends `hello`
- **THEN** `commands` contains `wifi.connect` and `wifi.status` besides the existing commands
