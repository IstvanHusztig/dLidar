# dLidar phone protocol, version 1

The contract between dLidar (the scanner) and a client such as the Android app. It is the same over every transport: the local Unix socket (now) and Bluetooth LE (next). The requirements are in `openspec/changes/phone-link-core/specs/phone-api/spec.md`; this document explains them for client authors.

Version 1 lets a client query and watch the scanner, start and stop recordings, put the scanner on a Wi-Fi network, and read its log.

## Framing

- Every message is one compact UTF-8 JSON object followed by `\n`. There is no raw newline inside a message (newlines in text are escaped as `\n` by JSON).
- A message from a client is at most **4096 bytes including the `\n`**. A message from dLidar is at most **16384 bytes including the `\n`**.
- dLidar treats everything a client writes on one connection as one byte stream: a write may hold several messages or part of one. Collect bytes until `\n`, then parse.
- A client line over 4096 bytes is answered with error `too_large` (`"id":null`); dLidar discards it up to the next `\n` and keeps the connection open.

## Messages

In the examples, `→` is a message from the client and `←` one from dLidar.

**Request:** `v` (protocol version, required), `id` (an integer 0 to 2147483647 chosen by the client), `cmd`, and optional `args` (an object).

```
→ {"v":1,"id":7,"cmd":"status.get"}
```

**Response:** exactly one per request, with the same `id`. `ok:true` always has a `result` object (possibly empty).

```
← {"id":7,"ok":true,"result":{"state":"idle","chunks":0,"points":0,"last_ok":true,"location":"/home/pi/PointCloudDump/","free_bytes":61203456000,"warnings":[],"summary":null}}
← {"id":8,"ok":false,"error":{"code":"unknown_command","message":"unknown command \"reboot\""}}
```

**Event:** pushed without a request. `seq` counts the events sent on this connection from 1; `dropped`, when present, is the number of events left out just before this one. `data` is always an object.

```
← {"event":"progress","seq":5,"data":{"chunks":12,"points":168000}}
← {"event":"state","seq":9,"dropped":3,"data":{"state":"idle","chunks":40,"points":560000,"last_ok":true,"location":"/home/pi/PointCloudDump/"}}
```

## Ordering and ids

- dLidar handles a connection's requests one at a time, and responses come in request order. Several requests may be in flight.
- No command waits for the device: every response comes within milliseconds unless the link stalls. A timeout of about 5 s is right; after a timeout, call `status.get` rather than reconnect. A response that arrives after its timeout can be dropped.
- Ids are scoped to the connection, and dLidar never checks them for reuse. Counting from 1 on each connection and wrapping is fine.
- An error with `"id":null` caused by a line (`too_large`, or `bad_request` when no valid id could be read) takes that line's place in the response order: the client may complete its **oldest pending request** with it. `too_many_clients` belongs to no request.

```
→ {"v":1,"id":5,"cmd":"info.get"}
→ this is not JSON
→ {"v":1,"id":6,"cmd":"hello"}
← {"id":5,"ok":true,"result":{"version":"0e1e413","sensor":"unitree_l2","sensor_model":"unitree_l2","recording_format":3,"protocol":1}}
← {"id":null,"ok":false,"error":{"code":"bad_request","message":"not valid JSON"}}
← {"id":6,"ok":true,"result":{"protocol":1,"server":"dLidar","commands":["hello","status.get","info.get","recording.start","recording.stop","wifi.connect","wifi.status","log.get"],"events":["state","warning","progress","summary"],"limits":{"max_request_bytes":4096,"max_message_bytes":16384}}}
```

## Events and gaps

- Nothing is sent when a connection opens; the first event is `seq:1`, sent at the next change. **Call `status.get` after every (re)connect.**
- Over BLE a connection opens when the client turns on TX notifications; events before that are neither sent nor counted.
- `seq` never jumps: events that are left out get no `seq`. `dropped` on the next event is the loss signal; then call `status.get`. dLidar always keeps a client's latest state and latest progress, so the next events after a gap are current.
- `progress` comes at most once per second. The last `progress` of a recording comes before the `finalizing` state; none comes after it. The recording's totals (its last, partial chunk included) are in the final `idle` or `error` state.

## Errors

| Code | When | `id` |
|---|---|---|
| `bad_request` | not JSON, not an object, `v`/`id`/`cmd` missing or of the wrong type or range, `args` not an object, an argument the command does not define (the message names it) | the request's id, or `null` if none could be read |
| `unknown_command` | `cmd` is not a command of this server | the request's id |
| `unsupported_version` | `v` is not 1; the error carries `"supported":[1]` | the request's id |
| `busy` | not now: `recording.start` while the last recording is still being saved (`finalizing`), try again once the state is `idle`; `wifi.connect` while another Wi-Fi join is running, try again after its result | the request's id |
| `unavailable` | `log.get`: the scanner's log cannot be read (the message says why) | the request's id |
| `too_large` | a line over 4096 bytes | `null` |
| `too_many_clients` | another client is already connected; the connection is then closed | `null` |

```
→ {"v":2,"id":3,"cmd":"hello"}
← {"id":3,"ok":false,"error":{"code":"unsupported_version","message":"protocol version 2 is not supported","supported":[1]}}
→ {"v":1,"id":4,"cmd":"status.get","args":{"verbose":true}}
← {"id":4,"ok":false,"error":{"code":"bad_request","message":"unknown argument \"verbose\" for status.get"}}
← {"id":null,"ok":false,"error":{"code":"too_many_clients","message":"another client is connected"}}
```

`supported` lists every version the scanner speaks. If the client's version is above the highest, tell the user to update the scanner; if it is below the lowest, to update the app.

## Commands

### `hello`

Optional; a client is never required to send it first. Lists the server's commands and events and its limits.

```
→ {"v":1,"id":1,"cmd":"hello"}
← {"id":1,"ok":true,"result":{"protocol":1,"server":"dLidar","commands":["hello","status.get","info.get","recording.start","recording.stop","wifi.connect","wifi.status","log.get"],"events":["state","warning","progress","summary"],"limits":{"max_request_bytes":4096,"max_message_bytes":16384}}}
```

### `status.get`

| Field | Meaning |
|---|---|
| `state` | `idle`, `preparing` (sensor start-up), `recording`, `finalizing` (files being finished) or `error`. `error` lasts until the next recording starts; a start is accepted in `error` as in `idle`. |
| `chunks`, `points` | of the current or last recording (unsigned 64-bit for `points`) |
| `last_ok` | the last finished recording was saved completely; `true` before the first recording |
| `location` | the absolute path of the output directory, never empty |
| `free_bytes` | free space there, or `null` if it cannot be read |
| `warnings` | the current or last recording's warnings (`kind`, `text`), at most the 16 newest, emptied when a recording starts. The reason for `error` is among them. |
| `summary` | the last recording's summary (as in the `summary` event), or `null` |

The state is never older than a `state` event the client already received.

```
→ {"v":1,"id":2,"cmd":"status.get"}
← {"id":2,"ok":true,"result":{"state":"error","chunks":0,"points":0,"last_ok":false,"location":"/home/pi/PointCloudDump/","free_bytes":null,"warnings":[{"kind":"sensor_failed","text":"The sensor did not start."}],"summary":null}}
```

### `info.get`

| Field | Meaning |
|---|---|
| `version` | the dLidar build (`git describe`) |
| `sensor` | the configured sensor: `unitree_l2` or `livox_mid360s` |
| `sensor_model` | the sensor model, as in `timingNNNN.json`: `unitree_l2`, `livox_mid360s` (today the same as `sensor`; a model name may differ from the sensor id later) |
| `recording_format` | the recording format version (3) |
| `protocol` | 1 |
| `hostname` | the scanner's short host name (`gethostname()` up to the first `.`), e.g. `dlidar-01`. Optional: older builds leave it out, and it is left out when the name cannot be read. |

```
→ {"v":1,"id":3,"cmd":"info.get"}
← {"id":3,"ok":true,"result":{"version":"0e1e413-dirty","sensor":"livox_mid360s","sensor_model":"livox_mid360s","recording_format":3,"protocol":1,"hostname":"dlidar-01"}}
```

### `recording.start`

Starts a recording, as a short press of the record button does in `idle` or `error`. No arguments. The answer comes at once and carries `accepted` and `state`, the state the command saw:

| Answer | When |
|---|---|
| `accepted:true` | the state was `idle` or `error`; a recording starts, and the `state` events `preparing`, `recording`, … follow |
| `accepted:false` | nothing to do: a recording is already `preparing` or `recording`, and goes on unaffected |
| error `busy` | the last recording is still `finalizing`; nothing starts. Try again once the state is `idle`. |

```
→ {"v":1,"id":5,"cmd":"recording.start"}
← {"id":5,"ok":true,"result":{"accepted":true,"state":"idle"}}
→ {"v":1,"id":6,"cmd":"recording.start"}
← {"id":6,"ok":true,"result":{"accepted":false,"state":"recording"}}
→ {"v":1,"id":9,"cmd":"recording.start"}
← {"id":9,"ok":false,"error":{"code":"busy","message":"the last recording is still being saved; try again when it is idle"}}
```

A start from `error` answers `{"accepted":true,"state":"error"}`. Whether the sensor started is told by the events (`recording`, or `error` with a `sensor_failed` warning), not by the answer.

### `recording.stop`

Stops the running recording, as a short press of the record button does in `preparing` or `recording`: the files are saved normally. No arguments.

| Answer | When |
|---|---|
| `accepted:true` | the state was `recording` (then `finalizing` and `idle` follow) or `preparing` (the start-up is abandoned, no files are written, and `idle` follows) |
| `accepted:false` | nothing to do: the state was `idle`, `error` or `finalizing` |

```
→ {"v":1,"id":7,"cmd":"recording.stop"}
← {"id":7,"ok":true,"result":{"accepted":true,"state":"recording"}}
→ {"v":1,"id":8,"cmd":"recording.stop"}
← {"id":8,"ok":true,"result":{"accepted":false,"state":"idle"}}
```

Every client may send both commands: the bonded phone over BLE and a local socket client. They are handled one at a time with the record button, so a stop from the phone and a button press at the same moment save the recording once. dLidar prints one line per command (`phone: start`, `local client: stop ignored, idle`).

### `wifi.connect`

Puts the scanner on a Wi-Fi network that the phone found by its own scan: the scanner never scans. Arguments:

| Argument | |
|---|---|
| `ssid` | the network name, 1 to 32 bytes of UTF-8 (hidden networks and names that are not UTF-8 are not supported) |
| `security` | `wpa2-psk` (also for a WPA2/WPA3 transition network), `wpa3-sae` or `open` |
| `psk` | the password: 8 to 63 printable ASCII characters, or 64 hexadecimal digits. Required for `wpa2-psk` and `wpa3-sae`; leave it out for `open`. |

The answer comes at once: `{"accepted":true,"timeout_s":60}`. The outcome follows as one `result` event ("Results that come later"), within `timeout_s` seconds:

| Result | When |
|---|---|
| `ok:true`, `result` `{"ssid","ip"}` | the scanner joined; `ip` is its IPv4 address there (`null` if it got none) |
| error `wrong_password` | the network rejected the password |
| error `not_found` | the network is not in range. The scanner relies on its last periodic scan, so a network that is not there may also end in `timeout`. |
| error `timeout` | no outcome within `timeout_s` |
| error `failed` | any other reason; the message says which (for example the polkit rule missing on the scanner, NetworkManager not running, no Wi-Fi device). Show it as it is. |

```
→ {"v":1,"id":20,"cmd":"wifi.connect","args":{"ssid":"Home","security":"wpa2-psk","psk":"secret123"}}
← {"id":20,"ok":true,"result":{"accepted":true,"timeout_s":60}}
← {"event":"result","seq":4,"op":20,"ok":true,"result":{"ssid":"Home","ip":"192.168.1.50"}}
→ {"v":1,"id":21,"cmd":"wifi.connect","args":{"ssid":"Site","security":"open"}}
← {"id":21,"ok":false,"error":{"code":"busy","message":"another Wi-Fi join is still running; try again when it has ended"}}
→ {"v":1,"id":22,"cmd":"wifi.connect","args":{"ssid":"Home","security":"wpa2-psk","psk":"short"}}
← {"id":22,"ok":false,"error":{"code":"bad_request","message":"\"psk\" must be 8 to 63 printable ASCII characters or 64 hexadecimal digits"}}
```

Other `bad_request` cases name the argument the same way: `ssid` missing, empty or over 32 bytes; `security` missing or unknown; `psk` missing for a secured network or given for an open one; an argument that is not a string. The message never contains the password.

What the scanner does with the network:
- It saves the network only when the join succeeded, next to the networks it already knows, and joins whichever saved network is in range after a reboot. A network sent again under the same name replaces the saved one: this is how its password is changed.
- After any error nothing is saved, the saved networks are unchanged, and the scanner rejoins the network it was on before, all within `timeout_s`. A wrong password therefore never cuts off a scanner that is reached over Wi-Fi.
- The network the scanner is already on is joined again in full, with a short Wi-Fi drop.
- It never touches the sensor's Ethernet connection, so a join during a recording is allowed.
- The password appears in no log line, no message and no event.
- Bluetooth and Wi-Fi share the Pi's radio: the BLE link may drop while the scanner joins. Reconnect through the bond and send `wifi.status`: `last` gives the outcome of the lost `result` event.

dLidar prints one line for the request (`phone: wifi.connect Home (wpa2-psk)`) and one for the outcome (`Wi-Fi: connected to Home, 192.168.1.50`, `Wi-Fi: Home: wrong password`).

### `wifi.status`

No arguments; the answer comes at once.

| Field | Meaning |
|---|---|
| `state` | `connected`, `connecting` (a `wifi.connect` is running, or the scanner is joining a saved network by itself), `disconnected` or `unavailable` (no Wi-Fi device, or NetworkManager is not running: do not offer Wi-Fi setup) |
| `ssid`, `ip` | the network and the scanner's IPv4 address on it; while a `wifi.connect` runs, `ssid` is the network being joined. `null` when there is none. |
| `last` | the most recent `wifi.connect` since dLidar started: `ssid` and `result`, which is `pending` while it runs, then `connected` or the result's error code. `null` if there was none. |

```
→ {"v":1,"id":23,"cmd":"wifi.status"}
← {"id":23,"ok":true,"result":{"state":"connecting","ssid":"Home","ip":null,"last":{"ssid":"Home","result":"pending"}}}
→ {"v":1,"id":24,"cmd":"wifi.status"}
← {"id":24,"ok":true,"result":{"state":"connected","ssid":"Home","ip":"192.168.1.50","last":{"ssid":"Home","result":"connected"}}}
```

### `log.get`

Reads the scanner's log: the lines of the dLidar service, and systemd's lines about it (started, stopped, exited, crashed, restarted), newest first, one page per request. Nothing else from the system is ever included. Lines of a dLidar started by hand (over SSH) go to its terminal and are not in the log.

| Argument | Meaning |
|---|---|
| `before` | optional: the `more` of an earlier answer; the page then starts just older than that page's last line. Without it the page starts at the newest line. |
| `limit` | optional: at most this many lines, an integer 1–200, default 50 |
| `level` | optional: the lowest level returned, `info` (default, every line), `warning` (warnings and errors) or `error` |

The answer has `entries` and `more`. Each entry:

| Field | Meaning |
|---|---|
| `t` | the time in Unix milliseconds, from the scanner's clock. The Pi has no battery-backed clock, so offline the time can be wrong or jump at a reboot: order lines by their position, not by `t`. |
| `boot` | a short id of the boot the line was written in; it changes at each reboot, so a client can mark restarts |
| `level` | `info`, `warning` (dLidar's lines starting `WARNING:`, systemd's warnings) or `error` (dLidar's `ERROR:` lines, systemd's errors) |
| `source` | `dlidar` (the service's own lines) or `system` (systemd's lines about the service) |
| `text` | the line, at most 1024 bytes of UTF-8 |

`more` is the cursor for the next, older page, an opaque string (about 130 characters), or `null` at the start of the log. Pages do not shift while new lines arrive: paging with `before` until `more` is `null` returns every line once. To see new lines, ask again without `before` and drop the lines you already hold.

A page always fits the 16384-byte message limit, and one request reads a bounded part of the log. So a page may hold fewer lines than `limit`, even none (a rare `level` over a long stretch), while `more` is not `null`: keep paging. A `before` that is not in the form of a cursor answers `bad_request`; a cursor whose line was removed by the log's rotation continues at the newest remaining older line.

```
→ {"v":1,"id":30,"cmd":"log.get","args":{"limit":3}}
← {"id":30,"ok":true,"result":{"entries":[{"t":1793781326123,"boot":"36276c97","level":"info","source":"dlidar","text":"Ready: press the button (GPIO 17) to start a recording, again to save it; hold it 5 s to power down."},{"t":1793781325870,"boot":"36276c97","level":"warning","source":"dlidar","text":"WARNING: Bluetooth (bluetoothd) is gone; the phone link waits for it"},{"t":1793781320410,"boot":"36276c97","level":"info","source":"system","text":"Started dlidar.service - dLidar LiDAR scanner (push button recording, phone link)."}],"more":"s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fa91;b=36276c9730b3433b91d86044fa35fc8a;m=8f1e2a;t=65f6ee2e1c4a2;x=7d1c0e5f3a9b2c44"}}
→ {"v":1,"id":31,"cmd":"log.get","args":{"before":"s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fa91;b=36276c9730b3433b91d86044fa35fc8a;m=8f1e2a;t=65f6ee2e1c4a2;x=7d1c0e5f3a9b2c44","limit":2}}
← {"id":31,"ok":true,"result":{"entries":[{"t":1793781315402,"boot":"36276c97","level":"info","source":"system","text":"dlidar.service: Scheduled restart job, restart counter is at 1."},{"t":1793781310398,"boot":"36276c97","level":"warning","source":"system","text":"dlidar.service: Failed with result 'signal'."}],"more":"s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fa8e;b=36276c9730b3433b91d86044fa35fc8a;m=8b0c11;t=65f6ee2d4f8a0;x=1a2b3c4d5e6f7081"}}
→ {"v":1,"id":32,"cmd":"log.get","args":{"level":"warning","limit":1}}
← {"id":32,"ok":true,"result":{"entries":[{"t":1793781325870,"boot":"36276c97","level":"warning","source":"dlidar","text":"WARNING: Bluetooth (bluetoothd) is gone; the phone link waits for it"}],"more":"s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fac7;b=36276c9730b3433b91d86044fa35fc8a;m=c1efbf;t=65f6ee330bd6f;x=22fe7aa7b597862e"}}
→ {"v":1,"id":33,"cmd":"log.get","args":{"limit":500}}
← {"id":33,"ok":false,"error":{"code":"bad_request","message":"\"limit\" must be an integer from 1 to 200"}}
→ {"v":1,"id":34,"cmd":"log.get"}
← {"id":34,"ok":false,"error":{"code":"unavailable","message":"cannot open the journal: Permission denied"}}
```

dLidar prints nothing for `log.get`, so reading the log adds no lines to it.

## Events

| Event | `data` |
|---|---|
| `state` | each state change: `state`, `chunks`, `points`, `last_ok`, `location` |
| `warning` | `kind` and `text`: show the text as it is, it is complete and readable |
| `progress` | `chunks` and `points`, at most once per second |
| `summary` | at the end of a recording: `imu_mode` (`sensor` = IMU times from the sensor's clock, the normal case; `host_aligned` = rebuilt from arrival times, a fallback; `auto` = too short to decide), `decision` (a readable reason, show as it is), `imu_in`, `imu_out`, `lost_samples` |

Warning kinds: `lens_dirty`, `storage_failed`, `storage_slow`, `storage_low_space`, `sensor_failed`, `sensor_silent`, `recording_failed`. Warnings report events, not conditions: there is no "cleared" event.

A whole recording, as a connected client sees it:

```
← {"event":"state","seq":1,"data":{"state":"preparing","chunks":0,"points":0,"last_ok":true,"location":"/home/pi/PointCloudDump/"}}
← {"event":"state","seq":2,"data":{"state":"recording","chunks":0,"points":0,"last_ok":true,"location":"/home/pi/PointCloudDump/"}}
← {"event":"progress","seq":3,"data":{"chunks":14,"points":196000}}
← {"event":"warning","seq":4,"data":{"kind":"lens_dirty","text":"The lens is 7% dirty; clean it."}}
← {"event":"progress","seq":5,"data":{"chunks":28,"points":392000}}
← {"event":"state","seq":6,"data":{"state":"finalizing","chunks":28,"points":392000,"last_ok":true,"location":"/home/pi/PointCloudDump/"}}
← {"event":"summary","seq":7,"data":{"imu_mode":"sensor","decision":"sensor stamps increase, evenly spaced, host rate","imu_in":10000,"imu_out":10000,"lost_samples":0}}
← {"event":"state","seq":8,"data":{"state":"idle","chunks":28,"points":392000,"last_ok":true,"location":"/home/pi/PointCloudDump/"}}
```

## Results that come later

Used by `wifi.connect`. `recording.start` and `recording.stop` are not of this kind: their answer is final, and the `state` events show what follows. A command whose result takes time answers `{"accepted":true}` at once; the result comes later as event `result`, with `op` (that request's id) at the top level and `ok` plus `result` or `error` as in a response, and no `data`. It goes only to the connection that sent the request and is never left out; if that connection is lost first, the result is lost, and the command's status query gives the outcome.

```
→ {"v":1,"id":12,"cmd":"wifi.connect","args":{"ssid":"Home","psk":"secret123","security":"wpa2-psk"}}
← {"id":12,"ok":true,"result":{"accepted":true,"timeout_s":60}}
← {"event":"result","seq":14,"op":12,"ok":false,"error":{"code":"wrong_password","message":"the network rejected the password"}}
```

## Evolution rules for clients

Within protocol version 1, dLidar only adds: commands, events, result fields, enum values and error codes. Nothing is removed, renamed or changed in meaning or unit. So a client must:

- ignore unknown fields and unknown events;
- treat an unknown `state` or warning `kind` as a generic one, and an unknown error code as a generic error;
- never send arguments the scanner did not document: an unknown argument is rejected with `bad_request`, so an older scanner never silently ignores an option a newer app relies on.

## Transports

**Local socket (now).** A Unix stream socket `dlidar.sock` in the service's runtime directory (`/run/dlidar/dlidar.sock` under systemd), else in `$XDG_RUNTIME_DIR`, mode 0600, at most 4 clients. `tools/phone_client.py` is a command-line client:

```bash
tools/phone_client.py status.get
tools/phone_client.py --follow            # print events
```

**Bluetooth LE.** dLidar advertises while it runs and no phone is connected (`[phone] ble`, on by default). The 128-bit service UUID is in the advertisement data; the name (`[phone] name`, default `dLidar`) is in the scan response. **Match the scanner by the UUID.** Once bonded, the phone shows the same name: dLidar gives the Pi's Bluetooth adapter that name too.

| | UUID | |
|---|---|---|
| Service | `b06b0001-1294-4457-85c4-ef6a46d0c127` | |
| RX | `b06b0002-1294-4457-85c4-ef6a46d0c127` | the client writes here (write; write-without-response is handled the same); encrypted link only |
| TX | `b06b0003-1294-4457-85c4-ef6a46d0c127` | dLidar notifies here; encrypted link only |

- **Every request and every event needs the bond.** RX and TX both require an encrypted link. An unbonded phone gets an ATT security error, and Android shows its pairing prompt by itself (already when subscribing to TX). Pairing (Just Works, no PIN) only succeeds while the scanner's pairing window is open: the pairing button, and until it is mounted `tools/pair.sh` on the Pi (the signal `SIGUSR1`), which opens it for 2 minutes. A phone that was unpaired on the phone can pair again inside the window.
- **One phone at a time.** Advertising stops while a phone is connected and resumes within 0.5 s after it leaves. A second phone that connects anyway (by a cached address) is disconnected. Over BLE it does **not** get `too_many_clients`: a notification would also reach the first phone.
- **Subscribe first.** Turn on TX notifications before the first write. A connection opens when notifications go on, and `seq` counts from 1 on every new connection. A write while notifications are off is rejected (ATT error). A connected phone that has not turned on notifications within 30 s is disconnected, so a leftover link never hides the scanner.
- **Pieces.** dLidar splits each message into notifications of at most MTU−3 bytes of this connection, and a message always starts in a fresh notification. Until the phone's first write reports the MTU, dLidar assumes 23. It sets no MTU limit of its own (Android 14+ asks for 517). The client splits its writes the same way; long writes (an offset) are rejected.
- **Pace.** dLidar sends at most 24 KiB/s (measured: 54 KiB/s over the air at MTU 517, no loss). A phone that does not take its notifications gets no events until it does (its latest state and progress kept, the rest counted in `dropped`), and is disconnected past 64 KiB of unsent bytes.
- There is no idle timeout once subscribed and no keep-alive; BLE's supervision timeout detects a lost link. Reconnect through the bond. When bluetoothd restarts, dLidar registers again at once.
