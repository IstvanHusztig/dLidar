# dLidar phone protocol, version 1

The contract between dLidar (the scanner) and a client such as the Android app. It is the same over every transport: the local Unix socket (now) and Bluetooth LE (next). The requirements are in `openspec/changes/phone-link-core/specs/phone-api/spec.md`; this document explains them for client authors.

Version 1 is read-only: a client can query the scanner and watch it, but not start or stop recordings yet.

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
← {"id":5,"ok":true,"result":{"version":"0e1e413","sensor":"unitree_l2","sensor_model":"Unitree L2","recording_format":3,"protocol":1}}
← {"id":null,"ok":false,"error":{"code":"bad_request","message":"not valid JSON"}}
← {"id":6,"ok":true,"result":{"protocol":1,"server":"dLidar","commands":["hello","status.get","info.get"],"events":["state","warning","progress","summary"],"limits":{"max_request_bytes":4096,"max_message_bytes":16384}}}
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
← {"id":1,"ok":true,"result":{"protocol":1,"server":"dLidar","commands":["hello","status.get","info.get"],"events":["state","warning","progress","summary"],"limits":{"max_request_bytes":4096,"max_message_bytes":16384}}}
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
| `sensor_model` | its name, as in `timingNNNN.json`: `Unitree L2`, `Livox Mid-360S` |
| `recording_format` | the recording format version (3) |
| `protocol` | 1 |

```
→ {"v":1,"id":3,"cmd":"info.get"}
← {"id":3,"ok":true,"result":{"version":"0e1e413-dirty","sensor":"livox_mid360s","sensor_model":"Livox Mid-360S","recording_format":3,"protocol":1}}
```

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

Used from a later phase on (`wifi.connect`). A command whose result takes time answers `{"accepted":true}` at once; the result comes later as event `result`, with `op` (that request's id) at the top level and `ok` plus `result` or `error` as in a response, and no `data`. It goes only to the connection that sent the request and is never left out; if that connection is lost first, the result is lost, and the command's status query gives the outcome.

```
→ {"v":1,"id":12,"cmd":"wifi.connect","args":{"ssid":"Home","psk":"secret123","security":"wpa2-psk"}}
← {"id":12,"ok":true,"result":{"accepted":true}}
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

**Bluetooth LE (next, `phone-link-ble`).** Advertised with the 128-bit service UUID in the advertisement packet itself (the name `dLidar` may move to the scan response); match the scanner by UUID.

| | UUID | |
|---|---|---|
| Service | `b06b0001-1294-4457-85c4-ef6a46d0c127` | |
| RX | `b06b0002-1294-4457-85c4-ef6a46d0c127` | the client writes here (write; write-without-response is handled the same) |
| TX | `b06b0003-1294-4457-85c4-ef6a46d0c127` | dLidar notifies here |

- One phone at a time: advertising stops while one is connected, and a second gets `too_many_clients`.
- Turn on TX notifications before the first write.
- dLidar splits each message into notifications of at most MTU−3 bytes of this connection, and a message always starts in a fresh notification. The client splits its writes the same way. dLidar sets no MTU limit of its own (Android 14+ asks for 517).
- **Every request needs the bond:** RX requires an encrypted link, so an unbonded phone gets an ATT "insufficient encryption" error on its first write and Android starts pairing (Just Works). Pairing only succeeds while the scanner's pairing window is open (the pairing button; until it is mounted, `[phone] pairing_at_start` in `dlidar.toml` opens it for 2 minutes when dLidar starts).
- There is no idle timeout and no keep-alive; BLE's supervision timeout detects a lost link. Reconnect through the bond.
