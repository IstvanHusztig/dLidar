# Spec Delta

## Purpose

Defines the message protocol between dLidar and a phone app: how requests, responses and events look, which commands and events exist, and how a client is kept from disturbing a recording. The same protocol runs over every transport (local socket now, Bluetooth LE later).

## ADDED Requirements

### Requirement: Newline-delimited JSON messages
Every message SHALL be one compact UTF-8 JSON object followed by a newline, with no raw newline inside it. A message from a client SHALL be at most 4096 bytes including the newline; a message from dLidar SHALL be at most 16384 bytes including the newline. dLidar SHALL treat everything a client writes on one connection as one byte stream, so a write may hold several messages or part of one.

#### Scenario: Two messages in one write
- **WHEN** a client sends two complete requests in one write
- **THEN** dLidar answers both, in order

#### Scenario: Message split across writes
- **WHEN** a request arrives in several pieces, with the newline in the last one
- **THEN** dLidar answers it once, after the last piece

#### Scenario: Oversized message
- **WHEN** a client sends more than 4096 bytes without a newline
- **THEN** dLidar sends error `too_large` with `"id":null`, discards the data up to the next newline and keeps the connection open

### Requirement: Request and response correlation
A request SHALL be an object with `v` (protocol version, required on every request including `hello`), `id` (an integer 0 to 2147483647 chosen by the client), `cmd` (a string) and optional `args` (an object). Each request SHALL get exactly one response with the same `id`, and a client's responses SHALL arrive in the order of its requests. A successful response SHALL always carry a `result` object, possibly empty. Ids are scoped to the connection and dLidar does not check them for reuse.

#### Scenario: Successful request
- **WHEN** a client sends `{"v":1,"id":7,"cmd":"status.get"}`
- **THEN** it receives one response `{"id":7,"ok":true,"result":{…}}`

#### Scenario: Pipelined requests
- **WHEN** a client sends `hello`, `info.get` and `status.get` without waiting
- **THEN** the three responses arrive in that order with their ids

#### Scenario: Unknown command
- **WHEN** a client sends a request whose `cmd` dLidar does not know
- **THEN** the response is `{"id":N,"ok":false,"error":{"code":"unknown_command","message":…}}`

#### Scenario: Malformed request
- **WHEN** a line is not valid JSON, or lacks `v`, `id` or `cmd`, or one of them has the wrong type or range
- **THEN** dLidar answers with error code `bad_request` (with `"id":null` if no valid id could be read) and keeps the connection open

### Requirement: Requests handled one at a time
dLidar SHALL handle a client's requests one at a time, and no command SHALL wait for the device: a command whose result comes later answers `{"accepted":true}` at once. An error sent with `"id":null` because of a line (`too_large`, or `bad_request` without a readable id) SHALL take that line's place in the response order.

#### Scenario: Unreadable line between requests
- **WHEN** a client sends request 5, a line that is not JSON, and request 6
- **THEN** it receives the response to 5, then a `bad_request` error with `"id":null`, then the response to 6

### Requirement: Unknown arguments rejected
A request SHALL be rejected with error `bad_request` naming the argument when `args` contains a name the command does not define, so an older dLidar never silently ignores an option a newer app relies on.

#### Scenario: Argument from a newer app
- **WHEN** a client sends `{"v":1,"id":3,"cmd":"status.get","args":{"verbose":true}}`
- **THEN** the response is error `bad_request` and its message names `verbose`

### Requirement: Protocol version
dLidar SHALL implement protocol version 1. A request whose `v` is not supported SHALL get error `unsupported_version` with `"supported":[1]` in the error object.

#### Scenario: Newer app
- **WHEN** a client sends a request with `"v":2`
- **THEN** the response has error code `unsupported_version` and `"supported":[1]`

### Requirement: Compatible evolution
Within a protocol version, commands, events, result fields, enum values and error codes SHALL only be added, never removed, renamed or changed in meaning or unit. The protocol document SHALL require clients to ignore unknown fields and events and to treat unknown enum values and error codes as generic ones.

#### Scenario: Later phase adds a state or a command
- **WHEN** a later dLidar adds a command, a result field, a warning kind or an error code
- **THEN** an app built against the protocol v1 document keeps working unchanged

### Requirement: hello command
`hello` SHALL return `protocol` (1), `server` ("dLidar"), `commands` and `events` (the supported names) and `limits` (`max_request_bytes`, `max_message_bytes`). A client SHALL NOT be required to send `hello` first.

#### Scenario: Discover the server
- **WHEN** a client sends `hello`
- **THEN** the result has `"protocol":1`, `"server":"dLidar"`, `commands` containing `hello`, `status.get` and `info.get`, `events` containing `state`, `warning`, `progress` and `summary`, and `"max_request_bytes":4096`

### Requirement: status.get command
`status.get` SHALL return `state` (`idle`, `preparing`, `recording`, `finalizing` or `error`), `chunks` and `points` of the current or last recording, `last_ok` (it was saved completely), `location` (the absolute path of the output directory), `free_bytes` (free space there, or null if it cannot be read), `warnings` and `summary`. `last_ok` is true before the first recording. The state `error` lasts until the next recording starts.

#### Scenario: Status while recording
- **WHEN** a client sends `status.get` during a recording
- **THEN** the result has `"state":"recording"` and `chunks` and `points` at least those of the last `progress` event it received

#### Scenario: Before the first recording
- **WHEN** dLidar has just started and a client sends `status.get`
- **THEN** the result has `"state":"idle"`, `"last_ok":true`, `"warnings":[]` and `"summary":null`

#### Scenario: Status after a failure
- **WHEN** the last recording ended in Error and a client sends `status.get`
- **THEN** the result has `"state":"error"`, `"last_ok":false` and the warning that ended it in `warnings`

#### Scenario: Reconnect after a recording
- **WHEN** a client connects after a recording that raised a `lens_dirty` warning was saved
- **THEN** `status.get` has that warning in `warnings` and the recording's `summary`

### Requirement: Warnings and summary recoverable
`status.get` SHALL return `warnings` (the current or last recording's warnings as `kind` and `text`, at most the 16 newest, emptied when a recording starts) and `summary` (the last recording's summary as in the `summary` event, or null), so a client that missed the events can recover them. The reason for the state `error` is among these warnings.

#### Scenario: Missed warning
- **WHEN** a recording raises `storage_slow` while no client is connected, and a client then sends `status.get`
- **THEN** `warnings` contains that `storage_slow` warning with its text

### Requirement: Status never older than delivered events
The state in a `status.get` result SHALL be at least as new as every `state` event the same client received before that response.

#### Scenario: Request while events are queued
- **WHEN** a recording starts and a client sends `status.get` immediately after receiving the `preparing` event
- **THEN** the result's state is `preparing` or a later state, never `idle` from before the start

### Requirement: info.get command
`info.get` SHALL return `version` (the dLidar build's version string), `sensor` (the configured sensor's id, `unitree_l2` or `livox_mid360s`), `sensor_model` (its name as in `timingNNNN.json`), `recording_format` (the recording format version) and `protocol` (the protocol version).

#### Scenario: Which sensor
- **WHEN** a client sends `info.get` on a device configured for the Mid-360S
- **THEN** the result has `"sensor":"livox_mid360s"`, `"sensor_model":"livox_mid360s"` and `"recording_format":3`

### Requirement: Events pushed to every client
Every connected client SHALL receive events `{"event":NAME,"seq":N,"data":{…}}` without asking, where `seq` counts the events sent on that connection from 1; `data` SHALL always be an object:
- `state`: each state change, with `state`, `chunks`, `points`, `last_ok`, `location`;
- `warning`: `kind` and `text`;
- `progress`: `chunks` and `points`;
- `summary`: `imu_mode` (`sensor`, `host_aligned` or `auto`), `decision`, `imu_in`, `imu_out` and `lost_samples` at the end of a recording.

#### Scenario: Watching a recording
- **WHEN** a client is connected and a recording runs from start to save
- **THEN** it receives `state` events for preparing, recording, finalizing and idle in that order, `progress` events in between, a `summary` event, and `seq` increases by one from event to event

#### Scenario: Warning
- **WHEN** the lens is reported dirty during a recording
- **THEN** every connected client receives a `warning` event with kind `lens_dirty` and the same text as the console

### Requirement: Event sequence numbers
`seq` SHALL increase by one from event to event on a connection; events that are left out get no `seq` and are reported by `dropped`. No event SHALL be sent when a connection opens: a client calls `status.get`. Over BLE a connection opens when the client turns on TX notifications, so events before that are neither sent nor counted.

#### Scenario: New connection
- **WHEN** a client connects while dLidar is idle, and a recording starts a minute later
- **THEN** the first event it receives is `state` `preparing` with `"seq":1`

### Requirement: Results that come later
A command that answers `{"accepted":true}` SHALL deliver its result later as event `result`, with top-level `"op"` (that request's id) and `ok` plus `result` or `error` as in a response, and no `data`. It SHALL be sent only to the connection that sent the request and SHALL never be left out.

#### Scenario: Wi-Fi connect (phase 4)
- **WHEN** a client sends `wifi.connect` with id 12 and the scanner then connects
- **THEN** it receives `{"id":12,"ok":true,"result":{"accepted":true}}` and later `{"event":"result","seq":N,"op":12,"ok":true,"result":{…}}`

### Requirement: Progress rate limited
`progress` events SHALL be sent at most once per second per client, carrying the latest values; a recording's last progress SHALL be sent before its `finalizing` state event, and none after it; the final `idle` or `error` state SHALL carry the recording's totals, its last partial chunk included.

#### Scenario: Fast sensor
- **WHEN** the Mid-360S writes about 14 chunks per second for 10 s
- **THEN** a client receives at most 11 `progress` events in that time, the last one comes before `finalizing`, and the `idle` state has the recording's final chunk and point counts

### Requirement: Event loss is visible
When events for a client must be left out because it cannot take them in time, dLidar SHALL keep that client's latest `state` and latest `progress`, SHALL set `dropped` (the number left out since the previous event) on the next event sent, and SHALL NOT drop responses.

#### Scenario: Slow client catches up
- **WHEN** a client stops reading during a recording and resumes after it was saved
- **THEN** the next event it receives has a `dropped` count above 0, and it receives the final `idle` state

### Requirement: Read-only commands in this phase
The commands of this phase SHALL NOT change the device's state; a recording SHALL be started or stopped only by the existing input channels.

#### Scenario: Only queries
- **WHEN** a client sends any command of this phase during a recording
- **THEN** the recording and its files are unaffected

### Requirement: Clients never affect a recording
A slow, stalled, disconnecting or misbehaving client SHALL NOT delay, stop or change a recording, and the memory used for one client SHALL be bounded. A client whose unsent output exceeds its bound SHALL be disconnected.

#### Scenario: Client stops reading
- **WHEN** a client connects, keeps sending requests and never reads while a recording runs
- **THEN** the recording's data is identical to a run without the client, other clients still receive their events, and the stalled client is eventually disconnected

#### Scenario: Client disconnects mid-request
- **WHEN** a client disconnects with a partial message or unread responses
- **THEN** dLidar closes that connection and nothing else changes

### Requirement: Local socket transport
dLidar SHALL accept clients on a Unix stream socket `dlidar.sock` in the service's runtime directory, else in `$XDG_RUNTIME_DIR`, with mode 0600, and at most 4 clients at a time. It SHALL remove the socket on a clean exit. Without a usable directory it SHALL print a warning and run without the socket.

#### Scenario: Fifth client
- **WHEN** four clients are connected and a fifth connects
- **THEN** the fifth receives error `too_many_clients` with `"id":null` and is closed; the others are unaffected

#### Scenario: Leftover socket file
- **WHEN** dLidar starts and a `dlidar.sock` from a killed run exists that no process answers on
- **THEN** it is replaced and clients can connect

#### Scenario: Socket in use
- **WHEN** dLidar starts and another running dLidar answers on `dlidar.sock`
- **THEN** the new dLidar prints a warning and runs without the socket, leaving the other one's socket in place

#### Scenario: No runtime directory
- **WHEN** neither the service's runtime directory nor `$XDG_RUNTIME_DIR` is usable
- **THEN** dLidar prints a warning naming the reason and records with the button as usual

### Requirement: Final state reaches clients at shutdown
On a normal exit (SIGTERM, SIGINT, `--once` finished) dLidar SHALL try for up to 200 ms to deliver pending events, including the final `idle` or `error` state, before closing the connections.

#### Scenario: Service stopped during a recording
- **WHEN** a client is connected and the service receives SIGTERM during a recording
- **THEN** the client receives `finalizing` and then `idle` (or `error`) before the connection closes
