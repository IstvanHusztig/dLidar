# Spec Delta

## Purpose

Defines the protocol commands with which a connected client (a bonded phone over BLE, or a local socket client) changes what the device does: starting and stopping a recording, their answers and errors, and how they are logged.

## ADDED Requirements

### Requirement: recording.start command
`recording.start` (no arguments) SHALL start a recording exactly as a short press of the record button does in Idle or Error. It SHALL answer at once with `{"accepted":true,"state":S}` when it started one, where S is the state it saw (`idle` or `error`). The state events (`preparing`, `recording`, …) SHALL follow as for any recording.

#### Scenario: Start from the phone
- **WHEN** the device is idle and a bonded phone sends `{"v":1,"id":5,"cmd":"recording.start"}`
- **THEN** it receives `{"id":5,"ok":true,"result":{"accepted":true,"state":"idle"}}`, then the events `preparing` and `recording`, and the recording is the same as one started with the button

#### Scenario: Start after a failure
- **WHEN** the last recording ended in error and a client sends `recording.start`
- **THEN** the answer is `accepted:true` with `"state":"error"`, and a new recording starts

### Requirement: Nothing to do is not an error
A command that would not change anything SHALL answer `ok:true` with `"accepted":false` and the state it saw:
- `recording.start` while `preparing` or `recording`;
- `recording.stop` while `idle`, `error` or `finalizing`.

#### Scenario: Start while recording
- **WHEN** a recording runs and a client sends `recording.start`
- **THEN** the answer is `{"accepted":false,"state":"recording"}` and the running recording goes on unaffected

#### Scenario: Stop while idle
- **WHEN** the device is idle and a client sends `recording.stop`
- **THEN** the answer is `{"accepted":false,"state":"idle"}`

### Requirement: Busy while saving
`recording.start` while the last recording is being saved (`finalizing`) SHALL answer the error `busy` with a readable message, so the client can try again later.

#### Scenario: Start while saving
- **WHEN** a recording is being saved and a client sends `recording.start` with id 9
- **THEN** it receives `{"id":9,"ok":false,"error":{"code":"busy","message":…}}` and no new recording starts

### Requirement: recording.stop command
`recording.stop` (no arguments) SHALL stop the running recording exactly as a short press of the record button does in Preparing or Recording: the files are saved normally. It SHALL answer at once with `{"accepted":true,"state":S}`, S being the state it saw (`preparing` or `recording`); the `finalizing` and final state events follow.

#### Scenario: Stop from the phone
- **WHEN** a recording runs and a client sends `recording.stop`
- **THEN** it receives `accepted:true` with `"state":"recording"`, then the events `finalizing` and `idle`, and the files are complete

#### Scenario: Stop during the start-up
- **WHEN** a recording is preparing and a client sends `recording.stop`
- **THEN** the answer is `accepted:true` with `"state":"preparing"`, the start-up is abandoned, and no files are written

### Requirement: Every connected client may command
Any client the phone link serves SHALL be allowed both commands: a phone over BLE (which is always bonded) and a local socket client. Commands from several clients SHALL be handled one at a time in arrival order, through the same control interface as the button and the signals.

#### Scenario: Phone and button together
- **WHEN** the phone sends `recording.stop` and the record button is pressed at the same moment during a recording
- **THEN** the recording is stopped and saved once, and the phone gets one answer

### Requirement: Commands are logged and listed
Each accepted or refused command SHALL print one console line naming the client kind and the outcome (for example `phone: start` or `phone: start ignored, recording`). `hello` SHALL list `recording.start` and `recording.stop` among its commands.

#### Scenario: Journal shows who started
- **WHEN** a recording is started from the phone while dLidar runs as the service
- **THEN** `journalctl -u dlidar` shows `phone: start` before the recording's own messages
