# device-control Specification

## Purpose

Defines how the scanning device is commanded and observed, so new input channels (buttons, phone) and output channels (LEDs, phone status) can be added without touching the recording logic.

## Requirements

### Requirement: One recording at a time
The device SHALL run at most one recording at a time and SHALL report one of the states Idle, Preparing, Recording, Finalizing or Error.

#### Scenario: Start while recording
- **WHEN** a start command arrives while a recording is Preparing, Recording or Finalizing
- **THEN** the command is rejected with the current state, and the running recording is not affected

#### Scenario: Start accepted once Idle
- **WHEN** the reported state is Idle or Error (including inside an output channel's Idle notification)
- **THEN** a start command is accepted, and the state reads Preparing as soon as the command returns

#### Scenario: Recording ends by itself
- **WHEN** the recording reaches its configured length
- **THEN** the device passes through Finalizing and returns to Idle with the recording complete

### Requirement: A failed recording ends in Error
A recording that cannot be written completely SHALL end in Error with a warning, never crash the process or hold the recording in memory without bound, and the device SHALL accept the next start.

#### Scenario: Sensor does not start
- **WHEN** the sensor sends no packet for 10 s during its start-up, or does not answer its start-up queries within 60 s
- **THEN** the device reports a warning and Error, and no recording files are created

#### Scenario: Sensor goes silent while recording
- **WHEN** the sensor sends no points for 10 s during Recording (power or connection lost, or it restarted and sends only IMU packets)
- **THEN** the capture stops, the chunks already captured are written, the files are closed, the sensor is stopped, and the device reports a warning and Error

#### Scenario: Storage fails
- **WHEN** the storage cannot open or fails while writing
- **THEN** the capture stops, what was written is closed, and the device reports a warning and Error

#### Scenario: Internal error
- **WHEN** the sensor, the storage or a memory allocation throws during a recording
- **THEN** the chunks already captured are written, the files are closed, the sensor is stopped, and the device reports a warning and Error

### Requirement: Commands through one control interface
Start, stop and status requests from every input channel SHALL go through one thread-safe control interface. A stop from any channel SHALL finalize the recording exactly as SIGINT does today.

#### Scenario: Stop from any channel
- **WHEN** a stop arrives from a signal or from any other input channel during Recording
- **THEN** the recording is finalized into complete files, and the device returns to Idle

#### Scenario: Stop during start-up
- **WHEN** a stop arrives while the sensor is starting up (Preparing)
- **THEN** the start-up ends within about a second, no recording files are created, and the device returns to Idle

#### Scenario: Second signal still kills
- **WHEN** a second SIGINT/SIGTERM/SIGHUP arrives
- **THEN** the process ends at once, and the next start recovers the recording as today

### Requirement: Status delivered to every output channel
State changes, warnings (for example a dirty lens) and the end-of-recording summary SHALL be delivered to every registered output channel. A slow or failing output channel SHALL NOT delay, stop or corrupt a recording.

#### Scenario: Several outputs
- **WHEN** two output channels are registered and a recording starts
- **THEN** both receive the Preparing and Recording state changes in that order

#### Scenario: Failing output
- **WHEN** an output channel throws or blocks
- **THEN** the recording continues and its files are byte-identical to a run without that channel

### Requirement: Adding a channel needs no core change
A new input or output channel SHALL require only a new adapter using the control interface or the status output contract, plus its registration in the startup wiring.

#### Scenario: Test channels
- **WHEN** a test registers a fake input that sends start/stop and a fake output that records events
- **THEN** the unchanged core runs a recording driven and observed only through them

### Requirement: CLI one-shot mode unchanged
Run without a daemon option, the program SHALL behave as today. It recovers leftovers, records one recording, stops on the chunk limit or a signal, then exits, with the same flags (`--finalize`, `--imu-time=auto|sensor|host`), exit codes and console messages.

#### Scenario: Plain run
- **WHEN** `bin/dLidar` is started with the L2 connected and then interrupted with Ctrl-C
- **THEN** it finalizes the recording, prints the same summary as before, and exits with code 0

#### Scenario: Finalize only
- **WHEN** `bin/dLidar --finalize` is run with a leftover temp file
- **THEN** the leftover is finalized in place, no sensor is opened, and the exit code is 0 on success
