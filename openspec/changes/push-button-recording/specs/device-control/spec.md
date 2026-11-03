# Spec Delta

## MODIFIED Requirements

### Requirement: One recording at a time
The device SHALL run at most one recording at a time and SHALL report one of the states Idle, Preparing, Recording, Finalizing or Error.

#### Scenario: Start while recording
- **WHEN** a start command arrives while a recording is Preparing, Recording or Finalizing
- **THEN** the command is rejected with the current state, and the running recording is not affected

#### Scenario: Start accepted once Idle
- **WHEN** the reported state is Idle or Error (including inside an output channel's Idle notification)
- **THEN** a start command is accepted, and the state reads Preparing as soon as the command returns

#### Scenario: Recording ends by itself
- **WHEN** a recording ends without a stop command (low disk space)
- **THEN** the device passes through Finalizing and returns to Idle with the recording complete

#### Scenario: Recording ends on a stop command
- **WHEN** a stop command arrives during a recording
- **THEN** the device passes through Finalizing and returns to Idle with the recording complete

### Requirement: CLI one-shot mode unchanged
Run with `--once`, the program SHALL behave as the one-shot mode before service mode: it recovers leftovers, records one recording, stops on a signal or a failure, then exits, with the same flags (`--finalize`, `--imu-time=auto|sensor|host`), exit codes and console messages. Without `--once` it runs as a service (see service-mode).

#### Scenario: Plain run
- **WHEN** `bin/dLidar --once` is started with the sensor connected and then interrupted with Ctrl-C
- **THEN** it finalizes the recording, prints the same summary as before, and exits with code 0

#### Scenario: Finalize only
- **WHEN** `bin/dLidar --finalize` is run with a leftover temp file
- **THEN** the leftover is finalized in place, no sensor is opened, and the exit code is 0 on success

## ADDED Requirements

### Requirement: Recordings have no length limit
A recording SHALL run until a stop command, a signal, a sensor failure or low disk space ends it; there SHALL be no limit on its number of chunks or its duration in the device application.

#### Scenario: Long recording
- **WHEN** a recording runs for 30 minutes without a stop command
- **THEN** it is still recording, and memory use has not grown with its length

### Requirement: Low disk space stops a recording
While recording, the device SHALL check the free space of the recording disk at least once per chunk. When it falls below what finalizing the recording needs (half the size of its points temp file, plus 500 MB), the recording SHALL stop with a warning and be finalized normally, ending in Idle.

#### Scenario: Disk nearly full
- **WHEN** the free space drops below the needed amount during a recording
- **THEN** a warning names the free and needed space, the recording is finalized completely, and the state returns to Idle

#### Scenario: No space to start
- **WHEN** a recording is started while the disk has less than 500 MB free
- **THEN** the start fails with a warning, no files are written, and the state is Error
