# Spec Delta

## Purpose

Defines how dLidar runs as the device's long-running service: what it does at start-up, how it waits for commands between recordings, and how it ends, so the scanner works when powered on without anyone starting a program.

## ADDED Requirements

### Requirement: Service waits for commands
Started without `--once` or `--finalize`, dLidar SHALL recover leftover recordings, open the configured sensor and button, and then wait in Idle. It SHALL run recordings one after another as commands arrive, until it is told to exit.

#### Scenario: Several recordings
- **WHEN** dLidar is running as a service and three recordings are started and stopped with the button
- **THEN** three complete recordings with consecutive numbers are written, and dLidar is still waiting in Idle afterwards

#### Scenario: Sensor missing at start
- **WHEN** the service starts and the sensor is not reachable when a recording is started
- **THEN** that recording ends in Error with a warning, no files are written, and the service keeps waiting for the next press

### Requirement: Signals end the service cleanly
SIGTERM, SIGINT or SIGHUP SHALL finalize a running recording and then exit with code 0; a second such signal SHALL kill the process as today. A leftover from a kill SHALL be finalized at the next start, as today.

#### Scenario: System shutdown during a recording
- **WHEN** systemd stops the service while a recording runs
- **THEN** the recording is finalized and the process exits before systemd's stop timeout

### Requirement: Started by systemd
The repository SHALL provide a systemd unit that starts dLidar at boot as the configured user, with access to the GPIO lines, restarts it after a crash, and gives it enough stop time to finalize a long recording.

#### Scenario: Boot
- **WHEN** the Pi boots with the unit enabled
- **THEN** dLidar runs and waits for the button without anyone logging in

### Requirement: One-shot mode for tests
With `--once`, dLidar SHALL behave as before this change: start one recording at once, without a button, stop it on a signal or failure, finalize it and exit.

#### Scenario: Test run over SSH
- **WHEN** `bin/dLidar --once` is started and then interrupted with Ctrl-C
- **THEN** it finalizes the recording, prints the summary and exits with code 0
