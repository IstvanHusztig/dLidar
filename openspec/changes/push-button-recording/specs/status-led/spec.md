# Spec Delta

## Purpose

Defines the status LED of the handheld scanner as an output channel: how it is wired and which light pattern shows which device state, so the user can see in the field whether a recording runs or is being saved.

## ADDED Requirements

### Requirement: LED wiring
The LED SHALL be driven by one GPIO line, by default GPIO 27 (header pin 13), active high: the line is high when the LED is lit. The line SHALL be low whenever dLidar starts or exits.

#### Scenario: Service exits
- **WHEN** dLidar exits, after a signal, a long press or an error
- **THEN** the LED is off

#### Scenario: LED line not available
- **WHEN** the configured LED line cannot be opened
- **THEN** dLidar prints why and keeps running without the LED; recordings are not affected

### Requirement: Recording shown by a steady light
The LED SHALL be lit steadily from the moment a recording is started (Preparing) until it ends (Recording).

#### Scenario: Start by button
- **WHEN** a short press starts a recording
- **THEN** the LED lights within 0.2 s and stays lit while the sensor starts and the recording runs

### Requirement: Saving shown by blinking
While a recording is being finalized (Finalizing), the LED SHALL blink at 2 Hz, and it SHALL go off when the recording is saved and the device is Idle.

#### Scenario: Stop by button
- **WHEN** a short press stops a recording
- **THEN** the LED blinks until the LAZ is complete, then goes off

#### Scenario: Low disk space
- **WHEN** a recording is stopped because the disk is nearly full
- **THEN** the LED blinks while it is saved, then goes off

### Requirement: Failure shown by fast flashes
When the device enters Error, the LED SHALL flash three times fast (about 100 ms on, 100 ms off), then stay off.

#### Scenario: Sensor does not start
- **WHEN** a press starts a recording but the sensor is not found
- **THEN** the LED is lit during the start-up, then flashes three times and goes off

### Requirement: The LED never affects a recording
The LED SHALL be driven from its own thread, through a bounded event queue, so a slow or failing GPIO write cannot delay or stop a recording.

#### Scenario: GPIO write fails
- **WHEN** writing the LED line fails during a recording
- **THEN** the recording continues unchanged, and one warning is printed
