# Spec Delta

## Purpose

Defines the push button of the handheld scanner as an input channel: how it is wired and read, and what a short and a long press do, so the device can be operated in the field without a laptop.

## ADDED Requirements

### Requirement: Button wiring and debounce
The button SHALL be read from one GPIO line, by default GPIO 17 (header pin 11), wired to GND with the Pi's internal pull-up, so a low level means pressed. Level changes shorter than the debounce time (50 ms) SHALL be ignored.

#### Scenario: Contact bounce
- **WHEN** the line toggles several times within 20 ms on one press
- **THEN** exactly one press is recognised

#### Scenario: GPIO not available
- **WHEN** the configured line cannot be opened (no permission, wrong pin, line in use)
- **THEN** dLidar prints why, names the `gpio` group and the `[button] gpio` setting, and exits with an error before opening the sensor

### Requirement: Short press starts and stops a recording
A press released within 5 s SHALL start a recording when the device is Idle or Error, and SHALL stop the running recording (which is then finalized) when it is Preparing or Recording. A short press while Finalizing SHALL be ignored.

#### Scenario: Start and stop
- **WHEN** the device is Idle and the button is pressed briefly, and later pressed briefly again
- **THEN** a recording starts with the first press and is finalized after the second, and the device returns to Idle

#### Scenario: Press during finalization
- **WHEN** the button is pressed briefly while a recording is Finalizing
- **THEN** nothing changes and the console says the press was ignored

### Requirement: Long press shuts the device down
Holding the button for 5 s SHALL stop a running recording, wait until it is finalized, and then power the Pi down. It SHALL act after 5 s of holding, without waiting for the release.

#### Scenario: Long press while recording
- **WHEN** the button is held for 5 s during a recording
- **THEN** the recording is finalized completely before the power-down starts

#### Scenario: Power-down not permitted
- **WHEN** the power-down request is refused (the polkit rule is not installed)
- **THEN** dLidar prints a warning naming the missing rule and keeps running in Idle
