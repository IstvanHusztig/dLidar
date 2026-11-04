# Spec Delta

## Purpose

Defines the pairing button: a second push button on the Pi that opens the Bluetooth pairing window or forgets every bonded phone, its wiring and setting, and how the status LED shows that the pairing window is open.

## ADDED Requirements

### Requirement: Pairing button wiring and setting
The pairing button SHALL be a push button between a GPIO line and GND, read with the internal pull-up and debounced like the record button. Its line SHALL be `[pairing_button] gpio` (default 22, header pin 15), and SHALL differ from the record button's and the LED's lines; the same line twice stops the start with the line number, like every settings error.

#### Scenario: Default wiring
- **WHEN** a push button connects header pin 15 (GPIO 22) to GND and dLidar starts with no `[pairing_button]` section
- **THEN** dLidar reads that button as the pairing button

#### Scenario: Same line as the record button
- **WHEN** the settings file has `gpio = 17` under `[pairing_button]` while the record button uses GPIO 17
- **THEN** dLidar stops at start with the file name, the line number and the reason

### Requirement: Pairing button is optional
A pairing button that cannot be used (the line is busy or missing) SHALL only print a warning; dLidar SHALL run as usual, and the pairing window SHALL still open with `tools/pair.sh`. A button that is not wired SHALL simply never be pressed.

#### Scenario: Line busy
- **WHEN** another program holds GPIO 22 at dLidar's start
- **THEN** dLidar prints a warning naming `[pairing_button] gpio` and runs with the record button and the phone link

### Requirement: Short press opens the pairing window
A short press SHALL open the 2-minute pairing window, as `tools/pair.sh` does; a press while it is open SHALL open it anew for 2 minutes. It SHALL not change a running recording.

#### Scenario: Pair a phone in the field
- **WHEN** the user presses the pairing button and connects an unbonded phone with the app within 2 minutes
- **THEN** the phone pairs and the app works, without a shell on the Pi

### Requirement: Long press forgets every phone
Holding the pairing button for 5 s SHALL remove every bonded phone from the Pi and then open the pairing window, as `tools/pair.sh --forget` does. The console SHALL print how many phones were forgotten. It SHALL not change a running recording.

#### Scenario: Phone unpaired on its side
- **WHEN** the user has unpaired the scanner on the phone, holds the pairing button 5 s, and then connects the phone with the app
- **THEN** the old bond on the Pi is gone, the phone pairs anew, and the console printed `Bluetooth: forgot 1 phone`

#### Scenario: Lost phone
- **WHEN** a phone was lost and the user holds the pairing button 5 s
- **THEN** the lost phone can no longer connect, and the user's new phone can pair within 2 minutes

### Requirement: Pairing window shown on the LED
While the pairing window is open and no recording is preparing, running or being saved, the status LED SHALL flash twice every second (two flashes of 100 ms, 100 ms apart). Recording, saving and failure patterns SHALL keep priority; after the failure flashes the pairing pattern SHALL show again while the window is open.

#### Scenario: Window open while idle
- **WHEN** the device is idle and the pairing window opens
- **THEN** the LED flashes twice every second until the window closes, then stays off

#### Scenario: Window opened during a recording
- **WHEN** a recording runs and the pairing button is pressed
- **THEN** the LED stays on (recording), and the pairing window is open for 2 minutes
