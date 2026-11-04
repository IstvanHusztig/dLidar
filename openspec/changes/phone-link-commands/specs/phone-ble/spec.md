# Spec Delta

## MODIFIED Requirements

### Requirement: Pairing window
New bonds SHALL be possible only while the pairing window is open: for 120 s after it is opened, Just Works (no PIN), any phone may pair. Outside the window the adapter SHALL not be pairable. The window SHALL be opened by a short press of the pairing button or the signal `SIGUSR1` (`tools/pair.sh`); a long press of the pairing button or the signal `SIGUSR2` (`tools/pair.sh --forget`) SHALL first remove every bonded phone, then open it. dLidar SHALL print when it opens, when a phone pairs, how many phones were forgotten, and when it closes, and SHALL never delay dLidar's start or a recording.

#### Scenario: Pairing on request
- **WHEN** `tools/pair.sh` is run and an unbonded phone writes RX within 120 s
- **THEN** the phone pairs, the console prints that it paired, and its request is answered after the retry

#### Scenario: Forget, then pair
- **WHEN** `tools/pair.sh --forget` is run for a phone that was unpaired on its side and still has a bond on the Pi
- **THEN** dLidar removes the Pi's bond, prints how many phones it forgot, opens the window, and the phone pairs anew

#### Scenario: Another program enables pairing
- **WHEN** the pairing window is closed and another program on the Pi makes the Bluetooth adapter pairable
- **THEN** dLidar makes it not pairable again and prints a warning (once per start)

#### Scenario: Window closed
- **WHEN** 120 s after the window opened an unbonded phone tries to pair
- **THEN** the pairing fails and the console has printed that the pairing window closed
