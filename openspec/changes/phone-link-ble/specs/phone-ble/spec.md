# Spec Delta

## Purpose

Defines how a phone finds dLidar and talks protocol v1 to it over Bluetooth LE: advertising, the GATT service and its security, how protocol messages map onto writes and notifications, one phone at a time, pairing, and behaviour when Bluetooth is missing or restarts.

## ADDED Requirements

### Requirement: Discoverable while dLidar runs
While dLidar runs with `[phone] ble` true (the default) and no phone is connected, it SHALL advertise as a connectable, general-discoverable LE peripheral with the 128-bit service UUID `b06b0001-1294-4457-85c4-ef6a46d0c127` in the advertisement packet itself, and the configured name (`[phone] name`, default `dLidar`) in the advertisement or the scan response. It SHALL advertise from start-up on, also while idle and during a recording, and stop advertising when it exits.

#### Scenario: Phone scans after the scanner boots
- **WHEN** the dLidar service has started and a phone scans with a filter on the service UUID
- **THEN** the phone finds the scanner by the service UUID in its advertisement data, and shows the name `dLidar`

#### Scenario: BLE off in the settings
- **WHEN** dLidar starts with `[phone] ble = false`
- **THEN** it registers no GATT service and does not advertise, and the local socket works as before

### Requirement: GATT service
dLidar SHALL provide one primary service `b06b0001-1294-4457-85c4-ef6a46d0c127` with two characteristics:
- RX `b06b0002-1294-4457-85c4-ef6a46d0c127`, which the phone writes with write or write-without-response, handled the same;
- TX `b06b0003-1294-4457-85c4-ef6a46d0c127`, which dLidar notifies.

Both SHALL require an encrypted link.

#### Scenario: Service discovery
- **WHEN** a bonded phone connects and discovers services
- **THEN** it finds the service with RX (write, write-without-response) and TX (notify)

### Requirement: Every request needs the bond
A phone SHALL only be able to write RX or turn on TX notifications over an encrypted link, which needs a bond. An unbonded phone SHALL get an ATT security error (insufficient encryption or authentication), not a protocol message.

#### Scenario: Unbonded phone writes
- **WHEN** a phone without a bond writes a request to RX while the pairing window is closed
- **THEN** the write fails with an ATT security error, pairing fails, and dLidar sends nothing to that phone

#### Scenario: Bonded phone reconnects
- **WHEN** a phone that bonded earlier reconnects, turns on TX notifications and sends `status.get`
- **THEN** it gets the response without a new pairing prompt

### Requirement: Pairing window
New bonds SHALL be possible only while the pairing window is open: for 120 s after it is opened, Just Works (no PIN), any phone may pair. Outside the window the adapter SHALL not be pairable. The window SHALL be opened at start by `[phone] pairing_at_start = true` (default false) until the pairing button is mounted, SHALL print when it opens, when a phone pairs, and when it closes, and SHALL never delay dLidar's start or a recording.

#### Scenario: Pairing at start
- **WHEN** dLidar starts with `pairing_at_start = true` and an unbonded phone writes RX within 120 s
- **THEN** the phone pairs, the console prints that it paired, and its request is answered after the retry

#### Scenario: Another program enables pairing
- **WHEN** the pairing window is closed and another program on the Pi makes the Bluetooth adapter pairable
- **THEN** dLidar makes it not pairable again and prints a warning (once per start)

#### Scenario: Window closed
- **WHEN** 120 s after start an unbonded phone tries to pair
- **THEN** the pairing fails and the console has printed that the pairing window closed

### Requirement: Messages over writes and notifications
Over BLE the protocol's byte stream SHALL map as follows:
- dLidar joins the phone's writes in order into one stream, so a request may arrive in several writes and a write may hold several requests;
- dLidar splits each message into notifications of at most MTU−3 bytes, using the MTU negotiated on this connection, and starts each message in a fresh notification;
- until the phone's first write reports the MTU, dLidar assumes the minimum MTU of 23.

#### Scenario: Request in pieces at MTU 517
- **WHEN** a bonded phone with MTU 517 writes `{"v":1,"id":7,"cmd":"status.get"}\n` in two writes
- **THEN** it receives one response with id 7, in notifications of at most 514 bytes, the first of which starts with `{`

#### Scenario: Large message
- **WHEN** a response of 3000 bytes is sent on a connection with MTU 247
- **THEN** it arrives in 13 notifications of at most 244 bytes, and the next message starts in a new notification

### Requirement: A connection is a subscribed phone
A BLE connection SHALL open when the connected phone turns on TX notifications, and close when it turns them off, disconnects or the link is lost. Events SHALL be sent and counted only while it is open. When the phone reconnects, a new connection starts: `seq` counts from 1 again.

#### Scenario: Reconnect
- **WHEN** a phone that received events up to `seq` 12 loses the link, reconnects and turns on notifications
- **THEN** the next event it receives has `seq` 1

### Requirement: One phone at a time
dLidar SHALL serve one phone at a time over BLE:
- it stops advertising while a phone is connected, and advertises again after that phone disconnects;
- a second phone that connects anyway (for example by a cached address) is disconnected by dLidar, and never receives the first phone's messages;
- it gets no `too_many_clients` message, since a notification would also reach the first phone.

#### Scenario: Second phone
- **WHEN** a phone is connected and a second bonded phone connects by address
- **THEN** dLidar disconnects the second phone, and the first phone's connection, `seq` and requests are unaffected

#### Scenario: Advertising resumes
- **WHEN** the connected phone disconnects
- **THEN** dLidar advertises again within 2 s and the phone can reconnect

### Requirement: Idle phones are disconnected
A connected phone that has not turned on TX notifications within 30 s (of connecting, of dLidar's start, or of turning them off) SHALL be disconnected, so a leftover link never keeps the scanner from advertising.

#### Scenario: Leftover link
- **WHEN** dLidar restarts while a phone keeps its LE link to the Pi but no app subscribes
- **THEN** dLidar disconnects that phone after 30 s and advertises again, so the app finds it

### Requirement: Slow link never holds up the device
Notifications SHALL be sent at a bounded rate. A BLE connection SHALL follow the same rules as any other client:
- events are held (latest state and progress kept, the rest counted in `dropped`) while its unsent output is backlogged;
- it is disconnected when its unsent output exceeds the per-connection cap.

Neither a slow nor a stalled link SHALL delay or change a recording, and memory in dLidar and in bluetoothd stays bounded.

#### Scenario: Phone out of range during a recording
- **WHEN** the connected phone moves out of range during a recording and comes back after the recording was saved
- **THEN** the recording is identical to one without a phone, and after reconnecting `status.get` reports the saved recording

### Requirement: Bluetooth trouble only warns
A missing adapter, a stopped bluetoothd, Bluetooth switched off, or a failed registration SHALL only print a warning, once per cause. dLidar SHALL record as usual and keep the local socket working. When bluetoothd (re)appears, dLidar SHALL register the service and advertise again without a restart. No Bluetooth operation SHALL block dLidar's start, a recording, or the socket clients.

#### Scenario: Bluetooth restarted
- **WHEN** `bluetooth.service` is restarted while dLidar runs
- **THEN** dLidar prints a warning, advertises again within 10 s of bluetoothd's return, and a bonded phone can reconnect

#### Scenario: No Bluetooth at all
- **WHEN** dLidar starts on a Pi without a Bluetooth adapter (or with `bluetooth.service` stopped)
- **THEN** it prints one warning, starts as fast as without the phone link, and records with the button as usual

### Requirement: Advertised name setting
`[phone] name` SHALL be 1 to 20 printable ASCII characters (it fits the scan response); anything else stops the start with the line number, like every settings error. dLidar SHALL also give the Bluetooth adapter this name, so a connected or bonded phone shows the same name as when scanning. `[phone] ble` SHALL be `true` or `false`.

#### Scenario: Name too long
- **WHEN** the settings file has `name = "dLidar scanner number 1"` (23 characters) under `[phone]`
- **THEN** dLidar stops at start with the file name, the line number and the reason
