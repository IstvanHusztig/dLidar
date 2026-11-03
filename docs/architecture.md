# dLidar architecture

dLidar is the application of a handheld scanning device. It records a LiDAR with an IMU (a Unitree L2 or a Livox Mid-360S, chosen in the settings file) into files that dMapper, our SLAM solution, processes. Over time the device gains buttons, status LEDs, a Bluetooth link to a phone, and possibly a different LiDAR. The code follows Clean Architecture (Robert C. Martin) so that those can be added without touching the recording logic.

The change that introduced this structure is `openspec/changes/clean-architecture-sensor-port/`.

## Layers and the dependency rule

```
src/
  domain/        entities and pure rules: no I/O, no vendor code
  application/   use cases and ports (interfaces the use cases need)
  adapters/      implementations of the ports: sensor, storage, console
  app/           composition root: configuration and wiring (main.cpp)
```

Dependencies point inward only: `app` → `adapters` → `application` → `domain`.

| Layer | May include | Never |
|---|---|---|
| `domain/` | C/C++ standard headers, `domain/` | anything else |
| `application/` | the above + `application/` | adapters, `app/`, vendor or OS device headers |
| `adapters/<x>/` | the core + its own directory (incl. its `vendor/`) and its vendor library's headers | `app/`, another adapter |
| `app/` | anything | |

Enforcement:
- **Build:** the CMake targets give each layer only what it needs.
  - `dlidar_core` (domain + application) has only `src/` on its include path and links nothing.
  - `dlidar_l2` adds the Unitree SDK, `dlidar_livox` adds Livox-SDK2 (submodule `external/Livox-SDK2`, its static library linked privately), `dlidar_storage` adds LASzip and `3rd/`, `dlidar_phone` adds nlohmann/json (`external/json`) and GLib/GIO, privately, `dlidar_gpio`, `dlidar_network` and `dlidar_console` add nothing.
  - The core and its tests therefore build without the SDK or LASzip.
- **Includes:** `tools/check_architecture.py --strict` (ctest `architecture`) checks every `#include` under `src/` against the table above.

## What lives where

**Domain (`src/domain/`)**
- `recording/`: the recorded data and the fixed file records.
  - `PointRecord`: 48 bytes, recording format 2.
  - `ImuSample`: an `imuNNNN.csv` row.
  - `PacketTiming`: a 40-byte `packetsNNNN.bin` record.
  - `SensorChunk`, `RecordingInfo` (with `kRecordingFormatVersion` and the timing JSON keys).
- `timing/`: IMU time alignment (`ImuTimeline`, `LowerEdge`, `ImuTimestamper`); see `docs/imu_lidar_time_alignment.md`.
- `geometry/`:
  - `AxisMap`, the device's mount: a signed axis permutation applied by select-and-negate, so values keep their exact bits.
  - `PointFilter`: the range and intensity rule.

**Application (`src/application/`)**
- `ports/`:
  - `LidarSensor`: the sensor contract.
  - `RecordingStorage`; `HasRoom` (after each `Append`, on the writer thread) stops a recording that would not fit on the disk, which is then saved normally.
  - `DeviceEvents`: an output channel.
  - `DeviceControl`: the commands an input channel gives (`Status`, `StartRecording`, `StopRecording`); `ScannerController` implements it.
  - `StorageSpace`: free space where the recordings go (`FileRecordingStorage` implements it).
  - `WifiManager`: the scanner's Wi-Fi (`Link()`, a cached value; `Connect(network, limit, done)`, which saves a network only when joined and calls `done` once); `NmcliWifi` implements it.
  - `StopToken`.
- `capture/`:
  - `CaptureTiming`: owns the time zero; applies the mount, m/s² → g and the filter; feeds `ImuTimeline`.
  - `ChunkAssembler`.
  - `RecordSession`: one recording, with a writer thread so the SD card never stalls packet reading.
  - `LensMonitor`.
- `device/`:
  - `ScannerController`: the state machine and the command interface.
  - `EventHub`: fans events out to the output channels.
- `remote/`: `DeviceApi`, the typed commands a remote client (the phone) may give, on `DeviceControl` and `StorageSpace`, independent of any transport or message format; `DeviceInfo`, `kProtocolVersion`. `Start()` / `Stop()` decide by the state they see and return a `CommandResult` (`accepted`, `state_seen`, `busy` for a start while saving); the check and the start are one `StartRecording` call, so they never race the button. `WifiSetup`: Wi-Fi setup by a remote client on `WifiManager`: the `wifi.connect` argument rules (`Check`, never echoing a value), one join at a time (`Begin`: `Accepted` or `Busy`, `kWifiConnectLimit` = 60 s), and what `wifi.status` reports (the link, `connecting` while its join runs, `last`: pending, then the outcome).
- `analysis/`: `ReplayPackets` (`bin/replay_packets`).

**Adapters (`src/adapters/`)**
- `unitree_l2/`: `UnitreeL2Sensor` over an `L2PacketSource`.
  - `LiveL2Source` is the SDK reader, with the start-up sequence and the 2 s backlog drain.
  - `SyntheticL2Source` replays a packet log.
  - `l2_conversion` and `l2_packet_state` handle the L2's packets.
  - `vendor/` holds Unitree's unmodified headers.
- `livox_mid360s/`: `LivoxMid360sSensor` over a `LivoxPacketSource` (`docs/livox_mid360s.md`).
  - `LiveLivoxSource` is the Livox-SDK2 reader: callbacks on the SDK's thread into a `LivoxPacketPool` that never drops, a best-effort start-up (commands may go unanswered) and the 2 s backlog drop.
  - `SyntheticLivoxSource` replays a packet log as Mid-360S packets (`livox_packet_builder.h`).
  - `livox_conversion` and `livox_packet_state` handle the Mid-360S packets (only the SDK's packed structs).
- `storage/`: `FileRecordingStorage`, `ContinuousDataWriter`, the LAZ encoder (the only LASzip user), recovery, packet log reading, and the crash guard.
- `console/`: `ConsoleEvents`, the texts `bin/dLidar` prints.
- `gpio/`: the push button and the status LED on the Pi's header (`docs/manual.html`, "Wiring and pinout"). `GpioLine` is the only code that sees `<linux/gpio.h>` (kernel GPIO uAPI v2, no library).
  - `ButtonInput` (input channel): its own thread, `PressDetector` (pure: debounce, short and long press), callbacks.
  - `StatusLed` (output channel, `hub.AddAsync`): its own 50 ms thread, `LedOn` (pure: on while a recording starts and runs, 2 Hz while saving, three flashes after a failure, two short flashes a second while the pairing window is open and nothing else shows). `SetPairing(bool)` is told by the phone link (`BleOptions::on_pairing`, bound in `main.cpp`); pairing is not device state, so it is not a `DeviceEvents` call.
- `networkmanager/` (CMake `dlidar_network`): `NmcliWifi` implements `WifiManager` through `nmcli` (openspec `phone-link-wifi`). Its own worker thread runs one job at a time: a join, or a refresh of the cached link every 5 s.
  - A join never scans. It adds an in-memory profile (`save no`, autoconnect off) and activates it. When joined, it saves the profile (`modify` autoconnect on) and deletes the older infrastructure profiles of that SSID (`ProfilesToReplace`). Otherwise it deletes the attempt and rejoins the previous profile. It never changes the sensor's Ethernet, another SSID or an access point.
  - `CommandRunner` is its only window to the system. `ProcessRunner` uses `posix_spawnp`, `LC_ALL=C`, no inherited descriptors, a timeout and `Cancel()`; it never logs an argument vector, which may hold the password. Tests script the answers.
  - `nmcli_helpers` are pure: the add arguments, outcomes from nmcli's exit codes and texts (the polkit hint for missing rights), and the terse parsers.
- `phone/` (CMake `dlidar_phone`): the phone link, protocol v1 (`docs/phone-protocol.md`), over the local socket and Bluetooth LE. The only code that sees nlohmann/json (`protocol_codec.cpp`) and GLib/GIO.
  - `PhoneEvents` (output channel, `hub.AddAsync`) writes into a `Mailbox` (bounded, progress coalesced, the latest state never dropped) and wakes the loop.
  - `PhoneLoop`: the link's own thread with a private `GMainContext`; every transport and the server run on it.
  - `PhoneServer`: framing (`LineFramer`), the command table over `DeviceApi` (`recording.start` / `recording.stop` too, each logged with the client's `Connection::Kind()`: `phone: start`, `local client: stop ignored, idle`; with a `WifiSetup`, `wifi.connect` / `wifi.status`, whose result comes back through `Post` (`PhoneLoop::Invoke` behind a gate that closes at shutdown) and goes as a `result` event to the requesting connection only, never held back), per-connection `seq` and `dropped`, progress at most 1 Hz; `status.get` answers from the events the loop has seen, so it is never older than an event already sent.
  - `SocketTransport` (a `Connection` per client): the local Unix socket `dlidar.sock` (`$RUNTIME_DIRECTORY`, else `$XDG_RUNTIME_DIR`), mode 0600, at most 4 clients, 64 KiB unsent bytes per client before it is closed.
  - `BleLink` (pure, unit-tested with fakes): the BLE transport's logic. One phone at a time, its `Connection` (open while TX notifications are on), MTU−3 pieces with the MTU from the phone's writes, a token bucket at 24 KiB/s, the 64 KiB cap, advertising off while a phone is connected, an idle phone disconnected after 30 s.
  - `BluezPeripheral` (BlueZ over D-Bus, asynchronous calls only): the GATT application (RX `encrypt-write`, TX `notify` + `encrypt-notify`), the advertisement, the adapter's name, the pairing window (Just Works agent, 120 s; `on_pairing` when it really opens and closes), `ForgetPhones` (`RemoveDevice` for every bonded device, then the window), and the `Pairable` guard (never pairable outside the window). It follows bluetoothd and the adapter and recovers on its own; problems only warn.
  - `PhoneLink`: what `main.cpp` starts and shuts down. BLE starts next to the socket, never when another dLidar answers on the socket. `OpenPairingWindow(forget)` is the one entry for the pairing button, `SIGUSR1` and `SIGUSR2`.

**App (`src/app/`)**
- `main.cpp`: CLI flags, the settings, recovery of leftovers, building the adapters (also the phone link with its `[phone]` settings, and `NmcliWifi` with `WifiSetup` for it; the Wi-Fi adapter is stopped before the phone link). Service mode (default, `systemd/dlidar.service`): waits for the button until a signal, with the optional pairing button (a second `ButtonInput`; a busy line only warns); `--once`: one recording at once.
- `button_commands.h`: what a short and a long press do (start/stop by state; save, then `PowerOff()` from `power.cpp`).
- `pairing_button_commands.h`: what the pairing button does (short: `OpenPairingWindow(false)`; 5 s: `OpenPairingWindow(true)`, forget first; without a phone link a warning).
- `settings.h/.cpp`: the settings file `dlidar.toml` in the project root (`--config PATH`), a strict TOML subset: which sensor, its endpoints and mount, the button, LED and pairing button pins (all different), the Mid-360S IMU setting, the IMU time mode. Unknown keys and bad values stop the start with the line number. The keys are listed in `docs/manual.html` ("Settings").
- `device_config.h`: the settings' built-in defaults (the sensors' endpoints, work mode and mount, the button, LED and pairing button pins, the Mid-360S IMU setting, the phone link's `kBleEnabled` and `kBleName`).
- `sensor_factory.h/.cpp`: the only code that knows every sensor adapter; opens the sensor the settings choose.
- `signals.h`: SIGINT/SIGTERM/SIGHUP stop (a second signal kills); SIGUSR1 asks for the Bluetooth pairing window (`tools/pair.sh`), SIGUSR2 to forget every phone first (`tools/pair.sh --forget`); the loop in `main.cpp` passes both to `PhoneLink`; fatal signals truncate the CSV.

## Data flow of a recording

```
input channel ──StartRecording──> ScannerController ──(worker thread)──> RecordSession
                                        │ states/warnings                │
                                        v                                │ Poll() per packet
                                    EventHub ──> console (sync)          v
                                             └─> LEDs, phone (async)  LidarSensor (UnitreeL2Sensor)
                                                                         │ ImuReading / points (sensor frame, SI)
                                                                         v
                                        CaptureTiming: zero, mount, g, filter, ImuTimeline
                                                                         v
                                        ChunkAssembler ──SensorChunk──> queue ──writer thread──> RecordingStorage
```

The phone output channel, on its own thread:

```
EventHub ──(async)──> PhoneEvents ──> Mailbox ──> PhoneLoop thread (GLib) ──> PhoneServer ─┬─> SocketTransport ──> local clients
                                                                                           └─> BleLink ──> BluezPeripheral ──> phone (BLE)
```

States: Idle → Preparing (sensor start-up) → Recording → Finalizing (sensor stopped, files finished) → Idle. The state is Error if the recording failed, with a warning first:
- the sensor did not start (the L2 gives up after 10 s without a packet, or 60 s without its status; the Mid-360S if it is not found within 10 s, or sends no packet for 10 s or no points within 60 s); no files are written
- the sensor sent no points for 10 s during the recording (`SessionConfig::sensor_silence_timeout`, host time; IMU packets do not count, so an L2 that restarted is caught too); the capture stops, what was captured is written and the files closed
- the storage could not open or failed while writing; the capture then stops
- an exception (sensor, storage, out of memory); the chunks already captured are written and the files closed

A new start is accepted from Idle or Error. A start command in any other state is rejected, and the current state is returned. The state that `Status()` and `StartRecording` report is the one that decides: an accepted start reads Preparing at once, and the final Idle/Error is stored before the channels hear of it, so a channel may start the next recording from its `OnState(Idle)`.

The storage opens only once the sensor has started, so a recording that never started (sensor failure, stop during Preparing: Preparing → Idle) leaves no files for dMapper to read.

The phone link is an output and an input channel: `PhoneEvents` runs on the hub's thread for it and hands events to its own loop thread through the mailbox; requests are answered on the loop from `DeviceApi` (a mutex copy of the status, a statvfs; `recording.start` / `recording.stop` call `ScannerController`, which never blocks on a recording, so the loop never waits). A stop from the phone saves the recording like the button. A client that does not read gets no events until it does (its latest state and progress are kept, the rest counted in `dropped`) and is closed past 64 KiB of unsent bytes, so no client can delay a recording. `wifi.connect` hands the join to `WifiSetup` and answers at once. `NmcliWifi` runs it on its own thread and never touches the sensor's Ethernet, so a join during a recording leaves the recording alone. The outcome comes back to the loop through `PhoneLoop::Invoke`, and `wifi.status` reads a cached link, so the loop never waits for NetworkManager either.

Threads: the controller's worker runs the recordings (capture loop, `CaptureTiming`, the events of the recording) and each recording has a writer thread (storage, storage warnings). The sensor is only used by the worker and the storage only by the writer. The chunk queue between them is not bounded, since dropping chunks would break the recording; past 50 waiting chunks a warning is sent.

## Recordings are byte-identical

The restructure changed no output. `tools/golden_replay.sh` replays a packet log through a base revision and the working tree and compares every file and the console log; the base was `36ecead`, the commit before the restructure. Since recording format 3 (`docs/recording-format.md`) the default base is `c6174f1`. Rules that keep this true:
- Every target builds with `-ffp-contract=off`. Fused multiply-adds would make results depend on the optimization level.
- `AxisMap` never multiplies. A multiply turns −0.0 into +0.0 and changes the IMU CSV text.
- The L2 conversion keeps its sequential angle accumulation and float `std::sin`/`std::cos`.
- The time zero rule: the stamp of the first point packet that has points, set before that packet is converted, even if all its points are filtered out.
- The order of work within a packet is fixed: log record, state row, points, IMU released by the point packet. `ChunkReady` is checked after every packet.

## SOLID

New code and refactors follow the SOLID principles on top of the dependency rule:
- **Single responsibility:** a class has one reason to change. `CaptureTiming` owns the timing, `ChunkAssembler` the chunks, `laz_encoder.cpp` is the only LASzip user. When a class gains a second job, split it instead of growing it.
- **Open/closed:** a new sensor, input or output channel is a new adapter wired in `main.cpp`, not an edit of the core (see "How to add …"). If a feature needs a core change, extend a port or add one rather than branching on the concrete adapter.
- **Liskov substitution:** every implementation of a port keeps its contract, so `FakeSensor`, `SyntheticL2Source`/`LiveL2Source` and `MemoryStorage`/`FileRecordingStorage` are interchangeable. What the port documents (units, frame, clock, return value on failure, threads) holds for each of them; the core tests with fakes only prove something because of this.
- **Interface segregation:** ports stay small and shaped by the use case that calls them. Optional capabilities get a default (`LensDirtiness()`, `Diagnostics()`) so a sensor without them implements nothing extra; a method only one adapter needs does not go on a shared port.
- **Dependency inversion:** the core depends on ports, never on adapters, and receives them by reference or injection. Adapters are constructed only in `src/app/`.

Byte-identical recordings come first: a SOLID cleanup that would change output goes to "Deferred" instead (like the timing report JSON).

## How to add …

**A sensor.**
1. Write an adapter in `src/adapters/<sensor>/` implementing `LidarSensor` (`application/ports/lidar_sensor.h`). It must deliver:
   - points in its own frame: metres, intensity, measured range, time in seconds relative to the zero the core passes to `ConvertPoints`
   - IMU samples in the same frame, in m/s² and rad/s, stamped on the same clock as the points
   - one record per packet with the host arrival time
2. Fields of `PointRecord` the sensor does not have are 0.
3. Provide its default `PointFilter`, and optionally `LensDirtiness()` and `Diagnostics()` (its own `packetstate` columns).
4. Add a CMake library for it, and a case for it in `src/app/sensor_factory.cpp`. Add its section to the settings (`src/app/settings.cpp`, with its defaults in `device_config.h`, and the table in `docs/manual.html`).

No core change is needed; `tests/application/fake_sensor.h` is a minimal example.

**An input channel** (button, phone command). Depend on `DeviceControl` (`application/ports/device_control.h`) or, for a remote client, `DeviceApi`, and call `ScannerController::StartRecording` / `StopRecording` / `Status` from the adapter's own thread. They are thread-safe and never block on a recording, and `StartRecording` returns false with the current state when busy. `WaitIdle` blocks until every accepted recording has ended; called from an event callback it returns at once.

**An output channel** (LEDs, phone status).
1. Implement `DeviceEvents` (`OnState`, `OnWarning`, `OnProgress`, `OnSummary`).
2. Register it in `main.cpp` with `hub.AddAsync(std::make_shared<MyChannel>(...))`. It gets its own thread and a bounded queue that drops the oldest event when full, so a slow or failing channel never affects a recording. At shutdown the async channels get 1 s together to deliver their queue; a channel still stuck is left on its thread, which keeps it alive.
3. `AddSync` is only for the console. The hub hands sync channels one event at a time (a recording publishes from two threads), on the publishing thread.

## Deferred

- The timing report JSON is built in the domain (`ImuTimeline::Report`, `AddRecordingInfo`). It belongs in a storage serializer, but moving it risks byte changes.
- `ContinuousDataWriter` still emits the timestamp-order warnings itself.
- `PointRecord` and the LAZ extra bytes keep the L2's layout (such as `raw_range` as uint16 mm); since format 3 each sensor defines what they mean (`RecordingProfile`, the Mid-360S puts distance and azimuth/elevation there). A sensor with other per-point data needs a new format.
- The Mid-360S's lens/window status (HMS code 0x0104) is not read: the port's `LensDirtiness()` is a percentage, the Mid-360S gives a warning level.
- `LidarSensor` has no IMU-clock capability yet. A sensor whose IMU is on another clock relies on the auto mode's fallback.
- The phone link's logs (phase 5), and removing or listing saved Wi-Fi networks (`wifi.forget`). Today it serves protocol v1 (status, info, start, stop, Wi-Fi setup) over the local socket and Bluetooth LE; any bonded phone may command (no per-phone allow-list).
- The signal stop flag stays set once raised, so it ends every later recording too. Right for both modes: a signal ends the service.
