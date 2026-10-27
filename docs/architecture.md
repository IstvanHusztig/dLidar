# dLidar architecture

dLidar is the application of a handheld scanning device. It records a LiDAR with an IMU (today a Unitree L2) into files that dMapper, our SLAM solution, processes. Over time the device gains buttons, status LEDs, a Bluetooth link to a phone, and possibly a different LiDAR. The code follows Clean Architecture (Robert C. Martin) so that those can be added without touching the recording logic.

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
| `adapters/<x>/` | the core + its own directory (incl. its `vendor/`) | `app/`, another adapter |
| `app/` | anything | |

Enforcement:
- **Build:** the CMake targets give each layer only what it needs.
  - `dlidar_core` (domain + application) has only `src/` on its include path and links nothing.
  - `dlidar_l2` adds the vendor SDK, `dlidar_storage` adds LASzip and `3rd/`, `dlidar_console` adds nothing.
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
  - `RecordingStorage`.
  - `DeviceEvents`: an output channel.
  - `StopToken`.
- `capture/`:
  - `CaptureTiming`: owns the time zero; applies the mount, m/s² → g and the filter; feeds `ImuTimeline`.
  - `ChunkAssembler`.
  - `RecordSession`: one recording, with a writer thread so the SD card never stalls packet reading.
  - `LensMonitor`.
- `device/`:
  - `ScannerController`: the state machine and the command interface.
  - `EventHub`: fans events out to the output channels.
- `analysis/`: `ReplayPackets` (`bin/replay_packets`).

**Adapters (`src/adapters/`)**
- `unitree_l2/`: `UnitreeL2Sensor` over an `L2PacketSource`.
  - `LiveL2Source` is the SDK reader, with the start-up sequence and the 2 s backlog drain.
  - `SyntheticL2Source` replays a packet log.
  - `l2_conversion` and `l2_packet_state` handle the L2's packets.
  - `vendor/` holds Unitree's unmodified headers.
- `storage/`: `FileRecordingStorage`, `ContinuousDataWriter`, the LAZ encoder (the only LASzip user), recovery, packet log reading, and the crash guard.
- `console/`: `ConsoleEvents`, the texts `bin/dLidar` prints.

**App (`src/app/`)**
- `main.cpp`: CLI flags, the settings, recovery of leftovers, building the adapters, one recording.
- `settings.h/.cpp`: the settings file `~/.config/dlidar/dlidar.toml` (`--config PATH`), a strict TOML subset: which sensor, its endpoints and mount, the chunk limit, the IMU time mode. Unknown keys and bad values stop the start with the line number. The keys are listed in `docs/manual.html` ("Settings").
- `device_config.h`: the settings' built-in defaults (the L2's endpoints, work mode and mount, the chunk limit).
- `sensor_factory.h/.cpp`: the only code that knows every sensor adapter; opens the sensor the settings choose.
- `signals.h`: SIGINT/SIGTERM/SIGHUP stop (a second signal kills); fatal signals truncate the CSV.

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

States: Idle → Preparing (sensor start-up) → Recording → Finalizing (sensor stopped, files finished) → Idle. The state is Error if the recording failed, with a warning first:
- the sensor did not start (the L2 gives up after 10 s without a packet, or 60 s without its status); no files are written
- the sensor sent no points for 10 s during the recording (`SessionConfig::sensor_silence_timeout`, host time; IMU packets do not count, so an L2 that restarted is caught too); the capture stops, what was captured is written and the files closed
- the storage could not open or failed while writing; the capture then stops
- an exception (sensor, storage, out of memory); the chunks already captured are written and the files closed

A new start is accepted from Idle or Error. A start command in any other state is rejected, and the current state is returned. The state that `Status()` and `StartRecording` report is the one that decides: an accepted start reads Preparing at once, and the final Idle/Error is stored before the channels hear of it, so a channel may start the next recording from its `OnState(Idle)`.

The storage opens only once the sensor has started, so a recording that never started (sensor failure, stop during Preparing: Preparing → Idle) leaves no files for dMapper to read.

Threads: the controller's worker runs the recordings (capture loop, `CaptureTiming`, the events of the recording) and each recording has a writer thread (storage, storage warnings). The sensor is only used by the worker and the storage only by the writer. The chunk queue between them is not bounded, since dropping chunks would break the recording; past 50 waiting chunks a warning is sent.

## Recordings are byte-identical

The restructure changed no output. `tools/golden_replay.sh` replays a packet log through a base revision and the working tree and compares every file and the console log; the base is `36ecead`, the commit before the restructure. Rules that keep this true:
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

**An input channel** (button, phone command). Call `ScannerController::StartRecording` / `StopRecording` / `Status` from the adapter's own thread. They are thread-safe and never block on a recording, and `StartRecording` returns false with the current state when busy. `WaitIdle` blocks until every accepted recording has ended; called from an event callback it returns at once.

**An output channel** (LEDs, phone status).
1. Implement `DeviceEvents` (`OnState`, `OnWarning`, `OnProgress`, `OnSummary`).
2. Register it in `main.cpp` with `hub.AddAsync(std::make_shared<MyChannel>(...))`. It gets its own thread and a bounded queue that drops the oldest event when full, so a slow or failing channel never affects a recording. At shutdown the async channels get 1 s together to deliver their queue; a channel still stuck is left on its thread, which keeps it alive.
3. `AddSync` is only for the console. The hub hands sync channels one event at a time (a recording publishes from two threads), on the publishing thread.

## Deferred

- The timing report JSON is built in the domain (`ImuTimeline::Report`, `AddRecordingInfo`). It belongs in a storage serializer, but moving it risks byte changes.
- `ContinuousDataWriter` still emits the timestamp-order warnings itself.
- `PointRecord` and the LAZ extra bytes follow L2 semantics, such as `raw_range` as uint16 mm. A second sensor may need recording format 3 with a sensor-defined extra-bytes schema.
- `LidarSensor` has no IMU-clock capability yet. A sensor whose IMU is on another clock relies on the auto mode's fallback.
- A daemon mode (wait idle for buttons/phone), GPIO and Bluetooth adapters.
- The signal stop flag stays set once raised, so it ends every later recording too. Right for the CLI; a daemon mode needs its own exit command.
