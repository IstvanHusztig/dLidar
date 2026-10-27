# Proposal

## Why

dLidar is growing from a one-shot capture program into the application that runs the whole handheld scanning device. Buttons, status LEDs and a Bluetooth link to a phone are coming, and the LiDAR model may change. The current code cannot absorb any of that without rewriting its most valuable parts:

- **Vendor types everywhere.** Unitree SDK types and two process-wide globals in the modified vendor header (`GetGlobalTimeOffsetNs`, `GetSensorOrientation`) reach the capture loop, the time alignment front end, the writer, every test except the IMU timestamper's, and the replay tool.
- **No seams for device I/O.** There is no place to plug in a new input (buttons, phone commands) or output (LEDs, phone status). `ProcessSensorData` is a single blocking function, `main` runs it once, and status goes only to `std::cout`.
- **No enforced boundaries.** The code is header-only and nothing stops new dependencies from leaking inward.

The timing work is finished and well tested, and a deterministic replay exists, so now is a good time to restructure while proving that the recording output does not change.

## What Changes

- **Layers enforced by the build** (Clean Architecture): domain and application (use cases) in a core target that can see neither vendor SDKs, LASzip nor OS device APIs. Adapters (Unitree L2, storage) and the app composition root sit outside it.
- **LiDAR sensor port**: a vendor-neutral interface for points, IMU samples, packet timing, host arrival times, identity and optional health/diagnostics. The Unitree L2 becomes one adapter behind it: SDK setup, start-up sequence, conversion, packet state rows.
- **Device geometry and point policy move to the core**: the mounting rotation (applied identically to points and IMU), the m/s² → g conversion, the shared time zero and the range/intensity filter. The globals are removed.
- **Device control seam**: a recording state machine (Idle → Preparing → Recording → Finalizing → Idle/Error) behind one thread-safe command interface (start, stop, status), and a status/event output port that can have several subscribers.
  - Implemented now: the CLI/signal input and a console output that keeps today's console text.
  - Later changes: buttons, LEDs and Bluetooth plug in as adapters without core changes.
- **Replay through the real use case**: `replay_capture` drives the actual recording use case through the L2 adapter fed with synthetic packets, so the golden replay covers the production path.
- **Deterministic floating point**: `-ffp-contract=off` on every target. Today `-O2` tools fuse multiply-adds that the `-O0` binary does not, so a replay does not reproduce production arithmetic.
- **No change to recordings**: every output file stays byte-identical for the same input (`imuNNNN.csv`, `lidarNNNN.laz`, `timingNNNN.json`, `packetsNNNN.bin`, `packetstateNNNN.tsv`). File names, format version, units, axes, timestamps, CLI flags, the stop/recovery behaviour and the console messages are also unchanged, so dMapper needs no change.

## Capabilities

### New Capabilities
- `sensor-integration`: the contract between the core and a LiDAR/IMU sensor. It covers what a sensor delivers, which policies the core owns (mount, units, time zero, filter), that the core does not depend on a vendor SDK, and that L2 recordings stay unchanged.
- `device-control`: how the device is driven and observed. One recording at a time; commands from any input channel through one interface; status events delivered to every output channel without affecting the recording; today's CLI one-shot behaviour kept.

### Modified Capabilities
- None. There are no existing specs.

## Impact

- **Code**:
  - `include/*` moves into `src/domain`, `src/application`, `src/adapters/{unitree_l2,storage}` and `src/app`.
  - The vendor headers become private to the L2 adapter, and the vendor `unitree_lidar_utilities.h` is reduced to the vendor's own parts.
  - `.h`/`.cpp` splits happen only where a boundary hides a dependency (L2 adapter, storage).
  - Tests are regrouped by layer, with fake sensor/storage/event sinks for the core.
- **Build**: targets `dlidar_core`, `dlidar_l2`, `dlidar_storage`, plus the executables. `-ffp-contract=off` is set globally. `cmake --preset ninja`, `cmake --build build` and `ctest` stay the same, and the binary is still `bin/dLidar`.
- **Tooling**:
  - New `tools/golden_replay.sh`: builds the base revision in a git worktree, replays, and `cmp`s every file.
  - New `tools/check_architecture.py`: include rules per layer, run by ctest.
  - `compare_recording_formats.sh` is updated for the new layout.
- **Runtime (Pi Zero 2 W)**:
  - Overhead: one virtual call per packet, one extra pass per packet for mount and filter, status events per chunk.
  - Threads and queue as today.
  - Capture moves to a worker thread owned by the controller, so the main thread is free for future input channels.
- **Docs**: `CLAUDE.md` (architecture, file paths), the `docs/*.md` file references, and a new `docs/architecture.md` (layers, ports, how to add a sensor/input/output).
- **Out of scope / follow-ups**:
  - GPIO buttons/LEDs, the Bluetooth adapter and a daemon mode.
  - A second sensor and a sensor-defined LAZ extra-bytes schema (format 3).
  - A config file.
  - The existing unbounded chunk queue when the writer fails to open (separate bug fix).
