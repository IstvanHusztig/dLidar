# Proposal

## Why

When a recording fails or the scanner behaves oddly in the field, the reason is in dLidar's console output, and in service mode that output is in the systemd journal. Reading it today needs a shell on the Pi (`journalctl -u dlidar`). The phone already shows the current warnings (`status.get`, `warning` events), but not what happened before: the previous recording, the previous boot, a crash and the restart after it.

This is the last phase of the phone link roadmap (`phone-link-core`, `-ble`, `-commands`, `-wifi`, then logs). The roadmap fixed its shape: a journald-based `log.get` with paging.

## What Changes

- **New command `log.get`.** It returns a page of dLidar's journal lines, newest first. Arguments, all optional:
  - `before`: a page cursor from an earlier answer. Without it the page starts at the newest line.
  - `limit`: at most this many lines, 1–200, default 50.
  - `level`: the lowest level to return, `info` (default), `warning` or `error`.

  The answer is `{"entries":[…],"more":C}`. Each entry has:
  - `t`: the time in Unix milliseconds, from the Pi's clock;
  - `boot`: a short id of the boot it came from, so the app can mark restarts;
  - `level`: `info`, `warning` or `error`;
  - `source`: `dlidar` (dLidar's own lines) or `system` (systemd's lines about the service: start, stop, exit, crash, restart);
  - `text`: the line, cut at 1024 bytes.

  `more` is the cursor for the next, older page, or `null` when there is nothing older. A page always fits the 16 KiB message limit, so it may hold fewer than `limit` lines while `more` is not `null`.
- **Errors:**
  - `bad_request`, naming the argument, for a bad `before`, `limit` or `level`;
  - a new error code `unavailable` when the journal cannot be read. It is additive: clients already treat unknown codes as generic.
- **Only dLidar's own service is read.** The command returns the lines of `dlidar.service` and systemd's lines about it, never another unit's or the kernel's. Passwords never reach the journal (spec `phone-wifi`), so none can leave through `log.get`.
- **systemd's lines about the service need a group.** The service gets `SupplementaryGroups=… systemd-journal`. Without it, `log.get` still returns dLidar's own lines, but not systemd's lines (start, stop, crash). The scanner has one user (user, 2026-11-03), so reading the system journal is acceptable. The installation reinstalls the unit.
- **dLidar run by hand** (`bin/dLidar`, `--once`) writes to the terminal, not the journal. Its lines are therefore not in `log.get`, which still shows the service's earlier lines.
- **`hello`** lists `log.get`. The protocol stays version 1: these are additions only.
- **Docs:**
  - `docs/phone-protocol.md`: `log.get`, paging, the `unavailable` code;
  - the manual: the unit's group and reading logs from the app;
  - `docs/architecture.md`, `CLAUDE.md` (the new build dependency) and `README.md`.

  The app agent gets the draft contract before it is frozen.

## Capabilities

### New Capabilities
- `phone-logs`: reading the scanner's log from a connected client:
  - the `log.get` command, its arguments, paging and answer;
  - which lines are included and how their level is decided;
  - the size and load bounds;
  - the `unavailable` error.

### Modified Capabilities

None. The `phone-ble`, `phone-wifi` and `device-control` requirements are unchanged; `hello`'s list of commands is not a requirement of a main spec (it lives in `phone-link-core`'s `phone-api` delta, which is not archived).

## Impact

- **Code:**
  - `src/application/ports/log_reader.h`: the port (a query in, a page out).
  - A new adapter `src/adapters/journal/`, CMake target `dlidar_journal`, which reads the journal in process through libsystemd's `sd-journal` API.
  - `src/adapters/phone/`: the codec (arguments, the page within 16 KiB) and the `log.get` command in `PhoneServer`. `PhoneLink::Start` takes the log reader.
  - `src/app/main.cpp` wires it.
  - No change to `src/domain`, `RecordSession`, `ScannerController` or the storage adapter.
- **Build dependency:** `libsystemd-dev` (headers and pkg-config file), installed with `apt` next to `libglib2.0-dev`. The runtime library `libsystemd0` is part of every Debian system.
- **System:** `systemd/dlidar.service` adds the group `systemd-journal`; the unit is reinstalled.
- **Recordings and dMapper:** no effect. `log.get` reads the journal and writes nothing; a request during a recording reads at most a bounded number of entries. Golden replay stays identical.
- **The app:** one new command and one new error code in the existing message format; nothing existing changes.
