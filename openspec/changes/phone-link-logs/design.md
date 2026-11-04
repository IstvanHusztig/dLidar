# Design

## Context

- In service mode dLidar's stdout and stderr go to the journal as `dlidar.service` (`_TRANSPORT=stdout`, every line `PRIORITY=6`). Warnings are marked only by their text: 44 `"WARNING:` and 7 `"ERROR:` prefixes in `src/`.
- The dev board's journal is persistent (`/var/log/journal`, 110 MB, 3 boots listed). Without any extra group, the service user already reads `dlidar.service`'s lines of every boot, because journald gives each user read access to the entries of their own processes (checked on the dev board 2026-11-04, user `vs2022` in groups `sudo`, `users`, `gpio`). systemd's own lines about the unit (`_PID=1`, `UNIT=dlidar.service`: started, main process exited, scheduled restart) belong to root, and only groups `systemd-journal` and `adm` may read them.
- The phone protocol handles requests on the phone loop's thread (`PhoneServer`), one at a time, and a message from dLidar is at most 16384 bytes. BLE carries about 24 KiB/s (`BleLink`'s token bucket), so a full page takes under a second to reach the phone.
- Adapters never include each other (`tools/check_architecture.py`). `ProcessRunner` lives in the NetworkManager adapter, so another adapter cannot reuse it.
- systemd 252 on the dev board. `libsystemd0` is installed; `libsystemd-dev` (headers, pkg-config) is not.

## Goals / Non-Goals

**Goals:**
- One request reads a bounded part of the journal and answers in a few tens of milliseconds, so the phone loop and a running recording are not disturbed.
- Paging is stable while new lines arrive, and survives journal rotation.
- Nothing outside `dlidar.service` leaves the scanner.

**Non-Goals:**
- Live streaming of new lines (a `log` event). `warning` events and `status.get` already show the current problems; a client polls `log.get` for more.
- Lines of dLidar run by hand: they go to the terminal.
- Searching the text or filtering by time. The app can filter a page it holds.
- Kernel or other units' logs, core dumps (`systemd-coredump` is not installed).

## Decisions

### D1. Read the journal in process with libsystemd's `sd-journal`
`sd_journal_open(SD_JOURNAL_LOCAL_ONLY)`, matches, `sd_journal_seek_cursor`/`seek_tail`, `sd_journal_previous`, `sd_journal_get_data`/`get_realtime_usec`/`get_cursor`. libsystemd is linked privately into the new adapter only, as GIO is into the phone adapter.

Alternatives:
- **`journalctl -o json` as a subprocess:** a process per page (tens of ms on the Pi, more under load), JSON parsing, and the runner sits in another adapter (it would have to be copied or moved to a shared layer that does not exist).
- **An in-memory ring of dLidar's own lines:** no previous boot, no crash, no restart. Those are the main reasons to read a log from the phone.

### D2. A `LogReader` port, the protocol rules in the phone adapter
`src/application/ports/log_reader.h`:
- `LogLevel {Info, Warning, Error}`, `LogSource {Dlidar, System}`;
- `LogEntry {time_ms, boot, level, source, text, cursor}`;
- `LogQuery {optional<string> before, int limit, LogLevel min_level}`;
- `LogPage {vector<LogEntry> entries, optional<string> more}`;
- `class LogReader { virtual LogPage Read(const LogQuery &) = 0; }` with two exceptions `LogCursorError` (a malformed `before`) and `LogUnavailable` (with the reason).

No use case: there is no device rule here, only reading. The argument rules (`limit` 1–200, `level` names) and the 16 KiB fit are protocol rules, so they are in the codec and `PhoneServer`. The reader gives each entry its own cursor, so the codec can end a page early and set `more` to the cursor of the last entry it kept.

### D3. Which entries, and their source and level
Two match groups, combined with a disjunction:
- `_SYSTEMD_UNIT=dlidar.service` → `source` `dlidar`;
- `_PID=1` and `UNIT=dlidar.service` → `source` `system`.

The unit name is an adapter option, default `dlidar.service`. Level: for `dlidar`, the `WARNING:` / `ERROR:` prefix as in the spec; for `system`, `PRIORITY` 0–3 `error`, 4 `warning`, else `info`. `boot` is the first 8 hex digits of `_BOOT_ID`. `t` is `get_realtime_usec / 1000`.

### D4. Paging by journal cursors
- No `before`: `seek_tail`, then `previous` for each entry.
- With `before`: `seek_cursor(before)` (`-EINVAL` → `LogCursorError` → `bad_request` naming `before`), then `previous`. A backward step after a cursor seek lands on the cursor's own entry, or, if that entry was vacuumed, on the nearest older one. If `sd_journal_test_cursor` says the reader stands on the cursor's own entry, it is skipped, so it is not repeated. No time comparison: the Pi's clock may jump backwards between boots, so a newer `t` does not mean a newer line.
- `more` is the cursor of the last entry kept, or `null` when `previous` reached the head.

Cursors are opaque strings for the client (about 130 bytes, well within a 4096-byte request). Newer lines do not move them, so pages stay stable while dLidar writes.

### D5. Bounds
- `limit` at most 200.
- At most 5000 entries are read per request, the level filter included. At that bound the page ends with `more` set to the cursor of the last entry read, even if it was filtered out (the cursor tells where to continue, not which entry was returned).
- `text` is cut at 1024 bytes, on a UTF-8 boundary; invalid UTF-8 bytes become U+FFFD before encoding.
- The codec adds entries while the encoded answer stays within 16384 bytes. With 1024-byte texts that is at least 13 entries.

### D6. On the phone loop, the journal opened per request
`Read` runs synchronously on the phone loop thread, which owns the `sd_journal` handle from open to close. The journal is opened and closed for every request. This sees new journal files after a rotation without `sd_journal_process` bookkeeping, and holds no file descriptors or mappings between requests.

The board check (task 5.1) measures the open time and a full 200-entry page. If a page takes over 100 ms, the handle is kept open between requests, with `sd_journal_get_fd` and `sd_journal_process` at the start of each request. That changes only the adapter.

### D7. The `systemd-journal` group on the unit
Chosen by the user (2026-11-04), so the phone shows systemd's crash and restart lines.
`SupplementaryGroups=gpio bluetooth systemd-journal`. The group lets the service read the whole system journal; D3 makes sure only its own unit's lines are returned. Without the group (an old unit file), `sd-journal` simply skips the root-owned files, and `log.get` returns dLidar's own lines only, without an error. The manual says to reinstall the unit.

### D8. `unavailable`
`sd_journal_open` failing, or every read failing with an error other than the end of the journal, raises `LogUnavailable` and `log.get` answers `unavailable` with the reason (e.g. `"cannot open the journal: Permission denied"`). An empty journal is not an error: `{"entries":[],"more":null}`.

### D9. Wiring
- `PhoneServer` takes a `LogReader *` (nullptr: no `log.get`, as for Wi-Fi), and `PhoneLink::Start` passes it.
- `main.cpp` builds `dlidar::journal::JournalReader` with the default unit name.
- CMake: `pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)`; the target `dlidar_journal` (PUBLIC `dlidar_core`, PRIVATE `PkgConfig::SYSTEMD`) is linked into `dLidar`.
- `check_architecture.py` needs no new rule: `systemd/sd-journal.h` is included only under `src/adapters/journal/`, as an external header.

### D10. Tests
- Codec: the argument rules, the entry encoding, the 16 KiB fit with 200 long entries (`more` set at the cut), the doc examples.
- `PhoneServer` with a fake `LogReader`: `bad_request` for `before` from `LogCursorError`, `unavailable`, `hello` with and without the reader.
- Journal adapter, pure helpers: the level of a dLidar line and of a systemd priority, the UTF-8 cut, the boot id.
- Journal adapter against the real journal, read-only, skipped when it has no `dlidar.service` lines: every entry belongs to the unit; paging to the head returns each cursor once and in descending time; a malformed cursor throws `LogCursorError`.

## Risks / Trade-offs

- **[The Pi's clock is wrong offline]** The Pi 5 keeps time only with a battery on its RTC connector; in a cave without network `t` may start at the last saved time. → The entries stay in journal order and `boot` groups them; the app shows `t` as written.
- **[A request blocks the phone loop]** → D5's 5000-entry bound and D6's measurement; BLE events are delayed at most for the length of one read.
- **[The group widens what the service may read]** The service could read every system log line. → Only D3's matches are ever returned; the scanner has one user (user, 2026-11-03).
- **[Rotation between pages]** → D4 continues from the nearest older entry.
- **[A bonded phone reads network names and paths in the log]** → Bonded phones are trusted with commands already; passwords are never logged (spec `phone-wifi`).

## Migration Plan

1. `sudo apt install libsystemd-dev` on each build machine (the manual's build line gets it next to `libglib2.0-dev`).
2. Build, then reinstall `systemd/dlidar.service` (`daemon-reload`, restart) for the group.
3. Clients see `log.get` in `hello`. Older builds answer `unknown_command`; the app already handles that.

Rollback: the previous build; the extra group does no harm to it.
