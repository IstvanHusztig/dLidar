# Tasks

## 1. Port and build dependency (D1, D2, D9)

- [x] 1.1 Ask the user to install `libsystemd-dev` on the dev board (`sudo apt install libsystemd-dev`). Then add `pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)` to `CMakeLists.txt`, with an empty `dlidar_journal` target. Verify that `cmake --preset ninja` and `cmake --build build` succeed.
- [x] 1.2 `src/application/ports/log_reader.h`: `LogLevel`, `LogSource`, `LogEntry`, `LogQuery`, `LogPage`, `LogReader`, `LogCursorError`, `LogUnavailable` (D2). Verify that `tools/check_architecture.py --strict` passes and the core builds without libsystemd.

## 2. Protocol and phone server (D2, D5, D8, D9)

- [x] 2.1 Codec:
  - decode the `log.get` arguments (`before` a string, `limit` 1–200, `level` `info`/`warning`/`error`), each `bad_request` naming the argument;
  - encode the page newest first within 16384 bytes, with `more` set to the cursor of the last entry kept when it stops early;
  - the `unavailable` error.

  Verify with ctest `protocol_codec`: every argument rule and boundary, 200 entries with 1024-byte texts (answer ≤ 16384 bytes, `more` at the cut), an empty page (`more` `null`).
- [x] 2.2 `PhoneServer` and `PhoneLink::Start`: the `log.get` command, offered only with a `LogReader`; `LogCursorError` → `bad_request` naming `before`; `LogUnavailable` → `unavailable` with its reason; `hello` lists `log.get`. Verify with ctest `phone_server` (fake `LogReader`: query passed through, both errors, `hello` with and without) and `phone_socket` (`log.get` over the socket).
- [x] 2.3 `docs/phone-protocol.md`:
  - a `log.get` section: arguments, entry fields, paging with `more`, the fewer-than-`limit` note, examples (first page, next page, `"level":"warning"`, `bad_request` for `limit`, `unavailable`);
  - the `unavailable` row in the error table;
  - `hello`'s examples list `log.get`.

  Verify that ctest `protocol_codec` accepts every new example line. Then send the draft section to the app agent and record its answer in `openspec/changes/phone-link-core/app-questions.md`.

## 3. Journal adapter (D1, D3–D6)

- [x] 3.1 Pure helpers in `src/adapters/journal/`: the level of a dLidar line (`WARNING:`, `ERROR:`) and of a systemd `PRIORITY`, the cut at 1024 bytes on a UTF-8 boundary with invalid bytes replaced, the short boot id. Verify with a new ctest `journal_reader` (`tests/adapters/journal/test_journal_reader.cpp`).
- [x] 3.2 `JournalReader` (`sd-journal`, opened per request; the two match groups, the unit name an option; seek to the tail or to `before`, step past the cursor's own entry (after a rotation the step lands on the nearest older one); at most 5000 entries read; `more`). Verify with ctest `journal_reader` against the dev board's journal, read-only, skipped without `dlidar.service` lines:
  - every entry belongs to the unit;
  - paging to the head returns each cursor once, in descending time;
  - a malformed cursor throws `LogCursorError`;
  - `"level":"error"` over a long stretch ends after at most 5000 entries read, with `more` set.

## 4. Wiring, unit and docs (D7, D9)

- [x] 4.1 `src/app/main.cpp` builds the `JournalReader` and passes it to `PhoneLink::Start`; `systemd/dlidar.service` adds `systemd-journal` to `SupplementaryGroups` (comment: why, and that only the unit's lines are returned). Verify that ctest passes and `tools/golden_replay.sh` is identical.
- [x] 4.2 Docs:
  - `docs/manual.html`: the build line with `libsystemd-dev`, the unit's group (reinstall), reading the log from the app or `tools/phone_client.py log.get`;
  - `docs/architecture.md`: the `LogReader` port and the journal adapter;
  - `CLAUDE.md`: the build dependency and an entry for `src/adapters/journal/`;
  - `README.md`;
  - `tools/phone_client.py`'s help text.

  Verify that every path and command named in them exists.

- [x] 4.3 Earlier reviewer notes, in a commit of their own: the Wi-Fi test fake's `Cancel()` follows the runner contract (the next Run is cancelled) with an `NmcliWifi` test where `Stop` lands between two nmcli steps; a comment on the cancel that arrives just as a child exits; `CLAUDE.md`'s link to the archived phone-link-ble design. Verify with ctest `nmcli_wifi` and that the link resolves.

## 5. Board checks

- [x] 5.1 Ask the user before restarting `dlidar.service` and reinstalling the unit with the group. Then, on the dev board:
  - `tools/phone_client.py log.get` and paging to the head;
  - systemd's start and stop lines with `source` `system`, after `sudo systemctl restart dlidar`;
  - the time of opening the journal and of a 200-entry page, logged once; if a page takes over 100 ms, apply D6's fallback;
  - `log.get` from the app over BLE (with the app agent).

  Record the results in design.md ("Board results").
- [x] 5.2 Without the group (the old unit), `log.get` returns dLidar's lines only, without an error. Verify on the board with a temporary unit drop-in, then remove it.

- [x] 5.3 BLE notifications at most 512 bytes (`BleLink::kMaxValue`; spec delta `phone-ble`): the app agent saw 2 bytes lost per 514-byte notification at MTU 517. Verify with ctest `ble_link` (MTU 517: 512 + 488; a 5000-byte message in pieces ≤ 512, joined unchanged) and `log.get` paging from the app over BLE.

## 6. Review

- [x] 6.1 Full ctest, `tools/golden_replay.sh` (identical), `tools/check_architecture.py --strict`, and the architecture-reviewer agent on the tip before the push.

## Workflow follow-up

- `/opsx:archive phone-link-logs` after 6.1; this ends the phone link roadmap. Then `Printable()`'s duplicate (two adapters; a shared helper needs a home in the core) and the parked hardware checks when the lidar arrives.
