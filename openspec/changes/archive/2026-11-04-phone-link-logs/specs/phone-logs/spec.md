# Spec Delta

## Purpose

Lets a connected client (a bonded phone over BLE, or a local socket client) read the scanner's log: the `log.get` command, its paging, which lines it returns, and the bounds that keep it from disturbing a recording.

## ADDED Requirements

### Requirement: log.get command
`log.get` SHALL answer at once with `{"entries":[…],"more":C}`: the newest log lines of the scanner's service, newest first. Each entry SHALL have:
- `t`: the time in Unix milliseconds;
- `boot`: a short identifier of the boot it was written in, equal for every line of the same boot;
- `level`: `info`, `warning` or `error`;
- `source`: `dlidar` or `system`;
- `text`: the line.

`more` SHALL be a cursor for the next older page, or `null` when no older line exists.

#### Scenario: Newest lines
- **WHEN** a client sends `{"v":1,"id":30,"cmd":"log.get"}`
- **THEN** it receives `{"id":30,"ok":true,"result":{"entries":[…],"more":…}}` with at most 50 entries, the first being the newest line

#### Scenario: Across a restart
- **WHEN** the service ran in the previous boot and runs again now
- **THEN** paging back returns this boot's lines, then the previous boot's lines, each with its own `boot` value

### Requirement: Paging
`log.get` with `before` set to a `more` cursor SHALL return the lines older than the last line of the page that gave that cursor, newest first. Paging until `more` is `null` SHALL return each line exactly once. Lines written after the first page SHALL NOT shift or repeat the older pages. When the line a cursor points to was removed from the journal in the meantime (rotation), paging SHALL continue from the newest remaining line older than it.

#### Scenario: Second page
- **WHEN** a client sends `log.get` with `"before":C`, C being the `more` of the first page
- **THEN** the first entry is the line just older than the first page's last entry

#### Scenario: New lines while paging
- **WHEN** dLidar writes new lines between two `log.get` requests of the same paging
- **THEN** the second page still continues just after the first page's last entry

### Requirement: log.get arguments
`log.get` SHALL take these optional arguments:
- `before`: a cursor;
- `limit`: an integer 1–200, default 50, the most entries a page holds;
- `level`: `info` (default), `warning` or `error`, the lowest level returned.

It SHALL answer `bad_request` naming the argument when `before` is not a string or not in the form of a cursor dLidar gives, `limit` is out of range or not an integer, or `level` is none of those values.

#### Scenario: Only warnings and errors
- **WHEN** a client sends `log.get` with `"level":"warning"`
- **THEN** every returned entry has `level` `warning` or `error`

#### Scenario: Bad limit
- **WHEN** a client sends `log.get` with `"limit":500`
- **THEN** it receives `bad_request` naming `limit`

### Requirement: Which lines are returned
`log.get` SHALL return only:
- the lines the scanner's service (`dlidar.service`) wrote, with `source` `dlidar`;
- systemd's own lines about that service (started, stopped, exited, crashed, restarted), with `source` `system`, when the system lets the service read them.

It SHALL NOT return lines of any other service, the kernel or other users. Lines of dLidar started by hand, outside the service, are not included.

#### Scenario: A crash and restart
- **WHEN** the service crashed and systemd restarted it, and the service may read systemd's lines
- **THEN** paging back shows systemd's lines about the crash and the restart with `source` `system`, between the lines of the two runs

#### Scenario: Nothing else
- **WHEN** a client pages through the whole log
- **THEN** no entry comes from another unit, the kernel, or another user's processes

### Requirement: Levels
A line of dLidar SHALL have `level` `warning` when it starts with `WARNING:`, `error` when it starts with `ERROR:`, and `info` otherwise. A line of systemd SHALL have `error` for its error priorities and worse, `warning` for its warning priority, and `info` otherwise.

#### Scenario: A dLidar warning
- **WHEN** dLidar printed `WARNING: Bluetooth (bluetoothd) is gone; the phone link waits for it`
- **THEN** its entry has `level` `warning`, `source` `dlidar`, and that text

### Requirement: Size and load bounds
Each `log.get` answer SHALL fit the protocol's 16384-byte message limit. A `text` longer than 1024 bytes SHALL be cut to at most 1024 bytes on a UTF-8 character boundary. One request SHALL read a bounded number of journal entries; when it stops at that bound or at the size limit, it SHALL return the entries found with `more` not `null`, so a page may hold fewer than `limit` entries, even none, while older lines exist.

#### Scenario: A large page
- **WHEN** a client sends `log.get` with `"limit":200` and the lines are long
- **THEN** the answer is at most 16384 bytes, and `more` continues where it stopped

#### Scenario: A rare level
- **WHEN** a client asks for `"level":"error"` and no error appears in a long stretch of the log
- **THEN** the answer comes after a bounded read, possibly with no entries and `more` not `null`

### Requirement: Log unavailable
When the journal cannot be read, `log.get` SHALL answer the error `unavailable` with a readable message, and the rest of the protocol SHALL keep working.

#### Scenario: No journal
- **WHEN** the journal cannot be opened
- **THEN** `log.get` answers `ok:false` with code `unavailable`, and `status.get` still answers

### Requirement: No effect on recordings
Reading the log SHALL NOT write any file, and SHALL NOT delay or change a recording.

#### Scenario: Paging during a recording
- **WHEN** a client pages through the whole log while a recording runs
- **THEN** the recording continues and its files are unaffected

### Requirement: log.get is listed
`hello` SHALL list `log.get` among its commands, and the protocol SHALL stay version 1.

#### Scenario: hello
- **WHEN** a client sends `hello`
- **THEN** `commands` contains `log.get` besides the existing commands
