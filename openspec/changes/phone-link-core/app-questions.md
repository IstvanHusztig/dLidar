# Protocol v1: answers to the app team's questions (2026-11-02)

Answers to `protocol-open-questions.md`. **Binding** means it is in the spec (`specs/phone-api/spec.md`) and goes into `docs/phone-protocol.md`. **Provisional** means it belongs to a later phase or waits for the BLE spike; the app may plan for it but must not depend on it yet.

## Framing and limits

1. **Yes.** 4096 includes the `\n`. The app's choice is right. *Binding.*
2. **Yes.** 16384 includes the `\n`. Accepting one more byte is harmless. *Binding.*
3. **Both are allowed.** dLidar joins everything written on one connection into one byte stream, so a write may hold several messages or part of one. One request per write is fine and is the safest. *Binding.*
4. **Both are handled the same way.** Use write-with-response: it gives back-pressure. dLidar answers each request at once, so it does not limit pending requests. But if the app writes faster than it takes in responses and dLidar's unsent output to it passes 64 KiB, the connection is closed. *Binding.*
5. **dLidar sets no MTU cap of its own.** BlueZ negotiates the MTU (up to 517), so 517 from Android 14 is fine. dLidar splits notifications by the MTU of this connection, as BlueZ reports it with the app's writes and the notification setup. The app must not assume 247. dLidar does not start the exchange itself. *Provisional until spike measurement 1.*
6. **No other response.** An `"id":null` error for `too_large`, or `bad_request` without a readable id, takes that line's place in the response order. So the app may complete its **oldest pending request** with that error instead of waiting 5 s. `too_many_clients` is the exception: it belongs to no request. *Binding.*

## Requests and responses

7. **Yes.** Ids are scoped to the connection, and dLidar never checks them for reuse. The app's scheme is fine. *Binding.*
8. **One at a time, in order.** No command waits for the device. `recording.start` answers at once: starting runs on another thread, and the state comes in events. A result that takes long uses `accepted` plus a `result` event (31). So responses never wait behind a slow command. *Binding.*
9. **Only if the link stalls.** dLidar answers within milliseconds. Dropping a late response is fine. After a timeout, call `status.get` rather than reconnect. *Binding.*
10. **Required on every request, `hello` included.** Without it: `bad_request`. *Binding.*
11. **`supported` lists every version dLidar speaks.** Compare: if the app's version is above the highest in `supported`, say "update the scanner"; if it is below the lowest, say "update the app". Today only `[1]` exists. *Binding.*
12. **Never.** `ok:true` always has a `result` object, possibly `{}`. *Binding.*

## Events

13. **`seq` never jumps.** It counts the events actually sent on this connection, from 1, with no gaps. Events left out get no `seq`, and `dropped:K` on the next event is the only loss signal. Treating a jump as a gap is still a good safety net. *Binding.*
14. **Nothing is sent on connect.** The first event is `seq:1`, sent at the next change. Call `status.get` after connecting, as the app already does. *Binding.*
15. **Over BLE the connection opens when the app turns on TX notifications.** Events before that are neither sent nor counted. *Binding.*
16. **Added:** `status.get` now has `warnings`: the warnings of the current or last recording (`kind`, `text`), at most the 16 newest, emptied when a recording starts. Warnings report events, not conditions, so there is no "cleared" event. *Binding.*
17. **Added:** `status.get` now has `summary`: the last recording's summary, or `null`. *Binding.*
18. **No `progress` after `finalizing`.** The last `progress` comes before `finalizing`. The recording's totals, its last partial chunk included, are in the final `idle` (or `error`) state. Every `state` event carries `chunks` and `points`. *Binding.*
19. **`data` is always an object.** Treating a missing one as empty is fine. *Binding.*

## Field meanings

20. **`location`:** the absolute path of the output directory, for example `/home/pi/PointCloudDump`. It is never empty. Show it as text. *Binding.*
21. **`last_ok`:** whether the last finished recording was saved completely. It is `true` before the first recording since dLidar started. *Binding.*
22. **`free_bytes: null`:** the free space could not be read (the directory is missing or unreadable). Show "unknown". *Binding.*
23. **`error` lasts until the next recording starts.** A start is accepted in `error` just as in `idle`. The reason is the warning that came with it, also in `status.get` `warnings`. *Binding.*
24. **`imu_mode`:** `sensor` (IMU times from the sensor's own clock, the normal case), `host_aligned` (rebuilt from arrival times, a fallback), or `auto` (a recording too short to decide). Users only need to know that `host_aligned` is a fallback: it is fine, but worth noting. **`decision`:** a readable reason; show it as it is. *Binding.*
25. **They stay far below 2^53 in practice,** but are unsigned 64-bit. Reading 64-bit integers is right. *Binding.*

## Connection

26. **One central at a time.** dLidar stops advertising while a phone is connected, so a second phone normally does not find it. If one connects anyway (by a cached address), it gets `too_many_clients` with `"id":null` on its first write and is disconnected. Not retrying on its own is right. *Provisional until spike measurement 3.*
27. **No idle timeout, and no keep-alive needed.** Link loss is detected by BLE's supervision timeout. *Binding.*
28. **Match on the service UUID, not the name.** The advertisement carries the service UUID and the name `dLidar`. Phase 2 adds a short device id to tell several scanners apart (exact form to come). The name may then move to the scan response, so filter by UUID. *Provisional.*
29. **Public and stable.** The dev scanner's controller address is public (`2C:CF:67:8A:FC:1B`) and does not change across reboots. Phase 3 may turn on BLE privacy for bonded phones; Android resolves that through the bond, so reconnect through the bonded device. *Provisional.*
30. **Yes.** The echo service and the real server always share the UUIDs, and change together if they change. *Binding.*

## Later phases

31. **One shared event, `result`, with `op` at top level** and `ok` plus `result` or `error`, shaped like a response: `{"event":"result","seq":9,"op":12,"ok":true,"result":{…}}`. It goes only to the connection that sent the request and is never left out. If that connection is lost first, the result is lost; the command's status query (for Wi-Fi: `wifi.status`) gives the outcome. *Binding.*
32. **`accepted:false` means "nothing to do"**: start while already recording, stop while idle. `state` says why. **`busy` means "not now, try again"**: start while the last recording is still being saved. *Provisional, phase 3.*
33. **Decided (user, 2026-11-02): every request needs the bond.** RX requires an encrypted link (`encrypt-write`), so an unbonded phone gets the ATT error "insufficient encryption" on its first write, and Android starts pairing by itself. That works only while the pairing window is open (the pairing button, or `[phone] pairing_at_start` until the button is mounted); outside it, pairing fails and Android reports it. There is no protocol error `not_paired`. dLidar keeps a small allow-list (about 4 phones); holding the pairing button long clears it. *Binding for phone-link-ble; the allow-list is phase 3.*
34. **Planned:**
    - WPA2/WPA3 transition networks: send `wpa2-psk`.
    - Out of scope for v1: hidden SSIDs, WPA-Enterprise, WEP (show them as not supported), and SSIDs that are not valid UTF-8 (leave them out).
    - The PSK is 8 to 63 printable ASCII characters, or 64 hex digits; anything else gets `bad_request`.
    - Result codes: `connected`, `wrong_password`, `not_found`, `timeout`, plus generic `failed`.

    *Provisional, phase 4.*
35. **Not designed yet.** Likely an opaque cursor string, with pages sized to stay under 16 KiB. *Phase 5.*
36. **`info`:** one line of space-separated `key=value`: `info mode=properties|acquire secure=0|1 mtu=N notifying=0|1 notify_fd=N devices=N`. **`flood` SIZE includes the `\n`** (12 to 4096 bytes). N is at most 10000. *Spike only; it is deleted after the spike.*
