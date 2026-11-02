# BLE spike (throwaway)

Tasks 1.1 and 1.2 of `openspec/changes/phone-link-core`. This program is not part of dLidar. It is deleted once the findings are in `design.md` ("Spike Findings").

```bash
cmake -DDLIDAR_BLE_SPIKE=ON build && cmake --build build --target ble_spike
bin/ble_spike                 # PropertiesChanged notifications
bin/ble_spike --acquire       # AcquireNotify notifications
bin/ble_spike --secure        # RX needs encryption; Just Works agent
# as the service would run it (systemd, the service user):
sudo systemd-run --uid=vs2022 --unit=ble-spike --collect -p WorkingDirectory=$PWD bin/ble_spike
journalctl -fu ble-spike
```

Service `b06b0001-1294-4457-85c4-ef6a46d0c127`, advertised as `dLidar`:

| Characteristic | UUID | Use |
|---|---|---|
| RX | `b06b0002-…c127` | write, write-without-response: lines ending in `\n` |
| TX | `b06b0003-…c127` | notify: `echo <line>\n`, `info …\n`, flood messages |

In nRF Connect: Scanner → `dLidar` → Connect → the unknown service. Enable notifications on TX (the triple arrow). Then write to RX (the up arrow), as **UTF-8 text** or as bytes, and end the line with `0A`.

## What to measure (task 1.2)

1. **MTU:** request an MTU of 247 in nRF Connect (menu → Request MTU). Write `info\n`; the spike logs `mtu=` and answers with it. Note the largest single notification the phone receives.
2. **Throughput:** write `flood 200 300\n` at MTU 23 and at MTU 247. The spike logs how long it took to hand the data to BlueZ. In the phone log, check that the `seq=` numbers 1–200 all arrived, and the time from the first to the last one.
3. **Two phones:** connect a second phone, or a second app, while the first one stays connected. Enable TX notifications on both, then write `echo-test\n` from one. Which phone receives the echo? Repeat with `--acquire`. Does the second phone even see the device while the first is connected?
4. **Reconnect:** walk out of range, or switch Bluetooth off on the phone, then come back. The spike logs `Connected` false/true. Do notifications work again without restarting the spike?
5. **Bonding:** run with `--secure`. Write to RX: the phone should be asked to pair (Just Works), and the agent logs it. Disconnect and reconnect: is it still bonded, with no new prompt? Remove the bond on the phone: what happens?
6. **Wi-Fi switch:** keep the phone connected, then switch the Pi to another known Wi-Fi network (`nmcli connection up <name>`) and back. The Pi never scans. Does the BLE link drop, and do notifications stall during the switch?
