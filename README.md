# dLidar

**This code is not production ready. The project is in development.**

dLidar is the software of a handheld LiDAR scanner. It runs on a Raspberry Pi 5 (at least 2 GB RAM), records a **Livox Mid-360S** or a **Unitree L2** (point cloud and IMU) into one LAZ file and one IMU CSV per recording, and is controlled by a push button with a status LED. A phone app can watch it, start and stop recordings, and put it on a Wi-Fi network over Bluetooth LE. The recordings are processed by [dMapper](https://github.com/IstvanHusztig/dMapper), our SLAM solution.

The full manual is [`docs/manual.html`](docs/manual.html).

## Status

| Part | State |
|---|---|
| Recording, LAZ/CSV output, recovery after a crash or power loss | done; tested on the Unitree L2 and by replaying its packet logs |
| Livox Mid-360S | done; records on the real sensor since 2026-11-09, bring-up checks of the axes, timing and resources still open ([`docs/livox_mid360s.md`](docs/livox_mid360s.md)) |
| Unitree L2 IMU timestamps on the L2's clock | done; not yet verified on hardware (the test unit's IMU is faulty) |
| Push button, status LED, service mode | done; not yet tested with the real parts |
| Phone link: status, start/stop, Wi-Fi setup, log over Bluetooth LE and the local socket | done; tested with the Android app over Bluetooth LE |
| Pairing button | done; not yet tested with the real button (`tools/pair.sh` is tested) |
| Later | listing and removing saved Wi-Fi networks, listing recordings and power-off from the phone, a live log stream |

The open hardware checks are the remaining tasks in `openspec/changes/` (`push-button-recording`, `phone-link-commands`, `phone-link-core`, `add-livox-mid360s-sensor`).

## Build

```bash
git submodule update --init --recursive    # LASzip, Livox-SDK2, nlohmann/json
sudo apt install libglib2.0-dev libsystemd-dev  # GIO (phone link, Bluetooth); sd-journal (log.get)
cmake --preset ninja                       # configures into build/
cmake --build build                        # bin/dLidar
ctest --test-dir build --output-on-failure # no sensor needed
```

## Settings

dLidar reads [`dlidar.toml`](dlidar.toml) in the project folder (or `--config PATH`): which sensor, its network, the mount, the button, LED and pairing button pins, and the phone link (`[phone] ble`, `name`). Every key is listed there, with the values of the dev board's scanner (its Mid-360S at `192.168.1.109`, sending to `192.168.1.5`) and otherwise the defaults; the manual ("Settings") gives every default.

## Run

**As a service** (the normal case on the device): dLidar starts at boot, waits for the button, and is visible to the phone. Short press: start; next short press: stop and save; hold 5 s: save and power down. Installation (once): see the manual, "Running as a service". It installs `systemd/dlidar.service`, the power-off rule and the Wi-Fi setup rule, and sets `ControllerMode = le` (Bluetooth LE only) in `/etc/bluetooth/main.conf`.

```bash
systemctl status dlidar
journalctl -fu dlidar                      # what it does, live
```

**By hand** (tests over SSH): only one dLidar can run at a time, so stop the service first.

```bash
sudo systemctl stop dlidar
./bin/dLidar --once                        # one recording at once, Ctrl-C saves it
sudo systemctl start dlidar
```

Recordings go to `~/PointCloudDump/` (numbered `lidarNNNN.laz`, `imuNNNN.csv` and diagnostics).

## Phone link

dLidar serves a protocol (state, progress, warnings, summary, free space, sensor; `recording.start` and `recording.stop`; `wifi.connect` and `wifi.status`; `log.get`) over Bluetooth LE and a local socket. Any bonded phone may start and stop; the journal says who did (`phone: start`). The contract for app authors is [`docs/phone-protocol.md`](docs/phone-protocol.md).

- **Bluetooth LE:** the scanner advertises as `dLidar` while no phone is connected, one phone at a time, **bonded phones only**.
- **Pairing a phone:** press the pairing button (GPIO 22, pin 15; or run `tools/pair.sh` on the Pi), then within 2 minutes connect with the app and tap **Pair** (no PIN). The LED flashes twice a second while the window is open. Outside this window the Pi refuses every pairing. A phone that was unpaired on the phone, or a lost one: hold the pairing button 5 s (or `tools/pair.sh --forget`), which forgets every bonded phone first.
- **The log on the phone:** `log.get` pages through the service's journal lines and systemd's lines about it (start, crash, restart), newest first; nothing else from the system.
- **Wi-Fi from the phone:** the app scans, you pick a network and type its password, and the scanner joins it through NetworkManager. It saves the network only when it joined, keeps the networks it already knows, and after a wrong password rejoins the network it was on.
- **On the Pi itself:** `tools/phone_client.py status.get` (or `--follow` for events) talks to the socket `/run/dlidar/dlidar.sock`.

## Documentation

- [`docs/manual.html`](docs/manual.html): how a recording works, the files, settings, wiring, the service, the phone link, failures.
- [`docs/phone-protocol.md`](docs/phone-protocol.md): the phone protocol.
- [`docs/recording-format.md`](docs/recording-format.md): every output file and field.
- [`docs/architecture.md`](docs/architecture.md): the code's layers and how to extend them.
- [`docs/livox_mid360s.md`](docs/livox_mid360s.md): the Mid-360S and its bring-up.
- [`CLAUDE.md`](CLAUDE.md): the reference for development with Claude Code.
