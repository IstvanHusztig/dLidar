#pragma once

#include "application/ports/device_events.h"
#include "application/ports/log_reader.h"
#include "application/remote/device_api.h"
#include "application/remote/wifi_setup.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace dlidar::phone
{
    // The [phone] settings the link needs for Bluetooth LE.
    struct BleOptions
    {
        bool enabled = false;          // [phone] ble
        std::string name = "dLidar";   // [phone] name
        // Called on the phone loop thread: true when the pairing window is
        // really open, false when it closes or BlueZ goes away (the LED).
        std::function<void(bool open)> on_pairing;
    };

    // The phone link as the composition root sees it (design D6): the loop
    // thread, the protocol server, the local socket and Bluetooth LE.
    // Events() goes to hub.AddAsync. A problem only prints a warning; dLidar
    // records as usual.
    class PhoneLink
    {
    public:
        // Listens in `socket_dir` (SocketDirectory() in production) and, if
        // enabled, over BLE (openspec phone-link-ble, design D12). A socket
        // problem sets `socket_why`; BLE still starts unless another dLidar
        // answers on the socket (two GATT services with the same UUIDs would
        // confuse the phone). Null if nothing runs. Bluetooth problems only
        // warn and recover later; they never fail the start. With `wifi`
        // it also serves wifi.connect and wifi.status (openspec
        // phone-link-wifi); stop the Wi-Fi adapter before destroying the link.
        // With `log`, log.get (openspec phone-link-logs); it must outlive the link.
        static std::unique_ptr<PhoneLink> Start(const DeviceApi &api, const std::string &socket_dir,
                                                const BleOptions &ble, std::string &socket_why,
                                                WifiSetup *wifi = nullptr, LogReader *log = nullptr);
        ~PhoneLink(); // Shutdown()

        std::shared_ptr<DeviceEvents> Events() const;
        std::string SocketPath() const;

        // Opens the Bluetooth pairing window for `seconds`; with `forget`,
        // first removes every bonded phone (openspec phone-link-commands,
        // design D4). The one entry for the pairing button, SIGUSR1 and
        // SIGUSR2. With BLE off only a warning. Thread-safe.
        void OpenPairingWindow(bool forget = false, std::chrono::seconds seconds = std::chrono::seconds(120));
        size_t Connections();

        // Delivers what is pending, the final idle or error state included,
        // for up to `flush`, then unregisters from BlueZ (up to 300 ms more),
        // closes the connections, removes the socket and stops the thread.
        // Idempotent.
        void Shutdown(std::chrono::milliseconds flush = std::chrono::milliseconds(200));

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        explicit PhoneLink(std::unique_ptr<Impl> impl);
    };
}
