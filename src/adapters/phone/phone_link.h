#pragma once

#include "application/ports/device_events.h"
#include "application/remote/device_api.h"

#include <chrono>
#include <memory>
#include <string>

namespace dlidar::phone
{
    // The [phone] settings the link needs for Bluetooth LE.
    struct BleOptions
    {
        bool enabled = false;          // [phone] ble
        std::string name = "dLidar";   // [phone] name
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
        // warn and recover later; they never fail the start.
        static std::unique_ptr<PhoneLink> Start(const DeviceApi &api, const std::string &socket_dir,
                                                const BleOptions &ble, std::string &socket_why);
        ~PhoneLink(); // Shutdown()

        std::shared_ptr<DeviceEvents> Events() const;
        std::string SocketPath() const;

        // Opens the Bluetooth pairing window for `seconds` (SIGUSR1, later the
        // pairing button). With BLE off only a warning. Thread-safe.
        void OpenPairingWindow(std::chrono::seconds seconds = std::chrono::seconds(120));
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
