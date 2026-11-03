#pragma once

#include "adapters/phone/ble_link.h"

#include <chrono>
#include <memory>
#include <string>

typedef struct _GMainContext GMainContext;

namespace dlidar::phone
{
    // The BlueZ side of the phone link over D-Bus (openspec phone-link-ble,
    // design D1, D8-D11), on the phone loop's thread only, asynchronous calls
    // only. It exports the GATT application (service, RX, TX), the
    // advertisement and the pairing agent, follows bluetoothd (name watch),
    // the adapter and its devices, and reports to BleLink. It also owns the
    // adapter's pairable state: false except inside the pairing window, and
    // set false again when another program turns it on (the Pairable guard).
    // Every problem prints one warning per cause; nothing here blocks.
    class BluezPeripheral : public BlePeripheral
    {
    public:
        struct Options
        {
            std::string name = "dLidar"; // advertised (BlueZ puts it in the scan response)
        };

        BluezPeripheral(GMainContext *context, Options options);
        ~BluezPeripheral() override; // unexports everything; on the loop thread

        // Where BlueZ's events go. Set before Start.
        void Attach(BleLink &link) { link_ = &link; }

        // Connects to the system bus and follows bluetoothd. On the loop thread.
        void Start();

        // Pairable for `seconds` with a Just Works agent; prints when it opens,
        // when a phone pairs and when it closes. If BlueZ is not there yet,
        // the window opens when it appears, for the time that remains.
        void OpenPairingWindow(std::chrono::seconds seconds);

        // Best effort, asynchronous: unregisters the advertisement, the
        // application and an open window's agent, sets Pairable false.
        // StopDone() turns true when BlueZ has answered every call.
        void Stop();
        bool StopDone() const;

        // BlePeripheral (called by BleLink).
        void Notify(const std::string &piece) override;
        void Disconnect(const std::string &device) override;
        void SetAdvertising(bool on) override;

        struct Impl; // the D-Bus state (bluez_peripheral.cpp)

    private:
        std::unique_ptr<Impl> impl_;
        BleLink *link_ = nullptr;
    };
}
