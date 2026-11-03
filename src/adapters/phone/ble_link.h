#pragma once

#include "adapters/phone/connection.h"
#include "adapters/phone/phone_server.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>

namespace dlidar::phone
{
    // What BleLink asks of the BlueZ glue (openspec phone-link-ble, design
    // D2). On the phone loop's thread.
    class BlePeripheral
    {
    public:
        virtual ~BlePeripheral() = default;
        // One notification on TX (PropertiesChanged on Value): reaches every
        // subscribed device (D3).
        virtual void Notify(const std::string &piece) = 0;
        // Device1.Disconnect.
        virtual void Disconnect(const std::string &device) = 0;
        // Register or unregister the advertisement.
        virtual void SetAdvertising(bool on) = 0;
    };

    // How the glue answers a WriteValue: Ok, or the D-Bus error to return
    // (org.bluez.Error.NotPermitted / InvalidOffset).
    enum class WriteResult
    {
        Ok,
        NotPermitted,
        InvalidOffset
    };

    // The phone link over BLE as a pure state machine (design D2, D4-D7): one
    // phone at a time, its connection to the protocol server, the MTU, paced
    // notifications. The glue reports BlueZ's events; BleLink decides and
    // calls BlePeripheral and the server's ConnectionHandler. No GLib, no
    // D-Bus: unit-tested with fakes. Not thread-safe: the loop thread only.
    //
    // - The first device that connects is the phone; advertising is off while
    //   any device is connected. A further device is disconnected and its
    //   writes are rejected (D5).
    // - The connection opens when the phone is connected and TX notifications
    //   are on, and closes when they go off, the phone leaves or BlueZ goes.
    //   A phone connected without notifications for `idle` is disconnected,
    //   so a leftover link never keeps the scanner from advertising.
    // - A message is split into notifications of at most MTU-3 bytes, the
    //   first piece fresh (D4, D6); the MTU is 23 until a write reports it.
    // - Pieces leave at `rate` bytes/s, drained on clock wakes in `tick`
    //   steps (a token bucket with a burst of one tick, which may run one
    //   piece into debt). Past `cap` queued bytes the connection is closed and
    //   the phone disconnected.
    class BleLink
    {
    public:
        static const uint16_t kMinMtu = 23;

        struct Options
        {
            double rate_bytes_per_s = 24 * 1024; // measured: 54 KiB/s at MTU 517 (design Measurements)
            size_t cap_bytes = 64 * 1024;
            std::chrono::milliseconds tick{10};
            // A connected phone not subscribed this long is disconnected; long
            // enough to answer Android's pairing prompt (design D5).
            std::chrono::seconds idle{30};
        };

        BleLink(ConnectionHandler &handler, BlePeripheral &peripheral, ServerClock &clock, Options options);
        BleLink(ConnectionHandler &handler, BlePeripheral &peripheral, ServerClock &clock)
            : BleLink(handler, peripheral, clock, Options()) {}
        ~BleLink(); // an open connection is closed (OnClosed)

        BleLink(const BleLink &) = delete;
        BleLink &operator=(const BleLink &) = delete;

        // BlueZ is there (registered) or gone. The glue reports the devices
        // already connected (DeviceConnected) before BluezUp.
        void BluezUp();
        void BluezDown();

        void DeviceConnected(const std::string &device);
        void DeviceDisconnected(const std::string &device);

        // TX notifications on or off (StartNotify / StopNotify).
        void NotifyStarted();
        void NotifyStopped();

        // A WriteValue on RX. `mtu` 0: not reported.
        WriteResult Write(const std::string &device, std::string_view bytes, uint16_t mtu, uint16_t offset);

        // A ServerClock wake: a deferred close, then the drain.
        void OnWake();

        bool Open() const { return conn_ != nullptr; }
        const std::string &Phone() const { return phone_; }
        uint16_t Mtu() const { return mtu_; }
        bool Advertising() const { return advertising_; }

    private:
        class Conn;

        ConnectionHandler &handler_;
        BlePeripheral &peripheral_;
        ServerClock &clock_;
        Options options_;

        bool up_ = false;
        bool advertising_ = false;
        bool notifying_ = false;
        std::set<std::string> connected_;
        std::string phone_;
        uint16_t mtu_ = kMinMtu;

        std::unique_ptr<Conn> conn_;
        bool close_pending_ = false; // the server closed it; OnClosed on the next wake
        std::optional<ServerClock::TimePoint> idle_since_; // the phone is connected but not subscribed

        std::deque<std::string> queue_; // pieces waiting to be notified
        size_t queued_ = 0;             // their bytes
        double tokens_ = 0;
        ServerClock::TimePoint refilled_{};

        void UpdateAdvertising();
        void StartIdle(); // the phone is there without a connection: disconnect it if that lasts
        void TryOpen();
        void CloseNow(); // OnClosed at once (BlueZ events, never inside a server call)
        void RequestClose();
        void Queue(const std::string &message);
        void Drain();
        double Burst() const; // the bucket's size: one tick at the rate
    };
}
