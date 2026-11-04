#pragma once

#include <chrono>
#include <functional>
#include <string>

namespace dlidar
{
    enum class WifiSecurity
    {
        Open,
        Wpa2Psk, // also a WPA2/WPA3 transition network
        Wpa3Sae,
    };

    // A network to join, as the phone's scan found it (openspec
    // phone-link-wifi). The password is never logged.
    struct WifiNetwork
    {
        std::string ssid;
        WifiSecurity security = WifiSecurity::Wpa2Psk;
        std::string psk; // empty for Open
    };

    enum class WifiOutcome
    {
        Connected,
        WrongPassword,
        NotFound,
        Timeout,
        Failed,
    };

    struct WifiResult
    {
        WifiOutcome outcome = WifiOutcome::Failed;
        std::string ip;      // Connected: the IPv4 address, empty if none
        std::string message; // readable; never contains the password
    };

    enum class WifiState
    {
        Unavailable, // no Wi-Fi device, or NetworkManager cannot be reached
        Disconnected,
        Connecting,
        Connected,
    };

    struct WifiLink
    {
        WifiState state = WifiState::Unavailable;
        std::string ssid; // empty unless connected or connecting
        std::string ip;   // empty unless connected
    };

    // The scanner's Wi-Fi, as the remote API needs it (design D2): the
    // networkmanager adapter implements it. Thread-safe.
    class WifiManager
    {
    public:
        virtual ~WifiManager() = default;

        // The link as last seen; never blocks.
        virtual WifiLink Link() const = 0;

        // Joins `network`, saving it only on success; a failure leaves the
        // saved networks as they were and rejoins the previous one. `done` is
        // called exactly once, on any thread, within `limit` (not after
        // the adapter was stopped). One call at a time.
        virtual void Connect(const WifiNetwork &network, std::chrono::seconds limit,
                             std::function<void(const WifiResult &)> done) = 0;
    };
}
