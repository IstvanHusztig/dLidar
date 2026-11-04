#pragma once

#include "application/ports/wifi_manager.h"

#include <cctype>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace dlidar
{
    // How long a Wi-Fi join may take, the cleanup after a failure included:
    // the phone is told this (timeout_s) and waits that long.
    const std::chrono::seconds kWifiConnectLimit{60};

    // An argument of wifi.connect that is not acceptable. The message never
    // contains the argument's value.
    struct WifiArgError
    {
        std::string argument; // "ssid", "security" or "psk"
        std::string message;
    };

    // The most recent join since dLidar started; no outcome yet: still running.
    struct WifiLast
    {
        std::string ssid;
        std::optional<WifiOutcome> outcome;
    };

    struct WifiStatusView
    {
        WifiLink link;
        std::optional<WifiLast> last; // empty: no join since dLidar started
    };

    // Wi-Fi setup by a remote client (openspec phone-link-wifi, design D2):
    // the argument rules, one join at a time, and what wifi.status reports.
    // Independent of any transport or message format. Thread-safe. It must
    // outlive the adapter's callbacks (stop the adapter first).
    class WifiSetup
    {
    public:
        enum class Begun
        {
            Accepted,
            Busy, // another join is still running
        };

        explicit WifiSetup(WifiManager &manager) : manager_(manager) {}

        // wifi.connect's argument rules; an empty optional is a missing
        // argument. Fills `out` when they hold.
        static std::optional<WifiArgError> Check(const std::optional<std::string> &ssid,
                                                 const std::optional<WifiSecurity> &security,
                                                 const std::optional<std::string> &psk, WifiNetwork &out)
        {
            if (!ssid)
                return WifiArgError{"ssid", "wifi.connect needs \"ssid\""};
            if (ssid->empty() || ssid->size() > 32)
                return WifiArgError{"ssid", "\"ssid\" must be 1 to 32 bytes"};
            if (!security)
                return WifiArgError{"security", "wifi.connect needs \"security\""};
            if (*security == WifiSecurity::Open)
            {
                if (psk)
                    return WifiArgError{"psk", "\"psk\" must be left out for an open network"};
            }
            else if (!psk)
                return WifiArgError{"psk", "\"psk\" is needed for a network with a password"};
            else if (!PskValid(*psk))
                return WifiArgError{"psk", "\"psk\" must be 8 to 63 printable ASCII characters or 64 hexadecimal digits"};
            out.ssid = *ssid;
            out.security = *security;
            out.psk = psk ? *psk : std::string();
            return std::nullopt;
        }

        // Starts joining `network` (already checked); `done` gets the outcome
        // once, on the adapter's thread, within kWifiConnectLimit.
        Begun Begin(const WifiNetwork &network, std::function<void(const WifiResult &)> done)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (running_)
                    return Begun::Busy;
                running_ = true;
                last_ = WifiLast{network.ssid, std::nullopt};
            }
            manager_.Connect(network, kWifiConnectLimit,
                             [this, ssid = network.ssid, done = std::move(done)](const WifiResult &r) {
                                 {
                                     std::lock_guard<std::mutex> lock(mutex_);
                                     running_ = false;
                                     last_ = WifiLast{ssid, r.outcome};
                                 }
                                 if (done)
                                     done(r);
                             });
            return Begun::Accepted;
        }

        // The link; while a join runs, "connecting" to its network.
        WifiStatusView Status() const
        {
            WifiStatusView v;
            v.link = manager_.Link();
            std::lock_guard<std::mutex> lock(mutex_);
            if (running_)
                v.link = WifiLink{WifiState::Connecting, last_->ssid, ""};
            v.last = last_;
            return v;
        }

        // 8 to 63 printable ASCII characters (a passphrase) or 64 hex digits
        // (a raw key).
        static bool PskValid(const std::string &psk)
        {
            if (psk.size() == 64)
            {
                for (char ch : psk)
                    if (!std::isxdigit(static_cast<unsigned char>(ch)))
                        return false;
                return true;
            }
            if (psk.size() < 8 || psk.size() > 63)
                return false;
            for (char ch : psk)
                if (ch < 0x20 || ch > 0x7e)
                    return false;
            return true;
        }

    private:
        WifiManager &manager_;
        mutable std::mutex mutex_;
        bool running_ = false;
        std::optional<WifiLast> last_;
    };
}
