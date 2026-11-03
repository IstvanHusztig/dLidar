#pragma once

#include "adapters/networkmanager/command_runner.h"
#include "application/ports/wifi_manager.h"

#include <string>
#include <vector>

// What the NetworkManager adapter reads from and writes to nmcli, as pure
// functions (openspec phone-link-wifi, design D3-D6). The texts are those
// of NetworkManager 1.42 in the C locale.
namespace dlidar::network
{
    // A saved NetworkManager profile, as far as D4 needs it.
    struct Profile
    {
        std::string uuid;
        std::string type; // 802-11-wireless, 802-3-ethernet, ...
        std::string ssid; // Wi-Fi only
        std::string mode; // Wi-Fi only: infrastructure, ap, adhoc, mesh, or empty
    };

    // nmcli connection add, in memory and not autoconnecting until it is
    // saved: the only argument vector that holds the password.
    std::vector<std::string> AddArgs(const WifiNetwork &network, const std::string &uuid, const std::string &ifname);

    // The saved profiles a successful join of `ssid` replaces (D4): Wi-Fi,
    // infrastructure, exactly that SSID, not the new profile. Never an
    // Ethernet or access point profile, never another SSID.
    std::vector<std::string> ProfilesToReplace(const std::vector<Profile> &profiles, const std::string &ssid,
                                               const std::string &new_uuid);

    // The outcome of `nmcli --wait N connection up` (D5); `message` from
    // nmcli's first error line.
    WifiResult OutcomeFor(const CommandOutput &up);

    // The reason any other nmcli step failed, as a Failed result: the
    // polkit hint, NetworkManager not running, no Wi-Fi device, or nmcli's
    // first error line.
    WifiResult FailureFor(const CommandOutput &step, const std::string &ifname);

    // nmcli's first error line without its "Error: " prefix.
    std::string ErrorLine(const CommandOutput &o);

    // Undoes nmcli's terse escaping ("\:" and "\\").
    std::string UnescapeTerse(const std::string &s);

    // "192.168.1.106/24" -> "192.168.1.106".
    std::string AddressOnly(const std::string &cidr);

    // `nmcli -t -f GENERAL.STATE,GENERAL.CON-UUID,IP4.ADDRESS device show
    // IFNAME`: the state (ssid not set) and the active profile's uuid.
    WifiLink ParseLink(const CommandOutput &o, std::string &con_uuid);

    // `nmcli -t -f UUID,TYPE connection show`.
    std::vector<Profile> ParseProfiles(const std::string &out);

    // `text` with every occurrence of `secret` replaced by "***" (a
    // message that would echo the password).
    std::string Scrub(std::string text, const std::string &secret);
}
