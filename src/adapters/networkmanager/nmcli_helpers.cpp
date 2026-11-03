#include "adapters/networkmanager/nmcli_helpers.h"

#include <cstdlib>

namespace dlidar::network
{
    namespace
    {
        bool Contains(const std::string &s, const char *part) { return s.find(part) != std::string::npos; }

        const char *const kPolkitHint = "dLidar may not change the network: install systemd/50-dlidar-network.rules";
    }

    std::vector<std::string> AddArgs(const WifiNetwork &network, const std::string &uuid, const std::string &ifname)
    {
        std::vector<std::string> a{"nmcli",           "connection", "add",          "type",  "wifi",
                                   "ifname",          ifname,       "con-name",     network.ssid, "ssid",
                                   network.ssid,      "connection.uuid", uuid,      "connection.autoconnect", "no"};
        if (network.security != WifiSecurity::Open)
        {
            a.insert(a.end(), {"wifi-sec.key-mgmt", network.security == WifiSecurity::Wpa3Sae ? "sae" : "wpa-psk",
                               "wifi-sec.psk", network.psk});
        }
        a.insert(a.end(), {"save", "no"});
        return a;
    }

    std::vector<std::string> ProfilesToReplace(const std::vector<Profile> &profiles, const std::string &ssid,
                                               const std::string &new_uuid)
    {
        std::vector<std::string> v;
        for (const Profile &p : profiles)
            if (p.type == "802-11-wireless" && (p.mode.empty() || p.mode == "infrastructure") && p.ssid == ssid &&
                p.uuid != new_uuid && !p.uuid.empty())
                v.push_back(p.uuid);
        return v;
    }

    std::string ErrorLine(const CommandOutput &o)
    {
        const std::string &text = o.err.empty() ? o.out : o.err;
        std::string line = text.substr(0, text.find('\n'));
        const std::string prefix = "Error: ";
        if (line.compare(0, prefix.size(), prefix) == 0)
            line.erase(0, prefix.size());
        return line;
    }

    WifiResult FailureFor(const CommandOutput &step, const std::string &ifname)
    {
        WifiResult r;
        r.outcome = WifiOutcome::Failed;
        std::string text = step.err + step.out;
        if (Contains(text, "Insufficient privileges") || Contains(text, "Not authorized"))
            r.message = kPolkitHint;
        else if (step.exit_code == 127)
            r.message = "nmcli cannot be run (" + ErrorLine(step) + ")";
        else if (step.exit_code == 8 || Contains(text, "NetworkManager is not running") ||
                 Contains(text, "Could not create NMClient"))
            r.message = "NetworkManager is not running";
        else if (step.exit_code == 10 && Contains(text, "Device '"))
            r.message = "no Wi-Fi device " + ifname;
        else if (step.timed_out)
            r.message = "nmcli did not answer in time";
        else
        {
            std::string line = ErrorLine(step);
            r.message = line.empty() ? "nmcli failed (exit " + std::to_string(step.exit_code) + ")" : line;
        }
        return r;
    }

    WifiResult OutcomeFor(const CommandOutput &up)
    {
        WifiResult r;
        std::string text = up.err + up.out;
        if (up.exit_code == 0 && !up.timed_out)
            r.outcome = WifiOutcome::Connected;
        else if (up.exit_code == 3)
        {
            r.outcome = WifiOutcome::Timeout;
            r.message = "the network did not answer in time";
        }
        else if (Contains(text, "Secrets were required") || Contains(text, "supplicant disconnected"))
        {
            r.outcome = WifiOutcome::WrongPassword;
            r.message = "the network rejected the password";
        }
        else if (Contains(text, "could not be found"))
        {
            r.outcome = WifiOutcome::NotFound;
            r.message = "the network was not found";
        }
        else
            r = FailureFor(up, "");
        return r;
    }

    std::string UnescapeTerse(const std::string &s)
    {
        std::string r;
        for (size_t i = 0; i < s.size(); i++)
        {
            if (s[i] == '\\' && i + 1 < s.size())
                i++;
            r += s[i];
        }
        return r;
    }

    std::string AddressOnly(const std::string &cidr) { return cidr.substr(0, cidr.find('/')); }

    WifiLink ParseLink(const CommandOutput &o, std::string &con_uuid)
    {
        WifiLink link;
        con_uuid.clear();
        if (o.exit_code != 0)
            return link; // unavailable
        int state = 0;
        size_t pos = 0;
        while (pos < o.out.size())
        {
            size_t end = o.out.find('\n', pos);
            std::string line = o.out.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
            pos = end == std::string::npos ? o.out.size() : end + 1;
            size_t colon = line.find(':');
            if (colon == std::string::npos)
                continue;
            std::string key = line.substr(0, colon), value = line.substr(colon + 1);
            if (key == "GENERAL.STATE")
                state = atoi(value.c_str());
            else if (key == "GENERAL.CON-UUID" && value != "--")
                con_uuid = UnescapeTerse(value);
            else if (key.compare(0, 11, "IP4.ADDRESS") == 0 && link.ip.empty())
                link.ip = AddressOnly(UnescapeTerse(value));
        }
        // NetworkManager's device states: 10 unmanaged, 20 unavailable (radio
        // off), 30 disconnected, 40-90 activating, 100 activated, 110
        // deactivating, 120 failed.
        if (state == 100)
            link.state = WifiState::Connected;
        else if (state >= 40 && state < 100)
            link.state = WifiState::Connecting;
        else if (state == 30 || state == 110 || state == 120)
            link.state = WifiState::Disconnected;
        else
            link.state = WifiState::Unavailable;
        if (link.state != WifiState::Connected)
            link.ip.clear();
        if (link.state == WifiState::Unavailable || link.state == WifiState::Disconnected)
            con_uuid.clear();
        return link;
    }

    std::vector<Profile> ParseProfiles(const std::string &out)
    {
        std::vector<Profile> v;
        size_t pos = 0;
        while (pos < out.size())
        {
            size_t end = out.find('\n', pos);
            std::string line = out.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
            pos = end == std::string::npos ? out.size() : end + 1;
            // UUIDs have no ':'; the type is the rest.
            size_t colon = line.find(':');
            if (colon == std::string::npos || colon == 0)
                continue;
            v.push_back(Profile{line.substr(0, colon), UnescapeTerse(line.substr(colon + 1)), "", ""});
        }
        return v;
    }

    std::string Scrub(std::string text, const std::string &secret)
    {
        if (secret.empty())
            return text;
        for (size_t at; (at = text.find(secret)) != std::string::npos;)
            text.replace(at, secret.size(), "***");
        return text;
    }
}
