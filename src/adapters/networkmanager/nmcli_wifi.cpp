#include "adapters/networkmanager/nmcli_wifi.h"

#include <algorithm>
#include <fstream>
#include <iostream>

namespace dlidar::network
{
    namespace
    {
        std::string KernelUuid()
        {
            std::ifstream in("/proc/sys/kernel/random/uuid");
            std::string u;
            std::getline(in, u);
            return u;
        }

        // A network name for a log line: control characters become '?'.
        std::string Printable(std::string s)
        {
            for (char &ch : s)
                if ((unsigned char)ch < 0x20 || ch == 0x7f)
                    ch = '?';
            return s;
        }
    }

    NmcliWifi::NmcliWifi(std::unique_ptr<CommandRunner> runner, Options options)
        : runner_(std::move(runner)), options_(std::move(options))
    {
        if (!options_.new_uuid)
            options_.new_uuid = KernelUuid;
    }

    NmcliWifi::~NmcliWifi() { Stop(); }

    void NmcliWifi::Start()
    {
        if (!worker_.joinable())
            worker_ = std::thread([this] { Loop(); });
    }

    void NmcliWifi::Stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ && !worker_.joinable())
                return;
            stopping_ = true;
        }
        wake_.notify_all();
        runner_->Cancel();
        if (worker_.joinable())
            worker_.join();
    }

    WifiLink NmcliWifi::Link() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return link_;
    }

    void NmcliWifi::Connect(const WifiNetwork &network, std::chrono::seconds limit,
                            std::function<void(const WifiResult &)> done)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!stopping_ && !job_)
            {
                job_ = Job{network, limit, std::move(done)};
                done = nullptr;
            }
        }
        if (done) // stopped, or a join already runs (WifiSetup prevents that)
            done(WifiResult{WifiOutcome::Failed, "", "the Wi-Fi setup is not running"});
        else
            wake_.notify_all();
    }

    bool NmcliWifi::Stopping() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopping_;
    }

    void NmcliWifi::Loop()
    {
        Refresh();
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stopping_)
        {
            if (!job_)
                wake_.wait_for(lock, options_.refresh, [this] { return stopping_ || job_.has_value(); });
            if (stopping_)
                break;
            std::optional<Job> job;
            job.swap(job_);
            lock.unlock();
            if (job)
            {
                WifiResult r = Join(*job);
                job->network.psk.assign(job->network.psk.size(), '\0');
                Refresh();
                if (!Stopping() && job->done)
                    job->done(r);
            }
            else
                Refresh();
            lock.lock();
        }
    }

    CommandOutput NmcliWifi::Nmcli(std::vector<std::string> args, std::chrono::milliseconds timeout)
    {
        args.insert(args.begin(), "nmcli");
        return runner_->Run(args, timeout);
    }

    std::string NmcliWifi::SsidOf(const std::string &uuid)
    {
        if (uuid == ssid_uuid_)
            return ssid_of_uuid_;
        CommandOutput o = Nmcli({"-g", "802-11-wireless.ssid", "connection", "show", "uuid", uuid});
        if (o.exit_code != 0)
            return "";
        ssid_uuid_ = uuid;
        ssid_of_uuid_ = UnescapeTerse(o.out.substr(0, o.out.find('\n')));
        return ssid_of_uuid_;
    }

    void NmcliWifi::Refresh()
    {
        if (Stopping())
            return;
        std::string uuid;
        WifiLink link = ParseLink(
            Nmcli({"-t", "-f", "GENERAL.STATE,GENERAL.CON-UUID,IP4.ADDRESS", "device", "show", options_.ifname}, std::chrono::seconds(5)),
            uuid);
        if (!uuid.empty())
            link.ssid = SsidOf(uuid);
        std::lock_guard<std::mutex> lock(mutex_);
        link_ = link;
    }

    // D3: remember, add in memory, activate; then save and replace, or
    // delete and rejoin the previous network. Within job.limit.
    WifiResult NmcliWifi::Join(const Job &job)
    {
        const WifiNetwork &n = job.network;
        const std::string &ifname = options_.ifname;
        auto deadline = std::chrono::steady_clock::now() + job.limit;

        // 1. Remember the profile to come back to, and the saved profiles.
        CommandOutput active = Nmcli({"-g", "GENERAL.CON-UUID", "device", "show", ifname});
        if (active.exit_code != 0)
            return FailureFor(active, ifname);
        std::string previous = UnescapeTerse(active.out.substr(0, active.out.find('\n')));
        CommandOutput list = Nmcli({"-t", "-f", "UUID,TYPE", "connection", "show"});
        if (list.exit_code != 0)
            return FailureFor(list, ifname);
        std::vector<Profile> profiles = ParseProfiles(list.out);
        for (Profile &p : profiles)
        {
            if (p.type != "802-11-wireless")
                continue;
            CommandOutput o = Nmcli({"-g", "802-11-wireless.ssid,802-11-wireless.mode", "connection", "show", "uuid", p.uuid});
            size_t nl = o.out.find('\n');
            p.ssid = UnescapeTerse(o.out.substr(0, nl));
            if (nl != std::string::npos)
                p.mode = o.out.substr(nl + 1, o.out.find('\n', nl + 1) - nl - 1);
        }
        if (Stopping())
            return WifiResult{};

        // 2. Add it in memory only.
        std::string uuid = options_.new_uuid();
        CommandOutput add = runner_->Run(AddArgs(n, uuid, ifname), std::chrono::seconds(10));
        if (add.exit_code != 0)
        {
            WifiResult r = FailureFor(add, ifname);
            r.message = Scrub(r.message, n.psk);
            return r;
        }

        // 3. Activate it; nmcli waits, NetworkManager uses its own scan list.
        auto left = std::chrono::duration_cast<std::chrono::seconds>(deadline - std::chrono::steady_clock::now());
        long wait_s = std::max<long>(5, (long)(left - options_.cleanup).count());
        CommandOutput up = Nmcli({"--wait", std::to_string(wait_s), "connection", "up", "uuid", uuid, "ifname", ifname},
                                 std::chrono::seconds(wait_s + 5));
        WifiResult r = Stopping() ? WifiResult{} : OutcomeFor(up);
        r.message = Scrub(r.message, n.psk);

        if (r.outcome == WifiOutcome::Connected)
        {
            // 4. Joined: its address, save it, replace the older profiles of that name.
            CommandOutput ip = Nmcli({"-g", "IP4.ADDRESS", "device", "show", ifname});
            r.ip = AddressOnly(UnescapeTerse(ip.out.substr(0, ip.out.find('\n'))));
            CommandOutput save = Nmcli({"connection", "modify", "uuid", uuid, "connection.autoconnect", "yes"});
            if (save.exit_code != 0)
            {
                std::cerr << "WARNING: Wi-Fi: joined " << Printable(n.ssid) << " but could not save it ("
                          << Scrub(ErrorLine(save), n.psk) << "); it is forgotten at the next reboot" << std::endl;
                return r;
            }
            for (const std::string &old : ProfilesToReplace(profiles, n.ssid, uuid))
            {
                CommandOutput del = Nmcli({"connection", "delete", "uuid", old});
                if (del.exit_code != 0)
                    std::cerr << "WARNING: Wi-Fi: could not remove the older profile of " << Printable(n.ssid) << " ("
                              << ErrorLine(del) << ")" << std::endl;
            }
            return r;
        }

        // 5. Not joined (or dLidar is stopping): remove the attempt and
        // rejoin the previous network.
        CommandOutput del = Nmcli({"connection", "delete", "uuid", uuid},
                                  Stopping() ? std::chrono::seconds(1) : std::chrono::seconds(10));
        if (del.exit_code != 0)
            std::cerr << "WARNING: Wi-Fi: could not remove the failed attempt for " << Printable(n.ssid) << " ("
                      << ErrorLine(del) << "); it is gone after the next reboot" << std::endl;
        if (!previous.empty() && previous != uuid && !Stopping())
        {
            CommandOutput back = Nmcli({"--wait", "10", "connection", "up", "uuid", previous, "ifname", ifname},
                                       std::chrono::seconds(15));
            if (back.exit_code != 0)
                std::cerr << "WARNING: Wi-Fi: could not rejoin the previous network yet (" << ErrorLine(back)
                          << "); NetworkManager keeps trying" << std::endl;
        }
        return r;
    }
}
