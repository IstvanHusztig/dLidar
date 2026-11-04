#pragma once

#include "adapters/networkmanager/command_runner.h"
#include "adapters/networkmanager/nmcli_helpers.h"
#include "application/ports/wifi_manager.h"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace dlidar::network
{
    // The WifiManager port over NetworkManager's nmcli (openspec
    // phone-link-wifi, design D1, D3-D6). One worker thread runs one job at
    // a time: a join, or a refresh of the cached link every `refresh`.
    // Never scans: a join adds an in-memory profile and activates it, saves
    // it only when joined, and otherwise deletes it and rejoins the network
    // the device was on.
    class NmcliWifi : public WifiManager
    {
    public:
        struct Options
        {
            std::string ifname = "wlan0";
            std::chrono::milliseconds refresh{5000};
            std::chrono::seconds cleanup{15}; // of a join's limit, kept for the cleanup
            std::function<std::string()> new_uuid; // default: /proc/sys/kernel/random/uuid
        };

        NmcliWifi(std::unique_ptr<CommandRunner> runner, Options options);
        ~NmcliWifi() override; // Stop()

        // Starts the worker; the first refresh runs at once.
        void Start();
        // Cancels a running join (its unsaved profile is deleted, best
        // effort, and `done` is not called) and joins the worker. Idempotent.
        void Stop();

        WifiLink Link() const override;
        void Connect(const WifiNetwork &network, std::chrono::seconds limit,
                     std::function<void(const WifiResult &)> done) override;

    private:
        struct Job
        {
            WifiNetwork network;
            std::chrono::seconds limit;
            std::function<void(const WifiResult &)> done;
        };

        std::unique_ptr<CommandRunner> runner_;
        Options options_;
        std::thread worker_;

        mutable std::mutex mutex_;
        std::condition_variable wake_;
        bool stopping_ = false;
        std::optional<Job> job_;
        WifiLink link_;

        // Worker thread only.
        std::string ssid_uuid_, ssid_of_uuid_; // the SSID of the last active profile seen

        void Loop();
        bool Stopping() const;
        CommandOutput Nmcli(std::vector<std::string> args, std::chrono::milliseconds timeout = std::chrono::seconds(10));
        void Refresh();
        std::string SsidOf(const std::string &uuid);
        WifiResult Join(const Job &job);
    };
}
