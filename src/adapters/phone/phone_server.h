#pragma once

#include "adapters/phone/connection.h"
#include "adapters/phone/line_framer.h"
#include "adapters/phone/mailbox.h"
#include "adapters/phone/protocol_codec.h"
#include "application/ports/log_reader.h"
#include "application/remote/device_api.h"
#include "application/remote/wifi_setup.h"

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace dlidar::phone
{
    // Time and timers for PhoneServer: the loop's clock in production, a
    // fake one in tests.
    class ServerClock
    {
    public:
        using TimePoint = std::chrono::steady_clock::time_point;
        virtual ~ServerClock() = default;
        virtual TimePoint Now() const = 0;
        // Call PhoneServer::OnWake at `t` (an earlier request still stands).
        virtual void WakeAt(TimePoint t) = 0;
    };

    // The protocol v1 server for every connection of every transport (design
    // D4, D5), on the phone loop's thread only. Requests are answered inline,
    // in order; events come from the mailbox, each connection gets them with
    // its own seq, progress at most once per second. A connection whose
    // unsent bytes pass kEventBacklogBytes gets no events until it has taken
    // them: meanwhile its latest state and progress are kept, the rest is
    // counted in "dropped". Responses are never dropped (the transport closes
    // a connection past its cap instead).
    class PhoneServer : public ConnectionHandler
    {
    public:
        static const size_t kEventBacklogBytes = 8192;
        static const size_t kMaxWarnings = 16; // kept for status.get
        static constexpr std::chrono::seconds kProgressInterval{1};

        // Runs a function on the loop thread later; from any thread. It may
        // drop the function once the link shuts down.
        using Post = std::function<void(std::function<void()>)>;

        // With `wifi` (and `post`, which brings the Wi-Fi results back to
        // the loop) it also serves wifi.connect and wifi.status (openspec
        // phone-link-wifi, design D7). With `log` it serves log.get (openspec
        // phone-link-logs), read inline on the loop thread.
        PhoneServer(const DeviceApi &api, std::shared_ptr<Mailbox> mailbox, ServerClock &clock,
                    WifiSetup *wifi = nullptr, Post post = nullptr, LogReader *log = nullptr);

        void OnOpen(Connection &c) override;
        void OnBytes(Connection &c, std::string_view data) override;
        void OnWritable(Connection &c) override;
        void OnClosed(Connection &c) override;

        // The mailbox has events (the loop calls it after the mailbox's wake).
        void OnMailbox();
        // A WakeAt time was reached.
        void OnWake();

        // Nothing waits to be sent: no held event and no unsent bytes.
        bool Drained() const;

        size_t Connections() const { return clients_.size(); }
        // The latest state and progress the loop has seen (what status.get reports).
        const DeviceStatus &LastStatus() const { return last_status_; }

    private:
        struct Client
        {
            LineFramer framer{4096};
            uint64_t seq = 0;
            uint64_t dropped = 0; // goes on the next event
            std::optional<DeviceStatus> held_state;
            std::optional<DeviceStatus> pending_progress; // rate-limited or held
            std::optional<ServerClock::TimePoint> last_progress;
        };

        struct Command
        {
            std::vector<std::string> args;
            std::function<std::string(Connection &, const Request &)> run;
        };

        const DeviceApi &api_;
        std::shared_ptr<Mailbox> mailbox_;
        ServerClock &clock_;
        WifiSetup *wifi_;
        Post post_;
        LogReader *log_;
        std::map<Connection *, Client> clients_;
        // The connection that sent each running wifi.connect, by a serial
        // the result carries (null once it closed: the result is only logged).
        std::map<uint64_t, Connection *> wifi_waiting_;
        uint64_t wifi_serial_ = 0;
        std::map<std::string, Command> commands_;
        std::vector<std::string> command_names_; // in hello's order

        // What status.get reports: the latest state and progress seen here,
        // so it is never older than the events already sent.
        DeviceStatus last_status_;
        std::deque<DeviceWarning> warnings_;
        std::optional<RecordingSummary> summary_;

        void AddCommand(const std::string &name, std::vector<std::string> args,
                        std::function<std::string(Connection &, const Request &)> run);
        std::string RecordingCommand(Connection &c, const Request &r, bool start);
        std::string WifiConnect(Connection &c, const Request &r);
        std::string LogGet(const Request &r);
        void OnWifiResult(uint64_t serial, int64_t op, const std::string &ssid, const WifiResult &result);
        void HandleLine(Connection &c, std::string_view line);
        void Deliver(Connection &c, Client &cl, const Mailbox::Event &e);
        void SendEvent(Connection &c, Client &cl, const std::function<std::string(uint64_t, uint64_t)> &encode);
        void SendHeld(Connection &c, Client &cl);
        void TrySendProgress(Connection &c, Client &cl);
        bool Backlogged(const Connection &c) const { return c.Pending() > kEventBacklogBytes; }
    };
}
