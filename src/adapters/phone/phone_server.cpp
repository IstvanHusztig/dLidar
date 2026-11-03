#include "adapters/phone/phone_server.h"

#include "adapters/phone/protocol_codec.h"

#include <iostream>

namespace dlidar::phone
{
    namespace
    {
        const std::vector<std::string> kEvents{"state", "warning", "progress", "summary"};
    }

    PhoneServer::PhoneServer(const DeviceApi &api, std::shared_ptr<Mailbox> mailbox, ServerClock &clock)
        : api_(api), mailbox_(std::move(mailbox)), clock_(clock), last_status_(api.Status().status)
    {
        AddCommand("hello", {}, [this](Connection &, const Request &r) {
            return EncodeHello(r.id, command_names_, kEvents);
        });
        AddCommand("status.get", {}, [this](Connection &, const Request &r) {
            StatusView v;
            v.status = last_status_;
            v.free_bytes = api_.Status().free_bytes;
            return EncodeStatus(r.id, v, std::vector<DeviceWarning>(warnings_.begin(), warnings_.end()), summary_);
        });
        AddCommand("info.get", {}, [this](Connection &, const Request &r) { return EncodeInfo(r.id, api_.Info()); });
        AddCommand("recording.start", {}, [this](Connection &c, const Request &r) { return RecordingCommand(c, r, true); });
        AddCommand("recording.stop", {}, [this](Connection &c, const Request &r) { return RecordingCommand(c, r, false); });
    }

    // recording.start / recording.stop (design D1-D3): DeviceApi decides, the
    // answer carries the state the command saw, and one console line says
    // who did what, like the record button's "button: start".
    std::string PhoneServer::RecordingCommand(Connection &c, const Request &r, bool start)
    {
        const char *what = start ? "start" : "stop";
        CommandResult res = start ? api_.Start() : api_.Stop();
        if (res.busy)
        {
            std::cout << c.Kind() << ": " << what << " refused, the last recording is still being saved" << std::endl;
            return EncodeError(r.id, kBusy, "the last recording is still being saved; try again when it is idle");
        }
        if (res.accepted)
            std::cout << c.Kind() << ": " << what << std::endl;
        else
            std::cout << c.Kind() << ": " << what << " ignored, " << StateName(res.state_seen) << std::endl;
        return EncodeCommandResult(r.id, res.accepted, res.state_seen);
    }

    void PhoneServer::AddCommand(const std::string &name, std::vector<std::string> args,
                                 std::function<std::string(Connection &, const Request &)> run)
    {
        commands_[name] = Command{std::move(args), std::move(run)};
        command_names_.push_back(name);
    }

    void PhoneServer::OnOpen(Connection &c) { clients_[&c] = Client(); }

    void PhoneServer::OnBytes(Connection &c, std::string_view data)
    {
        auto it = clients_.find(&c);
        if (it == clients_.end())
            return;
        // The transport defers OnClosed, so the client stays valid while its
        // lines are handled, even if a Send closes the connection.
        it->second.framer.Feed(
            data, [&](std::string_view line) { HandleLine(c, line); },
            [&] { c.Send(EncodeError(std::nullopt, kTooLarge, "a request is at most 4096 bytes including its newline")); });
    }

    void PhoneServer::HandleLine(Connection &c, std::string_view line)
    {
        auto args = [this](const std::string &cmd) -> const std::vector<std::string> * {
            auto it = commands_.find(cmd);
            return it == commands_.end() ? nullptr : &it->second.args;
        };
        Request request;
        DecodeError error;
        if (!DecodeRequest(line, args, request, error))
            c.Send(EncodeError(error));
        else
            c.Send(commands_.at(request.cmd).run(c, request));
    }

    void PhoneServer::OnWritable(Connection &c)
    {
        auto it = clients_.find(&c);
        if (it != clients_.end())
            SendHeld(c, it->second);
    }

    void PhoneServer::OnClosed(Connection &c) { clients_.erase(&c); }

    void PhoneServer::OnMailbox()
    {
        uint64_t dropped = 0;
        std::deque<Mailbox::Event> events = mailbox_->Take(dropped);
        if (dropped)
            for (auto &kv : clients_)
                kv.second.dropped += dropped;
        for (const Mailbox::Event &e : events)
        {
            switch (e.kind)
            {
            case Mailbox::Event::State:
                last_status_ = e.status;
                if (e.status.state == DeviceState::Preparing)
                    warnings_.clear(); // a new recording
                break;
            case Mailbox::Event::Progress:
                last_status_ = e.status;
                break;
            case Mailbox::Event::Warning:
                warnings_.push_back(e.warning);
                if (warnings_.size() > kMaxWarnings)
                    warnings_.pop_front();
                break;
            case Mailbox::Event::Summary:
                summary_ = e.summary;
                break;
            }
            for (auto &kv : clients_)
                Deliver(*kv.first, kv.second, e);
        }
    }

    void PhoneServer::Deliver(Connection &c, Client &cl, const Mailbox::Event &e)
    {
        bool backlogged = Backlogged(c);
        switch (e.kind)
        {
        case Mailbox::Event::State:
            if (backlogged)
            {
                if (cl.held_state)
                    cl.dropped++;
                cl.held_state = e.status;
                // A state other than Recording ends that recording's progress.
                if (e.status.state != DeviceState::Recording && cl.pending_progress)
                {
                    cl.dropped++;
                    cl.pending_progress.reset();
                }
                return;
            }
            SendHeld(c, cl);
            // The recording's last progress goes before its finalizing state,
            // whatever the rate limit says.
            if (cl.pending_progress && e.status.state != DeviceState::Recording)
            {
                DeviceStatus p = *cl.pending_progress;
                cl.pending_progress.reset();
                cl.last_progress = clock_.Now();
                SendEvent(c, cl, [&](uint64_t seq, uint64_t d) { return EncodeProgressEvent(seq, d, p.chunks, p.points); });
            }
            SendEvent(c, cl, [&](uint64_t seq, uint64_t d) { return EncodeStateEvent(seq, d, e.status); });
            return;
        case Mailbox::Event::Progress:
            if (backlogged && cl.pending_progress)
                cl.dropped++;
            cl.pending_progress = e.status;
            if (!backlogged)
                SendHeld(c, cl); // a held state still goes first
            return;
        case Mailbox::Event::Warning:
        case Mailbox::Event::Summary:
            if (backlogged)
            {
                cl.dropped++;
                return;
            }
            SendHeld(c, cl);
            if (e.kind == Mailbox::Event::Warning)
                SendEvent(c, cl, [&](uint64_t seq, uint64_t d) { return EncodeWarningEvent(seq, d, e.warning); });
            else
                SendEvent(c, cl, [&](uint64_t seq, uint64_t d) { return EncodeSummaryEvent(seq, d, e.summary); });
            return;
        }
    }

    void PhoneServer::SendEvent(Connection &c, Client &cl, const std::function<std::string(uint64_t, uint64_t)> &encode)
    {
        cl.seq++;
        std::string line = encode(cl.seq, cl.dropped);
        cl.dropped = 0;
        c.Send(line);
    }

    // A held state first: a held progress is newer than it (a state other
    // than Recording drops the progress held before it).
    void PhoneServer::SendHeld(Connection &c, Client &cl)
    {
        if (Backlogged(c))
            return;
        if (cl.held_state)
        {
            DeviceStatus s = *cl.held_state;
            cl.held_state.reset();
            SendEvent(c, cl, [&](uint64_t seq, uint64_t d) { return EncodeStateEvent(seq, d, s); });
        }
        TrySendProgress(c, cl);
    }

    void PhoneServer::TrySendProgress(Connection &c, Client &cl)
    {
        if (!cl.pending_progress || Backlogged(c))
            return;
        ServerClock::TimePoint now = clock_.Now();
        if (cl.last_progress && now - *cl.last_progress < kProgressInterval)
        {
            clock_.WakeAt(*cl.last_progress + kProgressInterval);
            return;
        }
        DeviceStatus p = *cl.pending_progress;
        cl.pending_progress.reset();
        cl.last_progress = now;
        SendEvent(c, cl, [&](uint64_t seq, uint64_t d) { return EncodeProgressEvent(seq, d, p.chunks, p.points); });
    }

    void PhoneServer::OnWake()
    {
        for (auto &kv : clients_)
            SendHeld(*kv.first, kv.second);
    }

    bool PhoneServer::Drained() const
    {
        if (!mailbox_->Empty())
            return false;
        for (const auto &kv : clients_)
            if (kv.second.held_state || kv.second.pending_progress || kv.first->Pending() > 0)
                return false;
        return true;
    }
}
