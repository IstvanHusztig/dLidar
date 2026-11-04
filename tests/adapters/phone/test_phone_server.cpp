// PhoneServer + Mailbox (openspec phone-link-core, design D4/D5, specs/phone-api)
// with a fake Connection and a fake clock, no sockets: order and seq,
// progress at most 1 Hz and flushed before finalizing, dropped with the
// latest state kept, status.get never older than the events sent,
// pipelined responses in order, warnings and summary in status.get.
#include "adapters/phone/phone_server.h"
#include "adapters/phone/protocol_codec.h"
#include "../../check.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace dlidar;
using namespace dlidar::phone;
using Json = nlohmann::json;
using namespace std::chrono_literals;

namespace
{
struct FakeControl : DeviceControl
{
    DeviceStatus status;
    int starts = 0, stops = 0;
    DeviceStatus Status() const override { return status; }
    bool StartRecording(DeviceState *seen) override
    {
        if (seen)
            *seen = status.state;
        if (status.state != DeviceState::Idle && status.state != DeviceState::Error)
            return false;
        starts++;
        status.state = DeviceState::Preparing;
        return true;
    }
    void StopRecording() override { stops++; }
};

struct FakeSpace : StorageSpace
{
    std::optional<uint64_t> FreeBytes() const override { return 1000; }
};

struct FakeClock : ServerClock
{
    TimePoint now{};
    std::vector<TimePoint> wakes;
    TimePoint Now() const override { return now; }
    void WakeAt(TimePoint t) override { wakes.push_back(t); }
};

// Keeps what was sent; `stalled` means the client does not read.
struct FakeConnection : Connection
{
    std::string out;
    size_t taken = 0; // bytes of `out` already looked at by Lines()
    bool stalled = false;
    size_t unread = 0;
    bool closed = false;
    const char *Kind() const override { return "phone"; }

    void Send(const std::string &bytes) override
    {
        out += bytes;
        if (stalled)
            unread += bytes.size();
    }
    size_t Pending() const override { return unread; }
    void Close() override { closed = true; }

    // The messages sent since the last call.
    std::vector<Json> Lines()
    {
        std::vector<Json> v;
        size_t nl;
        while ((nl = out.find('\n', taken)) != std::string::npos)
        {
            v.push_back(Json::parse(out.substr(taken, nl - taken)));
            taken = nl + 1;
        }
        return v;
    }
};

struct Rig
{
    FakeControl control;
    FakeSpace space;
    DeviceApi api{control, &space, DeviceInfo{"v1", "unitree_l2", "unitree_l2", 3}};
    FakeClock clock;
    int wakes = 0;
    std::shared_ptr<Mailbox> mailbox = std::make_shared<Mailbox>([this] { wakes++; });
    PhoneEvents events{mailbox};
    PhoneServer server{api, mailbox, clock};
};

DeviceStatus Status(DeviceState state, int chunks = 0, uint64_t points = 0)
{
    DeviceStatus s;
    s.state = state;
    s.chunks = chunks;
    s.points = points;
    s.location = "/rec/";
    return s;
}

std::string Event(const Json &j)
{
    if (!j.contains("event"))
        return "response";
    if (j["event"] == "state")
        return "state:" + j["data"]["state"].get<std::string>();
    return j["event"].get<std::string>();
}

void TestOrderAndSeq()
{
    printf("a whole recording: order and seq\n");
    Rig r;
    FakeConnection c;
    r.server.OnOpen(c);
    r.events.OnState(Status(DeviceState::Preparing));
    r.events.OnState(Status(DeviceState::Recording));
    r.events.OnProgress(Status(DeviceState::Recording, 1, 100));
    r.events.OnWarning({WarningKind::LensDirty, "lens dirty"});
    r.events.OnState(Status(DeviceState::Finalizing, 1, 100));
    RecordingSummary sum;
    sum.decision = "ok";
    r.events.OnSummary(sum);
    r.events.OnState(Status(DeviceState::Idle, 1, 100));
    CHECK(r.wakes == 1, "one wake for the batch (%d)", r.wakes);
    r.server.OnMailbox();

    std::vector<Json> v = c.Lines();
    std::vector<std::string> want{"state:preparing", "state:recording", "progress", "warning",
                                  "state:finalizing", "summary", "state:idle"};
    CHECK(v.size() == want.size(), "%zu events", v.size());
    for (size_t i = 0; i < v.size() && i < want.size(); i++)
    {
        CHECK(Event(v[i]) == want[i], "event %zu: %s", i, Event(v[i]).c_str());
        CHECK(v[i]["seq"] == i + 1 && !v[i].contains("dropped"), "seq %zu", i);
    }

    // A second batch wakes again; a new connection counts from 1.
    FakeConnection c2;
    r.server.OnOpen(c2);
    r.events.OnState(Status(DeviceState::Preparing));
    CHECK(r.wakes == 2, "second batch wakes");
    r.server.OnMailbox();
    v = c.Lines();
    CHECK(v.size() == 1 && v[0]["seq"] == 8, "first connection goes on");
    v = c2.Lines();
    CHECK(v.size() == 1 && v[0]["seq"] == 1, "a new connection starts at 1");
}

void TestProgressRate()
{
    printf("progress at most once per second, the last before finalizing\n");
    Rig r;
    FakeConnection c;
    r.server.OnOpen(c);
    r.events.OnState(Status(DeviceState::Recording));
    r.server.OnMailbox();
    c.Lines();

    // 14 chunks per second for 10 s; the loop runs every chunk and at wakes.
    int chunks = 0;
    for (int step = 0; step < 140; step++)
    {
        r.clock.now = ServerClock::TimePoint{} + std::chrono::milliseconds(step * 1000 / 14);
        chunks++;
        r.events.OnProgress(Status(DeviceState::Recording, chunks, chunks * 14000ull));
        r.server.OnMailbox();
        while (!r.clock.wakes.empty() && r.clock.wakes.front() <= r.clock.now)
        {
            r.clock.wakes.erase(r.clock.wakes.begin());
            r.server.OnWake();
        }
    }
    r.events.OnState(Status(DeviceState::Finalizing, chunks, chunks * 14000ull));
    r.server.OnMailbox();
    std::vector<Json> v = c.Lines();
    int progress = 0;
    for (size_t i = 0; i + 1 < v.size(); i++)
        progress += Event(v[i]) == "progress";
    CHECK(progress >= 9 && progress <= 11, "%d progress events in 10 s", progress);
    CHECK(v.size() >= 2 && Event(v[v.size() - 2]) == "progress" && v[v.size() - 2]["data"]["chunks"] == chunks,
          "the final count right before finalizing");
    CHECK(Event(v.back()) == "state:finalizing", "then finalizing");
    for (size_t i = 0; i < v.size(); i++)
        CHECK(v[i]["seq"] == i + 2, "seq without gaps (the recording state was 1)");

    r.server.OnWake();
    CHECK(c.Lines().empty(), "no progress after finalizing");
}

void TestDropped()
{
    printf("a client that does not read: dropped, latest state kept\n");
    Rig r;
    FakeConnection slow, fast;
    r.server.OnOpen(slow);
    r.server.OnOpen(fast);
    slow.stalled = true;
    slow.unread = PhoneServer::kEventBacklogBytes + 1;

    r.events.OnState(Status(DeviceState::Preparing));
    r.events.OnState(Status(DeviceState::Recording));
    r.server.OnMailbox();
    for (int i = 1; i <= 5; i++)
    {
        r.clock.now += 2s;
        r.events.OnProgress(Status(DeviceState::Recording, i, i * 10ull));
        r.server.OnMailbox();
    }
    r.events.OnWarning({WarningKind::StorageSlow, "slow"});
    r.events.OnState(Status(DeviceState::Finalizing, 5, 50));
    r.events.OnSummary(RecordingSummary());
    r.events.OnState(Status(DeviceState::Idle, 5, 50));
    r.server.OnMailbox();

    CHECK(slow.Lines().empty(), "nothing sent while backlogged");
    CHECK(!r.server.Drained(), "held events count as not drained");
    std::vector<Json> f = fast.Lines();
    CHECK(f.size() == 11, "the other client gets everything (%zu)", f.size());

    // The client reads again: only the latest state, with the gap counted.
    slow.stalled = false;
    slow.unread = 0;
    r.server.OnWritable(slow);
    std::vector<Json> v = slow.Lines();
    CHECK(v.size() == 1 && Event(v[0]) == "state:idle", "latest state (%zu events)", v.size());
    CHECK(v.size() == 1 && v[0]["seq"] == 1 && v[0].value("dropped", 0) == 10, "dropped %d",
          v.empty() ? -1 : v[0].value("dropped", 0));
    CHECK(r.server.Drained(), "drained");

    // Responses are never dropped while backlogged.
    slow.stalled = true;
    slow.unread = PhoneServer::kEventBacklogBytes + 1;
    r.server.OnBytes(slow, "{\"v\":1,\"id\":5,\"cmd\":\"info.get\"}\n");
    v = slow.Lines();
    CHECK(v.size() == 1 && v[0]["id"] == 5, "response sent despite the backlog");

    // Backlogged during a recording: the latest progress is kept and comes
    // after the held state.
    r.events.OnState(Status(DeviceState::Preparing));
    r.events.OnState(Status(DeviceState::Recording));
    r.events.OnProgress(Status(DeviceState::Recording, 7, 70));
    r.server.OnMailbox();
    slow.unread = 0;
    r.clock.now += 2s;
    r.server.OnWritable(slow);
    v = slow.Lines();
    CHECK(v.size() == 2 && Event(v[0]) == "state:recording" && v[0].value("dropped", 0) == 1 &&
              Event(v[1]) == "progress" && v[1]["data"]["chunks"] == 7,
          "held state, then the held progress");
}

void TestMailboxOverflow()
{
    printf("mailbox overflow: counted for every client, latest state kept\n");
    Rig r;
    FakeConnection c;
    r.server.OnOpen(c);
    r.events.OnState(Status(DeviceState::Recording));
    for (size_t i = 0; i < Mailbox::kMaxEvents + 9; i++)
        r.events.OnWarning({WarningKind::StorageSlow, "slow " + std::to_string(i)});
    r.server.OnMailbox();
    std::vector<Json> v = c.Lines();
    CHECK(v.size() == Mailbox::kMaxEvents, "%zu events", v.size());
    CHECK(!v.empty() && Event(v[0]) == "state:recording" && v[0].value("dropped", 0) == 10,
          "the state kept, 10 dropped (%d)", v.empty() ? -1 : v[0].value("dropped", 0));
}

void TestStatusAndRequests()
{
    printf("status.get from the events seen, pipelined requests in order\n");
    Rig r;
    r.control.status = Status(DeviceState::Idle);
    FakeConnection c;
    r.server.OnOpen(c);

    // The controller's idle is older than the preparing event already sent.
    r.events.OnState(Status(DeviceState::Preparing));
    r.server.OnMailbox();
    c.Lines();
    r.server.OnBytes(c, "{\"v\":1,\"id\":1,\"cmd\":\"status.get\"}\n");
    std::vector<Json> v = c.Lines();
    CHECK(v.size() == 1 && v[0]["result"]["state"] == "preparing", "never older than the events sent");
    CHECK(v.size() == 1 && v[0]["result"]["free_bytes"] == 1000, "free bytes live");

    // Pieces, several requests per write, an unreadable line between them.
    r.server.OnBytes(c, "{\"v\":1,\"id\":2,\"cmd\":\"hel");
    CHECK(c.Lines().empty(), "nothing before the newline");
    r.server.OnBytes(c, "lo\"}\n{\"v\":1,\"id\":3,\"cmd\":\"info.get\"}\nnot json\n{\"v\":1,\"id\":4,\"cmd\":\"status.get\"}\n"
                        "{\"v\":1,\"id\":5,\"cmd\":\"nope\"}\n");
    v = c.Lines();
    CHECK(v.size() == 5, "%zu responses", v.size());
    if (v.size() == 5)
    {
        CHECK(v[0]["id"] == 2 && v[0]["result"]["commands"].size() == 5, "hello");
        CHECK(v[1]["id"] == 3 && v[1]["result"]["sensor"] == "unitree_l2", "info.get");
        CHECK(v[2]["id"].is_null() && v[2]["error"]["code"] == "bad_request", "unreadable line in its place");
        CHECK(v[3]["id"] == 4 && v[3]["ok"] == true, "status.get");
        CHECK(v[4]["id"] == 5 && v[4]["error"]["code"] == "unknown_command", "unknown command");
    }
    r.server.OnBytes(c, std::string(5000, 'x') + "\n{\"v\":1,\"id\":6,\"cmd\":\"hello\"}\n");
    v = c.Lines();
    CHECK(v.size() == 2 && v[0]["error"]["code"] == "too_large" && v[0]["id"].is_null() && v[1]["id"] == 6,
          "too_large, then the next request");

    // Warnings and summary, recoverable; a new recording empties the warnings.
    r.events.OnWarning({WarningKind::LensDirty, "dirty"});
    RecordingSummary sum;
    sum.decision = "fine";
    r.events.OnSummary(sum);
    r.events.OnState(Status(DeviceState::Idle, 3, 30));
    r.server.OnMailbox();
    c.Lines();
    r.server.OnBytes(c, "{\"v\":1,\"id\":7,\"cmd\":\"status.get\"}\n");
    v = c.Lines();
    CHECK(v.size() == 1 && v[0]["result"]["warnings"].size() == 1 && v[0]["result"]["warnings"][0]["kind"] == "lens_dirty",
          "warning in status.get");
    CHECK(v.size() == 1 && v[0]["result"]["summary"]["decision"] == "fine" && v[0]["result"]["chunks"] == 3,
          "summary in status.get");
    r.events.OnState(Status(DeviceState::Preparing));
    r.server.OnMailbox();
    c.Lines();
    r.server.OnBytes(c, "{\"v\":1,\"id\":8,\"cmd\":\"status.get\"}\n");
    v = c.Lines();
    CHECK(v.size() == 1 && v[0]["result"]["warnings"].empty() && !v[0]["result"]["summary"].is_null(),
          "warnings emptied by a new recording, summary kept");

    r.server.OnClosed(c);
    CHECK(r.server.Connections() == 0, "closed");
    r.events.OnState(Status(DeviceState::Idle));
    r.server.OnMailbox(); // no client: nothing to deliver, no crash
    CHECK(r.server.Drained(), "drained without clients");
}
void TestRecordingCommands()
{
    printf("recording.start and recording.stop: answers, busy, log lines, hello\n");
    Rig r;
    FakeConnection c;
    r.server.OnOpen(c);
    std::ostringstream log;
    std::streambuf *old = std::cout.rdbuf(log.rdbuf());
    auto ask = [&](int id, const char *cmd, const char *extra = "") {
        r.server.OnBytes(c, "{\"v\":1,\"id\":" + std::to_string(id) + ",\"cmd\":\"" + cmd + "\"" + extra + "}\n");
        std::vector<Json> v = c.Lines();
        return v.empty() ? Json() : v.back();
    };

    r.control.status.state = DeviceState::Idle;
    Json j = ask(1, "recording.start");
    CHECK(j["id"] == 1 && j["ok"] == true && j["result"]["accepted"] == true && j["result"]["state"] == "idle",
          "start from idle: %s", j.dump().c_str());
    CHECK(r.control.starts == 1, "started once");
    r.control.status.state = DeviceState::Recording;
    j = ask(2, "recording.start");
    CHECK(j["result"]["accepted"] == false && j["result"]["state"] == "recording", "start while recording");
    j = ask(3, "recording.stop");
    CHECK(j["result"]["accepted"] == true && j["result"]["state"] == "recording" && r.control.stops == 1,
          "stop while recording");
    r.control.status.state = DeviceState::Finalizing;
    j = ask(4, "recording.start");
    CHECK(j["id"] == 4 && j["ok"] == false && j["error"]["code"] == "busy" &&
              j["error"]["message"].get<std::string>().find("saved") != std::string::npos,
          "busy while saving: %s", j.dump().c_str());
    j = ask(5, "recording.stop");
    CHECK(j["result"]["accepted"] == false && j["result"]["state"] == "finalizing", "stop while saving");
    r.control.status.state = DeviceState::Idle;
    j = ask(6, "recording.stop");
    CHECK(j["result"]["accepted"] == false && j["result"]["state"] == "idle", "stop while idle");
    j = ask(7, "recording.start", ",\"args\":{\"seconds\":10}");
    CHECK(j["ok"] == false && j["error"]["code"] == "bad_request", "unknown argument rejected");
    CHECK(r.control.starts == 1 && r.control.stops == 1, "nothing else called");
    j = ask(8, "hello");
    std::vector<std::string> cmds = j["result"]["commands"];
    CHECK(std::count(cmds.begin(), cmds.end(), "recording.start") == 1 &&
              std::count(cmds.begin(), cmds.end(), "recording.stop") == 1,
          "hello lists both");

    std::cout.rdbuf(old);
    std::string l = log.str();
    CHECK(l.find("phone: start\n") != std::string::npos, "log: start");
    CHECK(l.find("phone: start ignored, recording\n") != std::string::npos, "log: ignored");
    CHECK(l.find("phone: stop\n") != std::string::npos, "log: stop");
    CHECK(l.find("phone: start refused, the last recording is still being saved\n") != std::string::npos,
          "log: busy");
    CHECK(l.find("phone: stop ignored, idle\n") != std::string::npos, "log: stop ignored");
}
// A WifiManager whose joins end when the test says so.
struct FakeWifi : WifiManager
{
    WifiLink link{WifiState::Connected, "Home", "192.168.1.106"};
    std::vector<WifiNetwork> connects;
    std::function<void(const WifiResult &)> pending;
    std::optional<WifiResult> at_once; // ends inside Connect

    WifiLink Link() const override { return link; }
    void Connect(const WifiNetwork &n, std::chrono::seconds, std::function<void(const WifiResult &)> done) override
    {
        connects.push_back(n);
        if (at_once)
            done(*at_once);
        else
            pending = std::move(done);
    }
    void Finish(const WifiResult &r)
    {
        auto done = std::move(pending);
        pending = nullptr;
        done(r);
    }
};

// A server with Wi-Fi; posted functions wait in `posted` until RunPosted().
struct WifiRig : Rig
{
    FakeWifi wifi;
    WifiSetup setup{wifi};
    std::vector<std::function<void()>> posted;
    PhoneServer wserver{api, mailbox, clock, &setup, [this](std::function<void()> fn) { posted.push_back(std::move(fn)); }};

    void RunPosted()
    {
        std::vector<std::function<void()>> v;
        v.swap(posted);
        for (auto &fn : v)
            fn();
    }
};

Json Ask(PhoneServer &s, FakeConnection &c, int id, const std::string &cmd, const std::string &args = "")
{
    std::string line = "{\"v\":1,\"id\":" + std::to_string(id) + ",\"cmd\":\"" + cmd + "\"";
    if (!args.empty())
        line += ",\"args\":" + args;
    s.OnBytes(c, line + "}\n");
    std::vector<Json> v = c.Lines();
    return v.empty() ? Json() : v.back();
}

std::string WifiArgs(const std::string &ssid, const std::string &security, const std::string &psk)
{
    Json a;
    a["ssid"] = ssid;
    a["security"] = security;
    if (!psk.empty())
        a["psk"] = psk;
    return a.dump();
}

void TestWifiCommands()
{
    printf("wifi.connect and wifi.status: answers, busy, result to the requester only, log lines\n");
    {
        Rig plain;
        FakeConnection c;
        plain.server.OnOpen(c);
        std::vector<std::string> cmds = Ask(plain.server, c, 1, "hello")["result"]["commands"];
        CHECK(std::count(cmds.begin(), cmds.end(), "wifi.connect") == 0, "no Wi-Fi commands without a WifiSetup");
        CHECK(Ask(plain.server, c, 2, "wifi.status")["error"]["code"] == "unknown_command", "wifi.status unknown");
    }

    WifiRig r;
    FakeConnection c1, c2;
    r.wserver.OnOpen(c1);
    r.wserver.OnOpen(c2);
    std::ostringstream log;
    std::streambuf *old = std::cout.rdbuf(log.rdbuf());

    std::vector<std::string> cmds = Ask(r.wserver, c1, 1, "hello")["result"]["commands"];
    CHECK(std::count(cmds.begin(), cmds.end(), "wifi.connect") == 1 && std::count(cmds.begin(), cmds.end(), "wifi.status") == 1,
          "hello lists both");

    Json j = Ask(r.wserver, c1, 2, "wifi.status");
    CHECK(j["result"]["state"] == "connected" && j["result"]["ssid"] == "Home" && j["result"]["last"].is_null(),
          "status before: %s", j.dump().c_str());

    j = Ask(r.wserver, c1, 12, "wifi.connect", WifiArgs("Site", "wpa2-psk", "secret123"));
    CHECK(j["id"] == 12 && j["ok"] == true && j["result"]["accepted"] == true && j["result"]["timeout_s"] == 60,
          "accepted: %s", j.dump().c_str());
    CHECK(r.wifi.connects.size() == 1 && r.wifi.connects[0].ssid == "Site" &&
              r.wifi.connects[0].security == WifiSecurity::Wpa2Psk && r.wifi.connects[0].psk == "secret123",
          "passed on");
    j = Ask(r.wserver, c2, 3, "wifi.status");
    CHECK(j["result"]["state"] == "connecting" && j["result"]["ssid"] == "Site" && j["result"]["last"]["result"] == "pending",
          "status while joining: %s", j.dump().c_str());

    j = Ask(r.wserver, c2, 4, "wifi.connect", WifiArgs("Other", "open", ""));
    CHECK(j["id"] == 4 && j["ok"] == false && j["error"]["code"] == "busy", "busy: %s", j.dump().c_str());
    CHECK(r.wifi.connects.size() == 1, "the second join not started");

    // The result, while c1 is behind on its events: it goes out anyway.
    c1.stalled = true;
    c1.unread = PhoneServer::kEventBacklogBytes + 1;
    r.wifi.Finish(WifiResult{WifiOutcome::Connected, "192.168.1.50", ""});
    CHECK(c1.Lines().empty(), "nothing before the loop runs it");
    r.RunPosted();
    std::vector<Json> got = c1.Lines();
    CHECK(got.size() == 1 && got[0]["event"] == "result" && got[0]["op"] == 12 && got[0]["ok"] == true &&
              got[0]["result"]["ssid"] == "Site" && got[0]["result"]["ip"] == "192.168.1.50" && got[0]["seq"] == 1,
          "result to the requester: %s", got.empty() ? "none" : got[0].dump().c_str());
    CHECK(c2.Lines().empty(), "nothing to the other client");
    c1.stalled = false;
    c1.unread = 0;

    // Seq goes on for the requester only.
    r.events.OnState(Status(DeviceState::Preparing));
    r.wserver.OnMailbox();
    got = c1.Lines();
    CHECK(got.size() == 1 && got[0]["seq"] == 2, "c1's next event is seq 2");
    got = c2.Lines();
    CHECK(got.size() == 1 && got[0]["seq"] == 1, "c2's first event is seq 1");

    j = Ask(r.wserver, c2, 5, "wifi.status");
    CHECK(j["result"]["last"]["ssid"] == "Site" && j["result"]["last"]["result"] == "connected", "last connected");

    // A requester that closes before the result: only logged.
    j = Ask(r.wserver, c2, 6, "wifi.connect", WifiArgs("Cafe", "wpa3-sae", "jelszo-123"));
    CHECK(j["result"]["accepted"] == true, "accepted from c2");
    r.wserver.OnClosed(c2);
    r.wifi.Finish(WifiResult{WifiOutcome::WrongPassword, "", ""});
    r.RunPosted();
    CHECK(c1.Lines().empty(), "the other client gets nothing");
    j = Ask(r.wserver, c1, 7, "wifi.status");
    CHECK(j["result"]["last"]["ssid"] == "Cafe" && j["result"]["last"]["result"] == "wrong_password",
          "the lost result in status: %s", j.dump().c_str());

    // A join that ends inside Connect: the answer still comes first.
    r.wifi.at_once = WifiResult{WifiOutcome::Failed, "", "NetworkManager is not running"};
    r.wserver.OnBytes(c1, "{\"v\":1,\"id\":8,\"cmd\":\"wifi.connect\",\"args\":" + WifiArgs("Home", "open", "") + "}\n");
    r.RunPosted();
    got = c1.Lines();
    CHECK(got.size() == 2 && got[0]["id"] == 8 && got[1]["event"] == "result" && got[1]["op"] == 8 &&
              got[1]["error"]["code"] == "failed" && got[1]["error"]["message"] == "NetworkManager is not running",
          "answer, then result");

    std::cout.rdbuf(old);
    std::string l = log.str();
    CHECK(l.find("phone: wifi.connect Site (wpa2-psk)\n") != std::string::npos, "log: request");
    CHECK(l.find("phone: wifi.connect Other refused, another Wi-Fi join is running\n") != std::string::npos,
          "log: busy");
    CHECK(l.find("Wi-Fi: connected to Site, 192.168.1.50\n") != std::string::npos, "log: connected");
    CHECK(l.find("Wi-Fi: Cafe: wrong password\n") != std::string::npos, "log: wrong password");
    CHECK(l.find("Wi-Fi: Home: failed: NetworkManager is not running\n") != std::string::npos, "log: failed");
    CHECK(l.find("secret123") == std::string::npos && l.find("jelszo-123") == std::string::npos, "no password logged");
}

void TestWifiBadArguments()
{
    printf("wifi.connect arguments: bad_request names the argument, never the password\n");
    WifiRig r;
    FakeConnection c;
    r.wserver.OnOpen(c);
    std::ostringstream log;
    std::streambuf *old = std::cout.rdbuf(log.rdbuf());
    const std::string marker = "MARKER-pw-7f3a";

    struct Case
    {
        const char *what;
        std::string args;
        const char *names;
    } cases[] = {
        {"short psk", WifiArgs("Home", "wpa2-psk", "short7!"), "\"psk\""},
        {"psk with open", WifiArgs("Home", "open", marker), "\"psk\""},
        {"psk missing", WifiArgs("Home", "wpa3-sae", ""), "\"psk\""},
        {"unknown security", WifiArgs("Home", "wep", marker), "\"security\""},
        {"security missing", "{\"ssid\":\"Home\",\"psk\":\"" + marker + "\"}", "\"security\""},
        {"ssid missing", "{\"security\":\"wpa2-psk\",\"psk\":\"" + marker + "\"}", "\"ssid\""},
        {"ssid too long", WifiArgs(std::string(33, 's'), "wpa2-psk", marker), "\"ssid\""},
        {"psk not a string", "{\"ssid\":\"Home\",\"security\":\"wpa2-psk\",\"psk\":123456789}", "\"psk\""},
        {"psk with a control character", WifiArgs("Home", "wpa2-psk", marker + "\t"), "\"psk\""},
        {"unknown argument", "{\"ssid\":\"Home\",\"security\":\"wpa2-psk\",\"psk\":\"" + marker + "\",\"password\":1}",
         "\"password\""},
    };
    int id = 1;
    for (const Case &k : cases)
    {
        Json j = Ask(r.wserver, c, id++, "wifi.connect", k.args);
        CHECK(j["ok"] == false && j["error"]["code"] == "bad_request", "%s: %s", k.what, j.dump().c_str());
        CHECK(j["error"]["message"].get<std::string>().find(k.names) != std::string::npos, "%s: names %s: %s", k.what,
              k.names, j.dump().c_str());
    }
    CHECK(r.wifi.connects.empty(), "no join started");

    // The marker through every outcome of a valid request too.
    const WifiOutcome outcomes[] = {WifiOutcome::Connected, WifiOutcome::WrongPassword, WifiOutcome::NotFound,
                                    WifiOutcome::Timeout, WifiOutcome::Failed};
    for (WifiOutcome o : outcomes)
    {
        Json j = Ask(r.wserver, c, id++, "wifi.connect", WifiArgs("Home", "wpa2-psk", marker));
        CHECK(j["result"]["accepted"] == true, "accepted");
        Ask(r.wserver, c, id++, "wifi.connect", WifiArgs("Home", "wpa2-psk", marker)); // busy
        r.wifi.Finish(WifiResult{o, "192.168.1.50", ""});
        r.RunPosted();
        Ask(r.wserver, c, id++, "wifi.status");
    }
    std::cout.rdbuf(old);
    CHECK(c.out.find(marker) == std::string::npos, "the password in nothing sent");
    CHECK(log.str().find(marker) == std::string::npos, "the password in no log line");
}

struct FakeLog : LogReader
{
    std::vector<LogQuery> queries;
    enum
    {
        Page,
        BadCursor,
        Unavailable,
        Other
    } mode = Page;
    LogPage Read(const LogQuery &q) override
    {
        queries.push_back(q);
        if (mode == BadCursor)
            throw LogCursorError("\"before\" is not a log cursor");
        if (mode == Unavailable)
            throw LogUnavailable("cannot open the journal: Permission denied");
        if (mode == Other)
            throw std::runtime_error("something else");
        LogPage p;
        p.entries.push_back(LogEntry{1762246526123, "36276c97", LogLevel::Warning, LogSource::Dlidar, "WARNING: x", "c1"});
        p.more = "c1";
        return p;
    }
};

void TestLogGet()
{
    printf("log.get: the query passed through, the page, bad cursor, unavailable, hello\n");
    {
        Rig plain;
        FakeConnection c;
        plain.server.OnOpen(c);
        std::vector<std::string> cmds = Ask(plain.server, c, 1, "hello")["result"]["commands"];
        CHECK(std::count(cmds.begin(), cmds.end(), "log.get") == 0, "no log.get without a LogReader");
        CHECK(Ask(plain.server, c, 2, "log.get")["error"]["code"] == "unknown_command", "log.get unknown");
    }
    Rig r;
    FakeLog log;
    PhoneServer server{r.api, r.mailbox, r.clock, nullptr, nullptr, &log};
    FakeConnection c;
    server.OnOpen(c);
    std::ostringstream out;
    std::streambuf *old = std::cout.rdbuf(out.rdbuf());

    std::vector<std::string> cmds = Ask(server, c, 1, "hello")["result"]["commands"];
    CHECK(std::count(cmds.begin(), cmds.end(), "log.get") == 1, "hello lists log.get");

    Json j = Ask(server, c, 2, "log.get");
    CHECK(j["ok"] == true && j["result"]["entries"].size() == 1 && j["result"]["more"] == "c1" &&
              j["result"]["entries"][0]["level"] == "warning",
          "page: %s", j.dump().c_str());
    CHECK(log.queries.size() == 1 && !log.queries[0].before && log.queries[0].limit == 50 &&
              log.queries[0].min_level == LogLevel::Info,
          "defaults passed");
    Ask(server, c, 3, "log.get", R"({"before":"c1","limit":7,"level":"error"})");
    CHECK(log.queries.size() == 2 && log.queries[1].before == std::string("c1") && log.queries[1].limit == 7 &&
              log.queries[1].min_level == LogLevel::Error,
          "arguments passed");

    j = Ask(server, c, 4, "log.get", R"({"limit":0})");
    CHECK(j["error"]["code"] == "bad_request" && j["error"]["message"].get<std::string>().find("\"limit\"") != std::string::npos &&
              log.queries.size() == 2,
          "bad limit, nothing read: %s", j.dump().c_str());
    j = Ask(server, c, 5, "log.get", R"({"since":1})");
    CHECK(j["error"]["code"] == "bad_request", "unknown argument");

    log.mode = FakeLog::BadCursor;
    j = Ask(server, c, 6, "log.get", R"({"before":"garbage"})");
    CHECK(j["id"] == 6 && j["error"]["code"] == "bad_request" &&
              j["error"]["message"].get<std::string>().find("\"before\"") != std::string::npos,
          "bad cursor: %s", j.dump().c_str());
    log.mode = FakeLog::Unavailable;
    j = Ask(server, c, 7, "log.get");
    CHECK(j["error"]["code"] == "unavailable" && j["error"]["message"] == "cannot open the journal: Permission denied",
          "unavailable: %s", j.dump().c_str());
    log.mode = FakeLog::Other;
    j = Ask(server, c, 8, "log.get");
    CHECK(j["error"]["code"] == "unavailable", "any other failure: %s", j.dump().c_str());
    CHECK(Ask(server, c, 9, "status.get")["ok"] == true, "status.get still answers");
    std::cout.rdbuf(old);
    CHECK(out.str().empty(), "log.get prints nothing: %s", out.str().c_str());
}
} // namespace

int main()
{
    TestLogGet();
    TestWifiCommands();
    TestWifiBadArguments();
    TestOrderAndSeq();
    TestProgressRate();
    TestDropped();
    TestMailboxOverflow();
    TestStatusAndRequests();
    TestRecordingCommands();
    return FinishTests("phone_server");
}
