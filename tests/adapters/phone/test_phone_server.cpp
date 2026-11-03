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
} // namespace

int main()
{
    TestOrderAndSeq();
    TestProgressRate();
    TestDropped();
    TestMailboxOverflow();
    TestStatusAndRequests();
    TestRecordingCommands();
    return FinishTests("phone_server");
}
