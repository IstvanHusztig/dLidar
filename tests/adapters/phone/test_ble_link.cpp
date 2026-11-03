// BleLink (openspec phone-link-ble, design D2/D4-D7, specs/phone-ble) with a
// fake BlueZ peripheral and a fake clock, no D-Bus: splitting by the
// connection's MTU, a fresh notification per message, the MTU from writes,
// paced output and the cap, open/close and seq after a reconnect (real
// PhoneServer), one phone at a time, advertising on/off, BlueZ down/up,
// write rejections, held events delivered after a slow link drains.
#include "adapters/phone/ble_link.h"
#include "adapters/phone/protocol_codec.h"
#include "../../check.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

using namespace dlidar;
using namespace dlidar::phone;
using Json = nlohmann::json;
using namespace std::chrono_literals;

namespace
{
struct FakeClock : ServerClock
{
    TimePoint now{};
    std::vector<TimePoint> wakes;
    TimePoint Now() const override { return now; }
    void WakeAt(TimePoint t) override { wakes.push_back(t); }
};

struct FakePeripheral : BlePeripheral
{
    std::vector<std::string> notified;
    std::vector<std::string> disconnected;
    std::vector<bool> advertising;
    void Notify(const std::string &piece) override { notified.push_back(piece); }
    void Disconnect(const std::string &device) override { disconnected.push_back(device); }
    void SetAdvertising(bool on) override { advertising.push_back(on); }

    // The notified bytes joined, split into messages at "\n".
    std::vector<Json> Messages(size_t from = 0) const
    {
        std::string all;
        for (size_t i = from; i < notified.size(); i++)
            all += notified[i];
        std::vector<Json> v;
        size_t start = 0, nl;
        while ((nl = all.find('\n', start)) != std::string::npos)
        {
            v.push_back(Json::parse(all.substr(start, nl - start), nullptr, false));
            start = nl + 1;
        }
        return v;
    }
};

// Records what BleLink tells the server.
struct FakeHandler : ConnectionHandler
{
    Connection *conn = nullptr;
    int opens = 0, closes = 0, writable = 0;
    std::string bytes;
    void OnOpen(Connection &c) override
    {
        conn = &c;
        opens++;
    }
    void OnBytes(Connection &, std::string_view d) override { bytes.append(d); }
    void OnWritable(Connection &) override { writable++; }
    void OnClosed(Connection &c) override
    {
        if (conn == &c)
            conn = nullptr;
        closes++;
    }
};

// Advances the fake clock to `until`, waking `wake` at each requested time.
template <typename Wake>
void RunUntil(FakeClock &clock, ServerClock::TimePoint until, Wake wake)
{
    for (;;)
    {
        auto it = std::min_element(clock.wakes.begin(), clock.wakes.end());
        if (it == clock.wakes.end() || *it > until)
            break;
        ServerClock::TimePoint t = *it;
        clock.wakes.erase(std::remove_if(clock.wakes.begin(), clock.wakes.end(),
                                         [&](ServerClock::TimePoint w) { return w <= t; }),
                          clock.wakes.end());
        clock.now = std::max(clock.now, t);
        wake();
    }
    clock.now = std::max(clock.now, until);
}

std::string Message(size_t size, char fill = 'x')
{
    std::string m(size - 1, fill);
    m[0] = '{';
    return m + "\n";
}

void OpenPhone(BleLink &link, const std::string &dev = "A")
{
    link.BluezUp();
    link.DeviceConnected(dev);
    link.NotifyStarted();
}

void TestSplitting()
{
    printf("splitting by the connection's MTU, a fresh notification per message\n");
    FakeClock clock;
    FakePeripheral p;
    FakeHandler h;
    BleLink link(h, p, clock);
    OpenPhone(link);
    CHECK(h.opens == 1 && link.Open() && link.Mtu() == 23, "open at MTU 23");

    auto run = [&] { RunUntil(clock, clock.now + 60s, [&] { link.OnWake(); }); };
    h.conn->Send(Message(100));
    run();
    CHECK(p.notified.size() == 5 && p.notified[0].size() == 20 && p.notified[4].size() == 20,
          "MTU 23: 100 bytes in 5 pieces of 20 (%zu)", p.notified.size());

    // The first write reports MTU 247: the spec's 3000-byte example.
    CHECK(link.Write("A", "{\"v\":1,\"id\":1,\"cmd\":\"hello\"}\n", 247, 0) == WriteResult::Ok, "write");
    CHECK(link.Mtu() == 247, "MTU from the write");
    size_t from = p.notified.size();
    h.conn->Send(Message(3000));
    h.conn->Send(Message(10));
    run();
    CHECK(p.notified.size() - from == 14, "13 pieces + 1 (%zu)", p.notified.size() - from);
    size_t biggest = 0;
    for (size_t i = from; i < p.notified.size(); i++)
        biggest = std::max(biggest, p.notified[i].size());
    CHECK(biggest == 244, "pieces of at most 244 (%zu)", biggest);
    CHECK(p.notified[from][0] == '{' && p.notified[from + 13][0] == '{' && p.notified[from + 13].size() == 10,
          "each message starts a fresh notification");

    link.Write("A", "x", 517, 0);
    from = p.notified.size();
    h.conn->Send(Message(1000));
    run();
    CHECK(p.notified.size() - from == 2 && p.notified[from].size() == 514 && p.notified[from + 1].size() == 486,
          "MTU 517: 514 + 486");
    CHECK(h.bytes == "{\"v\":1,\"id\":1,\"cmd\":\"hello\"}\nx", "writes go to the server in order");
}

void TestPacing()
{
    printf("paced at the rate, Pending, OnWritable, the cap\n");
    FakeClock clock;
    FakePeripheral p;
    FakeHandler h;
    BleLink link(h, p, clock);
    OpenPhone(link);
    link.Write("A", "", 517, 0);

    for (int i = 0; i < 60; i++)
        h.conn->Send(Message(1000));
    CHECK(h.conn->Pending() == 60000, "60000 bytes pending (%zu)", h.conn->Pending());
    auto sent = [&] {
        size_t n = 0;
        for (const std::string &s : p.notified)
            n += s.size();
        return n;
    };
    auto start = clock.now;
    RunUntil(clock, start + 1s, [&] { link.OnWake(); });
    size_t one = sent();
    CHECK(one > 23000 && one < 26500, "about 24 KiB in the first second (%zu)", one);
    CHECK(h.conn->Pending() == 60000 - one, "Pending counts down");
    CHECK(h.writable == 0, "no OnWritable while queued");
    RunUntil(clock, start + 3s, [&] { link.OnWake(); });
    CHECK(sent() == 60000 && h.conn->Pending() == 0, "all sent within 3 s (%zu)", sent());
    CHECK(h.writable == 1, "OnWritable once the queue is empty (%d)", h.writable);

    // 10 s at the rate: the average stays at 24 KiB/s.
    p.notified.clear();
    start = clock.now;
    for (int s = 0; s < 10; s++)
    {
        for (int i = 0; i < 30; i++)
            if (h.conn && h.conn->Pending() < 60000)
                h.conn->Send(Message(1000));
        RunUntil(clock, start + std::chrono::seconds(s + 1), [&] { link.OnWake(); });
    }
    double rate = sent() / 10.0;
    CHECK(rate > 24576 * 0.97 && rate < 24576 * 1.03, "average %.0f B/s", rate);

    // Past the cap: closed, the phone disconnected, OnClosed on the next wake.
    Connection *c = h.conn;
    for (int i = 0; i < 70 && h.conn; i++)
        c->Send(Message(1000));
    CHECK(p.disconnected.size() == 1 && p.disconnected[0] == "A", "phone disconnected at the cap");
    CHECK(h.closes == 0 && h.conn == c, "OnClosed deferred");
    c->Send(Message(10)); // after Close: ignored
    CHECK(c->Pending() == 0, "nothing queued after the close");
    RunUntil(clock, clock.now + 1s, [&] { link.OnWake(); });
    CHECK(h.closes == 1 && !link.Open(), "closed on the next wake");
    link.DeviceDisconnected("A");
    CHECK(link.Advertising(), "advertising again");
}

void TestOnePhone()
{
    printf("one phone at a time, advertising on and off\n");
    FakeClock clock;
    FakePeripheral p;
    FakeHandler h;
    BleLink link(h, p, clock);
    link.BluezUp();
    CHECK(p.advertising == std::vector<bool>{true}, "advertising when BlueZ is up");
    link.DeviceConnected("A");
    CHECK(p.advertising.back() == false, "off while connected");
    link.NotifyStarted();
    link.Write("A", "", 247, 0);

    link.DeviceConnected("B");
    CHECK(p.disconnected == std::vector<std::string>{"B"}, "second device disconnected");
    CHECK(link.Write("B", "{}\n", 247, 0) == WriteResult::NotPermitted, "its writes rejected");
    CHECK(h.bytes.empty() && link.Phone() == "A" && link.Mtu() == 247, "the phone unaffected");
    h.conn->Send(Message(50));
    RunUntil(clock, clock.now + 1s, [&] { link.OnWake(); });
    CHECK(p.notified.size() == 1, "the phone's stream goes on");

    link.DeviceDisconnected("A");
    CHECK(!link.Open() && h.closes == 1, "closed when the phone leaves");
    CHECK(p.disconnected.back() == "B" && !link.Advertising(), "B asked again, no advertising while it stays");
    link.DeviceDisconnected("B");
    CHECK(link.Advertising() && p.advertising.back() == true, "advertising again");
    CHECK(link.Phone().empty(), "B was not promoted");

    // The next phone may be B.
    link.DeviceConnected("B");
    link.NotifyStarted(); // BlueZ reported notifying already; a new StartNotify is harmless
    CHECK(link.Phone() == "B" && link.Open() && link.Mtu() == 23, "a new phone, MTU back to 23");

    link.BluezDown();
    CHECK(!link.Open() && h.closes == 2 && !link.Advertising(), "BlueZ gone: closed");
    size_t ads = p.advertising.size();
    link.BluezUp();
    CHECK(p.advertising.size() == ads + 1 && p.advertising.back(), "BlueZ back: advertising");
}

void TestWriteRules()
{
    printf("write rejections\n");
    FakeClock clock;
    FakePeripheral p;
    FakeHandler h;
    BleLink link(h, p, clock);
    link.BluezUp();
    // A write before its Connected signal: the writer is the phone.
    CHECK(link.Write("A", "{}\n", 100, 0) == WriteResult::NotPermitted, "before notifications: rejected");
    CHECK(link.Phone() == "A" && !link.Advertising() && link.Mtu() == 100, "the writer became the phone");
    link.NotifyStarted();
    CHECK(link.Write("A", "x", 0, 4) == WriteResult::InvalidOffset, "offset rejected");
    CHECK(link.Write("A", "ab", 0, 0) == WriteResult::Ok && h.bytes == "ab" && link.Mtu() == 100,
          "accepted; mtu 0 keeps the MTU");
    link.NotifyStopped();
    CHECK(!link.Open() && link.Write("A", "c", 0, 0) == WriteResult::NotPermitted, "notifications off: rejected");
    link.NotifyStarted();
    CHECK(link.Open() && h.opens == 2, "reopened");
}

void TestIdlePhone()
{
    printf("a phone without notifications is disconnected after 30 s\n");
    FakeClock clock;
    FakePeripheral p;
    FakeHandler h;
    BleLink link(h, p, clock);
    auto run = [&](std::chrono::milliseconds d) { RunUntil(clock, clock.now + d, [&] { link.OnWake(); }); };

    // A leftover link at start: connected before BlueZ is reported up.
    link.DeviceConnected("A");
    link.BluezUp();
    CHECK(!link.Advertising(), "no advertising while it is connected");
    run(29900ms);
    CHECK(p.disconnected.empty(), "not before 30 s");
    run(200ms);
    CHECK(p.disconnected == std::vector<std::string>{"A"}, "disconnected after 30 s");
    link.DeviceDisconnected("A");
    CHECK(link.Advertising(), "advertising again");

    // A phone that subscribes in time stays.
    link.DeviceConnected("A");
    run(3s);
    link.NotifyStarted();
    run(60s);
    CHECK(p.disconnected.size() == 1 && link.Open(), "subscribed: kept");

    // Notifications off while the link stays: 10 s, then disconnected.
    link.NotifyStopped();
    run(15s);
    CHECK(p.disconnected.size() == 1, "not yet");
    link.NotifyStarted(); // back within 30 s: kept
    run(60s);
    CHECK(p.disconnected.size() == 1 && link.Open(), "resubscribed in time: kept");
    link.NotifyStopped();
    run(30100ms);
    CHECK(p.disconnected.size() == 2 && p.disconnected[1] == "A", "off for 30 s: disconnected");
}

// With the real server: seq from 1 after a reconnect, held events after a
// slow link drains.
struct FakeControl : DeviceControl
{
    DeviceStatus status;
    DeviceStatus Status() const override { return status; }
    bool StartRecording(DeviceState *) override { return false; }
    void StopRecording() override {}
};

DeviceStatus State(DeviceState s, int chunks = 0)
{
    DeviceStatus d;
    d.state = s;
    d.chunks = chunks;
    d.location = "/rec/";
    return d;
}

void TestWithServer()
{
    printf("with the protocol server: seq after a reconnect, held events\n");
    FakeControl control;
    DeviceApi api(control, nullptr, DeviceInfo{"v", "livox_mid360s", "livox_mid360s", 3});
    FakeClock clock;
    auto mailbox = std::make_shared<Mailbox>(nullptr);
    PhoneEvents events(mailbox);
    PhoneServer server(api, mailbox, clock);
    FakePeripheral p;
    BleLink link(server, p, clock);
    auto run = [&](std::chrono::milliseconds d) {
        RunUntil(clock, clock.now + d, [&] {
            server.OnWake();
            link.OnWake();
        });
    };

    OpenPhone(link);
    link.Write("A", "{\"v\":1,\"id\":1,\"cmd\":\"status.get\"}\n", 517, 0);
    events.OnState(State(DeviceState::Preparing));
    events.OnState(State(DeviceState::Recording));
    server.OnMailbox();
    run(2s);
    std::vector<Json> v = p.Messages();
    CHECK(v.size() == 3 && v[0]["id"] == 1 && v[1]["seq"] == 1 && v[2]["seq"] == 2, "response, then events 1, 2");

    // Reconnect: a new connection, seq from 1.
    link.DeviceDisconnected("A");
    link.NotifyStopped();
    OpenPhone(link);
    size_t from = p.notified.size();
    events.OnState(State(DeviceState::Finalizing, 3));
    server.OnMailbox();
    run(1s);
    v = p.Messages(from);
    CHECK(v.size() == 1 && v[0]["seq"] == 1 && v[0]["data"]["state"] == "finalizing", "seq 1 after the reconnect");

    // A slow link: 80 responses (> 8 KiB) back up; the events meanwhile are
    // held, the latest state kept, and delivered after the drain.
    link.Write("A", "", 517, 0);
    from = p.notified.size();
    std::string burst;
    for (int i = 0; i < 80; i++)
        burst += "{\"v\":1,\"id\":" + std::to_string(100 + i) + ",\"cmd\":\"status.get\"}\n";
    link.Write("A", burst, 0, 0);
    events.OnWarning({WarningKind::LensDirty, "dirty"});
    events.OnState(State(DeviceState::Idle, 3));
    server.OnMailbox();
    run(5s);
    v = p.Messages(from);
    CHECK(v.size() == 81, "80 responses + 1 event (%zu)", v.size());
    if (v.size() == 81)
    {
        CHECK(v[0]["id"] == 100 && v[79]["id"] == 179, "responses in order");
        CHECK(v[80]["data"]["state"] == "idle" && v[80]["seq"] == 2 && v[80]["dropped"] == 1,
              "the latest state after the drain, the warning counted as dropped");
    }
}
} // namespace

int main()
{
    TestSplitting();
    TestPacing();
    TestOnePhone();
    TestWriteRules();
    TestIdlePhone();
    TestWithServer();
    return FinishTests("ble_link");
}
