// The phone link end to end over the real loop and Unix socket (openspec
// phone-link-core, design D10): pipelined requests, events during a
// FakeSensor recording, a fifth client, a client that never reads (the
// recording is identical to one without clients), a disconnect mid-line,
// leftover and live socket files, the flush at shutdown, and nothing left
// behind (threads, socket file).
#include "adapters/phone/phone_link.h"
#include "adapters/phone/socket_transport.h"
#include "application/device/event_hub.h"
#include "application/device/scanner_controller.h"
#include "../../application/fake_sensor.h"
#include "../../application/memory_storage.h"
#include "../../check.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <optional>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace dlidar;
using namespace dlidar::phone;
using namespace dlidar::test;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace
{
const int64_t kS0 = 1000000000000LL;

SessionConfig Config()
{
    SessionConfig c;
    c.timing.mode = ImuTimeline::Mode::Sensor;
    c.timing.log = false;
    c.timing.host.log = false;
    return c;
}

// 240 packets, 3 chunks; `poll_ms` stretches the recording in time only.
void Script(FakeSensor &s, int poll_ms = 0)
{
    for (int i = 0; i < 120; i++)
    {
        int64_t t = i * 3000000LL;
        s.AddImu(10 + t, kS0 + t + 2000, (uint32_t)i, 9.80665f, 0.01f * i);
        s.AddPoints(20 + t, kS0 + t + 1000, (uint32_t)i, 300);
    }
    if (poll_ms)
        s.on_poll = [poll_ms](int) { std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms)); };
}

bool SameChunks(const std::vector<SensorChunk> &a, const std::vector<SensorChunk> &b)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++)
    {
        const SensorChunk &x = a[i], &y = b[i];
        if (x.chunk_index != y.chunk_index || x.points.size() != y.points.size() ||
            x.imu_data.size() != y.imu_data.size() || x.packets.size() != y.packets.size() ||
            x.timing_report != y.timing_report || x.packet_state_rows != y.packet_state_rows)
            return false;
        if (memcmp(x.points.data(), y.points.data(), x.points.size() * sizeof(PointRecord)) ||
            memcmp(x.imu_data.data(), y.imu_data.data(), x.imu_data.size() * sizeof(ImuSample)) ||
            memcmp(x.packets.data(), y.packets.data(), x.packets.size() * sizeof(PacketTiming)))
            return false;
    }
    return true;
}

template <typename Cond>
bool WaitFor(Cond cond, int ms = 5000)
{
    auto end = Clock::now() + std::chrono::milliseconds(ms);
    while (!cond() && Clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return cond();
}

int Threads()
{
    int n = 0;
    if (DIR *d = opendir("/proc/self/task"))
    {
        while (dirent *e = readdir(d))
            n += e->d_name[0] != '.';
        closedir(d);
    }
    return n;
}

struct TempDir
{
    std::string path;
    TempDir()
    {
        char t[] = "/tmp/dlidar_phone_XXXXXX";
        path = mkdtemp(t);
    }
    ~TempDir() { rmdir(path.c_str()); }
};

// A test client: blocking connect, line reader with a timeout.
struct Client
{
    int fd = -1;
    std::string buf;
    bool eof = false;

    explicit Client(const std::string &path)
    {
        fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        sockaddr_un a{};
        a.sun_family = AF_UNIX;
        strncpy(a.sun_path, path.c_str(), sizeof a.sun_path - 1);
        if (connect(fd, (sockaddr *)&a, sizeof a) != 0)
        {
            close(fd);
            fd = -1;
        }
    }
    ~Client()
    {
        if (fd >= 0)
            close(fd);
    }
    bool Send(const std::string &s)
    {
        return fd >= 0 && send(fd, s.data(), s.size(), MSG_NOSIGNAL) == (ssize_t)s.size();
    }
    std::optional<Json> Next(int ms = 5000)
    {
        auto end = Clock::now() + std::chrono::milliseconds(ms);
        for (;;)
        {
            size_t nl = buf.find('\n');
            if (nl != std::string::npos)
            {
                Json j = Json::parse(buf.substr(0, nl), nullptr, false);
                buf.erase(0, nl + 1);
                return j;
            }
            int left = (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - Clock::now()).count();
            if (fd < 0 || eof || left <= 0)
                return std::nullopt;
            pollfd p{fd, POLLIN, 0};
            if (poll(&p, 1, left) <= 0)
                continue;
            char tmp[4096];
            ssize_t n = recv(fd, tmp, sizeof tmp, 0);
            if (n <= 0)
                eof = true;
            else
                buf.append(tmp, (size_t)n);
        }
    }
    // Events up to and including the state `last`.
    std::vector<Json> EventsUntil(const std::string &last)
    {
        std::vector<Json> v;
        while (std::optional<Json> j = Next())
        {
            v.push_back(*j);
            if ((*j)["event"] == "state" && (*j)["data"]["state"] == last)
                break;
        }
        return v;
    }
};

const char *const kStatus = "{\"v\":1,\"id\":1,\"cmd\":\"status.get\"}\n";

// The device with the phone link, wired as in main.cpp.
struct Device
{
    FakeSensor sensor;
    MemoryStorage storage;
    EventHub hub;
    std::unique_ptr<PhoneLink> link;
    std::unique_ptr<ScannerController> controller;
    std::unique_ptr<DeviceApi> api;

    explicit Device(const std::string &dir)
    {
        controller = std::make_unique<ScannerController>(sensor, storage, hub, Config());
        api = std::make_unique<DeviceApi>(*controller, nullptr, DeviceInfo{"test", "unitree_l2", "unitree_l2", 3});
        std::string why;
        link = PhoneLink::Start(*api, dir, why);
        CHECK(link != nullptr, "link: %s", why.c_str());
        if (link)
            hub.AddAsync(link->Events());
    }
    ~Device()
    {
        if (link)
            link->Shutdown();
        link.reset();
        api.reset();
        controller.reset();
    }
};

void TestSocketDirectory()
{
    printf("socket directory lookup\n");
    setenv("RUNTIME_DIRECTORY", "/run/dlidar:/run/other", 1);
    setenv("XDG_RUNTIME_DIR", "/run/user/1001", 1);
    CHECK(SocketDirectory() == "/run/dlidar", "%s", SocketDirectory().c_str());
    unsetenv("RUNTIME_DIRECTORY");
    CHECK(SocketDirectory() == "/run/user/1001", "%s", SocketDirectory().c_str());
    unsetenv("XDG_RUNTIME_DIR");
    CHECK(SocketDirectory().empty(), "neither: no socket, never /tmp");

    MemoryStorage storage;
    FakeSensor sensor;
    EventHub hub;
    ScannerController c(sensor, storage, hub, Config());
    DeviceApi api(c, nullptr, DeviceInfo{});
    std::string why;
    CHECK(!PhoneLink::Start(api, "", why) && why.find("RUNTIME_DIRECTORY") != std::string::npos, "%s", why.c_str());
    CHECK(!PhoneLink::Start(api, "/nonexistent/dir", why) && why.find("not a directory") != std::string::npos, "%s",
          why.c_str());
}

void TestRequestsAndRecording()
{
    printf("pipelined requests, events during a recording\n");
    TempDir dir;
    Device d(dir.path);
    if (!d.link)
        return;
    struct stat st;
    CHECK(stat(d.link->SocketPath().c_str(), &st) == 0 && (st.st_mode & 0777) == 0600, "mode 0600");
    Client c(d.link->SocketPath());
    CHECK(c.Send("{\"v\":1,\"id\":1,\"cmd\":\"hello\"}\n{\"v\":1,\"id\":2,\"cmd\":\"info.get\"}\n{\"v\":1,\"id\":3,\"cmd\":\"status.get\"}\n"),
          "send");
    for (int id = 1; id <= 3; id++)
    {
        std::optional<Json> r = c.Next();
        CHECK(r && (*r)["id"] == id && (*r)["ok"] == true, "response %d", id);
    }

    Script(d.sensor, 2);
    CHECK(d.controller->StartRecording(), "start");
    std::vector<Json> v = c.EventsUntil("idle");
    std::vector<std::string> states;
    bool summary = false, progress = false;
    for (size_t i = 0; i < v.size(); i++)
    {
        CHECK(v[i]["seq"] == i + 1, "seq %zu", i);
        if (v[i]["event"] == "state")
            states.push_back(v[i]["data"]["state"]);
        summary = summary || v[i]["event"] == "summary";
        progress = progress || v[i]["event"] == "progress";
    }
    CHECK((states == std::vector<std::string>{"preparing", "recording", "finalizing", "idle"}), "%zu states",
          states.size());
    CHECK(progress && summary, "progress %d summary %d", progress, summary);
    CHECK(!v.empty() && v.back()["data"]["chunks"] == d.storage.chunks.size(), "final chunks %s vs %zu stored", v.empty() ? "-" : v.back()["data"].dump().c_str(), d.storage.chunks.size());
    c.Send(kStatus);
    std::optional<Json> r = c.Next();
    CHECK(r && (*r)["result"]["state"] == "idle" && !(*r)["result"]["summary"].is_null(), "status after");
}

void TestFifthClient()
{
    printf("a fifth client gets too_many_clients\n");
    TempDir dir;
    Device d(dir.path);
    if (!d.link)
        return;
    std::vector<std::unique_ptr<Client>> four;
    for (int i = 0; i < 4; i++)
        four.push_back(std::make_unique<Client>(d.link->SocketPath()));
    CHECK(WaitFor([&] { return d.link->Connections() == 4; }), "four connected");
    Client fifth(d.link->SocketPath());
    std::optional<Json> e = fifth.Next();
    CHECK(e && (*e)["id"].is_null() && (*e)["error"]["code"] == "too_many_clients", "too_many_clients");
    CHECK(!fifth.Next(1000) && fifth.eof, "then closed");
    CHECK(four[0]->Send(kStatus) && four[0]->Next(), "the others are unaffected");
    four.pop_back();
    CHECK(WaitFor([&] { return d.link->Connections() == 3; }), "a client left");
    Client again(d.link->SocketPath());
    CHECK(again.Send(kStatus) && again.Next(), "room again");
}

std::vector<SensorChunk> Reference()
{
    FakeSensor sensor;
    Script(sensor);
    MemoryStorage storage;
    EventHub hub;
    {
        ScannerController c(sensor, storage, hub, Config());
        c.StartRecording();
        c.WaitIdle();
    }
    return storage.chunks;
}

void TestStalledClient()
{
    printf("a client that never reads is disconnected; the recording is unchanged\n");
    std::vector<SensorChunk> reference = Reference();
    TempDir dir;
    Device d(dir.path);
    if (!d.link)
        return;
    Client reader(d.link->SocketPath());
    Client stalled(d.link->SocketPath());
    int small = 4096;
    setsockopt(stalled.fd, SOL_SOCKET, SO_RCVBUF, &small, sizeof small);
    fcntl(stalled.fd, F_SETFL, O_NONBLOCK);
    CHECK(WaitFor([&] { return d.link->Connections() == 2; }), "two connected");

    Script(d.sensor, 3);
    CHECK(d.controller->StartRecording(), "start");
    // Requests without reading a byte, until the server gives up on it.
    std::string burst;
    for (int i = 0; i < 50; i++)
        burst += kStatus;
    bool gone = WaitFor(
        [&] {
            send(stalled.fd, burst.data(), burst.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
            return d.link->Connections() == 1;
        },
        20000);
    CHECK(gone, "the stalled client was disconnected");

    std::vector<Json> v = reader.EventsUntil("idle");
    CHECK(!v.empty() && v.back()["data"]["state"] == "idle", "the other client got every state");
    for (size_t i = 0; i < v.size(); i++)
        CHECK(v[i]["seq"] == i + 1 && !v[i].contains("dropped"), "reader lost nothing (%zu)", i);
    d.controller->WaitIdle();
    CHECK(SameChunks(d.storage.chunks, reference), "recording identical to one without clients");
}

void TestDisconnectMidLine()
{
    printf("a client leaving mid-line\n");
    TempDir dir;
    Device d(dir.path);
    if (!d.link)
        return;
    {
        Client c(d.link->SocketPath());
        CHECK(c.Send("{\"v\":1,\"id\":1,\"cmd\":\"sta"), "partial");
        CHECK(WaitFor([&] { return d.link->Connections() == 1; }), "connected");
    }
    CHECK(WaitFor([&] { return d.link->Connections() == 0; }), "closed");
    Client c(d.link->SocketPath());
    CHECK(c.Send(kStatus) && c.Next(), "a new client works");
}

void TestSocketFiles()
{
    printf("leftover socket replaced, live socket left alone\n");
    TempDir dir;
    std::string path = dir.path + "/dlidar.sock";
    {
        // A leftover: bound once, nobody listening.
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        sockaddr_un a{};
        a.sun_family = AF_UNIX;
        strncpy(a.sun_path, path.c_str(), sizeof a.sun_path - 1);
        CHECK(bind(fd, (sockaddr *)&a, sizeof a) == 0, "leftover bound");
        close(fd);
    }
    Device d(dir.path);
    if (!d.link)
        return;
    Client c(path);
    CHECK(c.Send(kStatus) && c.Next(), "the leftover was replaced");

    {
        // Another dLidar in the same directory: no socket, a warning.
        FakeSensor sensor;
        MemoryStorage storage;
        EventHub hub;
        ScannerController other(sensor, storage, hub, Config());
        DeviceApi api(other, nullptr, DeviceInfo{});
        std::string why;
        CHECK(!PhoneLink::Start(api, dir.path, why) && why.find("another dLidar") != std::string::npos, "%s",
              why.c_str());
    }
    Client c2(path);
    CHECK(c2.Send(kStatus) && c2.Next(), "the live socket is still in place");
    d.link->Shutdown();
    struct stat st;
    CHECK(stat(path.c_str(), &st) != 0, "socket removed on close");
}

void TestShutdownFlush()
{
    printf("the final state reaches the client at shutdown\n");
    TempDir dir;
    Device d(dir.path);
    if (!d.link)
        return;
    Client c(d.link->SocketPath());
    d.sensor.endless = true;
    CHECK(d.controller->StartRecording(), "start");
    CHECK(WaitFor([&] { return d.controller->Status().state == DeviceState::Recording; }), "recording");
    // As main.cpp on SIGTERM: stop, wait, then shut the link down.
    d.controller->StopRecording();
    d.controller->WaitIdle();
    d.link->Shutdown();
    std::vector<Json> v = c.EventsUntil("idle");
    CHECK(!v.empty() && v.back()["data"]["state"] == "idle", "idle delivered before the close");
    CHECK(!c.Next(1000) && c.eof, "then closed");
}
} // namespace

int main()
{
    int threads = Threads();
    TestSocketDirectory();
    TestRequestsAndRecording();
    TestFifthClient();
    TestStalledClient();
    TestDisconnectMidLine();
    TestSocketFiles();
    TestShutdownFlush();
    CHECK(Threads() == threads, "threads left behind: %d before, %d after", threads, Threads());
    return FinishTests("phone_socket");
}
