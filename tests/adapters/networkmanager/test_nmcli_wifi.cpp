// The NetworkManager adapter (openspec phone-link-wifi, design D1, D3-D6,
// D9): the pure helpers against nmcli 1.42's texts and the dev board's
// profiles, every path of a join through a scripted CommandRunner (the
// argument vectors checked against D4, the password only in the add), and
// the real ProcessRunner with harmless programs.
#include "adapters/networkmanager/command_runner.h"
#include "adapters/networkmanager/nmcli_helpers.h"
#include "adapters/networkmanager/nmcli_wifi.h"
#include "../../check.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <fcntl.h>
#include <future>
#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <utility>

using namespace dlidar;
using namespace dlidar::network;
using namespace std::chrono_literals;

namespace
{
const std::string kMarker = "MARKER-pw-7f3a";
// The dev board's profiles (2026-11-03), plus an access point and an older
// profile named like the network the tests join.
const std::string A = "4f8d20ea-4d13-4fdf-b109-2975e97efcda"; // TP-Link_B8CB, active
const std::string S = "ef6887ff-eb36-4035-ae7b-c8d7698d2b52"; // LidarStation
const std::string D = "7d89fbcc-083a-492e-9df3-a75af2b4fe2d"; // preconfigured: dlink-4570
const std::string L = "44271f40-8bca-33ae-bda6-e24d821c9110"; // lidar (Ethernet)
const std::string O = "3d6a3a84-9128-480a-af60-cbcefafcf651"; // lo
const std::string H = "11111111-2222-3333-4444-555555555555"; // Home, saved before
const std::string P = "99999999-8888-7777-6666-555555555555"; // Home, access point
const std::string NEW = "abcdef01-2345-6789-abcd-ef0123456789";

CommandOutput Ok(const std::string &out = "") { return CommandOutput{0, out, "", false, false}; }
CommandOutput Fail(int code, const std::string &err) { return CommandOutput{code, "", err, false, false}; }

bool Has(const std::vector<std::string> &v, const std::string &s) { return std::find(v.begin(), v.end(), s) != v.end(); }

// Answers like the dev board's nmcli; `up` decides the join.
struct ScriptedRunner : CommandRunner
{
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<std::vector<std::string>> calls;
    CommandOutput up = Ok("Connection successfully activated");
    CommandOutput add = Ok("Connection 'Home' (" + NEW + ") successfully added.");
    CommandOutput restore = Ok();
    CommandOutput modify = Ok();
    std::optional<CommandOutput> first; // the answer to the first step of a join
    bool active = true;                 // wlan0 is on TP-Link_B8CB
    bool block_up = false;              // the join's up waits until Cancel()
    // The CommandRunner contract: Cancel() kills the waiting Run; with none
    // waiting, the next Run returns `cancelled` without running.
    bool waiting = false, cancelled = false, cancel_pending = false;
    std::vector<std::vector<std::string>> skipped; // Runs cancelled before they started
    std::function<void()> during_add;              // called once inside the add, unlocked

    CommandOutput Run(const std::vector<std::string> &argv, std::chrono::milliseconds) override
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (cancel_pending)
        {
            cancel_pending = false;
            skipped.push_back(argv);
            CommandOutput o;
            o.cancelled = true;
            return o;
        }
        calls.push_back(argv);
        auto is = [&](std::initializer_list<const char *> want) {
            std::vector<std::string> w(want.begin(), want.end());
            return argv.size() >= w.size() && std::equal(w.begin(), w.end(), argv.begin());
        };
        if (is({"nmcli", "-t", "-f", "GENERAL.STATE,GENERAL.CON-UUID,IP4.ADDRESS", "device", "show", "wlan0"}))
            return active ? Ok("GENERAL.STATE:100 (connected)\nGENERAL.CON-UUID:" + A + "\nIP4.ADDRESS[1]:192.168.1.106/24\n")
                          : Ok("GENERAL.STATE:30 (disconnected)\nGENERAL.CON-UUID:\n");
        if (is({"nmcli", "-g", "GENERAL.CON-UUID", "device", "show", "wlan0"}))
            return first ? *first : Ok(active ? A + "\n" : "\n");
        if (is({"nmcli", "-t", "-f", "UUID,TYPE", "connection", "show"}))
            return Ok(A + ":802-11-wireless\n" + O + ":loopback\n" + S + ":802-11-wireless\n" + L +
                      ":802-3-ethernet\n" + D + ":802-11-wireless\n" + H + ":802-11-wireless\n" + P +
                      ":802-11-wireless\n");
        if (is({"nmcli", "-g", "802-11-wireless.ssid,802-11-wireless.mode", "connection", "show", "uuid"}))
        {
            const std::string &u = argv.back();
            if (u == A) return Ok("TP-Link_B8CB\ninfrastructure\n");
            if (u == S) return Ok("LidarStation\ninfrastructure\n");
            if (u == D) return Ok("dlink-4570\ninfrastructure\n");
            if (u == H) return Ok("Home\ninfrastructure\n");
            if (u == P) return Ok("Home\nap\n");
            return Fail(10, "Error: unknown connection");
        }
        if (is({"nmcli", "-g", "802-11-wireless.ssid", "connection", "show", "uuid"}))
            return argv.back() == A ? Ok("TP-Link_B8CB\n") : Ok("Home\n");
        if (is({"nmcli", "connection", "add"}))
        {
            if (std::function<void()> fn = std::exchange(during_add, nullptr))
            {
                lock.unlock();
                fn();
                lock.lock();
            }
            return add;
        }
        if (is({"nmcli", "--wait"}) && Has(argv, NEW))
        {
            if (!block_up)
                return up;
            waiting = true;
            cv.wait_for(lock, 10s, [&] { return cancelled; }); // bounded: a broken Stop fails the test, not hangs it
            waiting = false;
            cancelled = false;
            CommandOutput o = Fail(-1, "");
            o.cancelled = true;
            return o;
        }
        if (is({"nmcli", "--wait"}))
            return restore;
        if (is({"nmcli", "-g", "IP4.ADDRESS", "device", "show", "wlan0"}))
            return Ok("192.168.1.50/24\n");
        if (is({"nmcli", "connection", "modify"}))
            return modify;
        if (is({"nmcli", "connection", "delete"}))
            return Ok("Connection successfully deleted.");
        return Fail(2, "Error: unexpected call");
    }
    void Cancel() override
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (waiting)
            cancelled = true;
        else
            cancel_pending = true;
        cv.notify_all();
    }
    bool CancelPending()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return cancel_pending;
    }
    std::vector<std::vector<std::string>> Calls()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return calls;
    }
};

struct Rig
{
    ScriptedRunner *runner = new ScriptedRunner;
    std::unique_ptr<NmcliWifi> wifi;
    Rig()
    {
        NmcliWifi::Options o;
        o.refresh = std::chrono::hours(1); // only the refreshes a join causes
        o.new_uuid = [] { return NEW; };
        wifi = std::make_unique<NmcliWifi>(std::unique_ptr<CommandRunner>(runner), o);
    }
    // Starts, joins and waits for the outcome.
    WifiResult Join(const WifiNetwork &n, std::chrono::seconds limit = 60s)
    {
        wifi->Start();
        std::promise<WifiResult> p;
        wifi->Connect(n, limit, [&](const WifiResult &r) { p.set_value(r); });
        std::future<WifiResult> f = p.get_future();
        CHECK(f.wait_for(5s) == std::future_status::ready, "done within 5 s");
        return f.valid() && f.wait_for(0s) == std::future_status::ready ? f.get() : WifiResult{};
    }
};

// D4 on every call: nothing but the attempt and the older Home profile is
// ever changed; the password only in the add.
void CheckCalls(const std::vector<std::vector<std::string>> &calls, const char *what, bool may_delete_h)
{
    for (const auto &c : calls)
    {
        bool add = c.size() > 2 && c[1] == "connection" && c[2] == "add";
        bool changes = c.size() > 2 && c[1] == "connection" && (c[2] == "modify" || c[2] == "delete" || c[2] == "down");
        for (const std::string &u : {A, S, D, L, O, P})
            CHECK(!(changes && Has(c, u)), "%s: changes a profile it must not touch: %s", what, u.c_str());
        CHECK(!(changes && Has(c, H)) || (may_delete_h && c[2] == "delete"), "%s: changes the older Home profile", what);
        bool has_marker = false;
        for (const std::string &a : c)
            has_marker = has_marker || a.find(kMarker) != std::string::npos;
        CHECK(!has_marker || add, "%s: the password outside the add", what);
        CHECK(c.empty() || c[0] == "nmcli", "%s: only nmcli is run", what);
        CHECK(!(c.size() > 3 && c[1] == "device" && c[2] == "wifi"), "%s: a device wifi command (scans)", what);
    }
}

std::vector<std::string> Find(const std::vector<std::vector<std::string>> &calls, std::initializer_list<const char *> head)
{
    std::vector<std::string> w(head.begin(), head.end());
    for (const auto &c : calls)
        if (c.size() >= w.size() && std::equal(w.begin(), w.end(), c.begin()))
            return c;
    return {};
}

void TestHelpers()
{
    printf("helpers: add arguments, profiles to replace, outcomes, parsing\n");
    std::vector<std::string> a = AddArgs(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker}, NEW, "wlan0");
    CHECK(Has(a, "wpa-psk") && Has(a, kMarker) && Has(a, NEW) && a.back() == "no" && a[a.size() - 2] == "save",
          "wpa2: in memory");
    auto at = std::find(a.begin(), a.end(), "connection.autoconnect");
    CHECK(at != a.end() && at + 1 != a.end() && *(at + 1) == "no", "not autoconnecting until saved");
    a = AddArgs(WifiNetwork{"Home", WifiSecurity::Wpa3Sae, kMarker}, NEW, "wlan0");
    CHECK(Has(a, "sae") && !Has(a, "wpa-psk"), "wpa3: sae");
    a = AddArgs(WifiNetwork{"Cafe", WifiSecurity::Open, ""}, NEW, "wlan0");
    CHECK(!Has(a, "wifi-sec.key-mgmt") && !Has(a, "wifi-sec.psk"), "open: no security");

    std::vector<Profile> profiles{{A, "802-11-wireless", "TP-Link_B8CB", "infrastructure"},
                                  {S, "802-11-wireless", "LidarStation", "infrastructure"},
                                  {D, "802-11-wireless", "dlink-4570", "infrastructure"},
                                  {L, "802-3-ethernet", "", ""},
                                  {O, "loopback", "", ""},
                                  {H, "802-11-wireless", "Home", ""},
                                  {P, "802-11-wireless", "Home", "ap"},
                                  {NEW, "802-11-wireless", "Home", "infrastructure"}};
    std::vector<std::string> r = ProfilesToReplace(profiles, "Home", NEW);
    CHECK(r.size() == 1 && r[0] == H, "only the older Home profile");
    CHECK(ProfilesToReplace(profiles, "home", NEW).empty(), "exact SSID only");
    CHECK(ProfilesToReplace(profiles, "TP-Link_B8CB", NEW) == std::vector<std::string>{A}, "the active one when re-joined");
    CHECK(ProfilesToReplace(profiles, "", NEW).empty(), "never an Ethernet profile");

    CHECK(OutcomeFor(Ok()).outcome == WifiOutcome::Connected, "exit 0");
    CHECK(OutcomeFor(Fail(3, "Error: Timeout expired (44 seconds)")).outcome == WifiOutcome::Timeout, "exit 3");
    CHECK(OutcomeFor(Fail(4, "Error: Connection activation failed: Secrets were required, but not provided\nHint: use 'journalctl -xe NM_CONNECTION=x + NM_DEVICE=wlan0' to get more details.\n"))
                  .outcome == WifiOutcome::WrongPassword,
          "no secrets");
    CHECK(OutcomeFor(Fail(4, "Error: Connection activation failed: 802.1X supplicant disconnected\n")).outcome ==
              WifiOutcome::WrongPassword,
          "supplicant disconnected");
    CHECK(OutcomeFor(Fail(4, "Error: Connection activation failed: The Wi-Fi network could not be found\n")).outcome ==
              WifiOutcome::NotFound,
          "not found");
    WifiResult f = OutcomeFor(Fail(4, "Error: Connection activation failed: IP configuration could not be reserved (no available address, timeout, etc.).\n"));
    CHECK(f.outcome == WifiOutcome::Failed &&
              f.message == "Connection activation failed: IP configuration could not be reserved (no available address, timeout, etc.).",
          "other: %s", f.message.c_str());
    f = FailureFor(Fail(4, "Error: Failed to add 'Home' connection: Insufficient privileges\n"), "wlan0");
    CHECK(f.message.find("systemd/40-dlidar-network.rules") != std::string::npos, "polkit hint: %s", f.message.c_str());
    f = FailureFor(Fail(4, "Error: Connection activation failed: Not authorized to control networking.\n"), "wlan0");
    CHECK(f.message.find("40-dlidar-network.rules") != std::string::npos, "not authorized: %s", f.message.c_str());
    CHECK(FailureFor(Fail(8, "Error: NetworkManager is not running.\n"), "wlan0").message == "NetworkManager is not running",
          "exit 8");
    CHECK(FailureFor(Fail(1, "Error: Could not create NMClient object: Could not connect: No such file or directory.\n"),
                     "wlan0")
                  .message == "NetworkManager is not running",
          "no bus");
    CHECK(FailureFor(Fail(10, "Error: Device 'wlan0' not found.\n"), "wlan0").message == "no Wi-Fi device wlan0",
          "no device");
    CHECK(FailureFor(Fail(127, "cannot start nmcli: No such file or directory"), "wlan0").message.find("nmcli cannot be run") == 0,
          "no nmcli");

    std::string uuid;
    WifiLink l = ParseLink(Ok("GENERAL.STATE:100 (connected)\nGENERAL.CON-UUID:" + A +
                              "\nIP4.ADDRESS[1]:192.168.1.106/24\nIP4.ADDRESS[2]:10.0.0.5/8\n"),
                           uuid);
    CHECK(l.state == WifiState::Connected && l.ip == "192.168.1.106" && uuid == A, "connected");
    l = ParseLink(Ok("GENERAL.STATE:50 (connecting (configuring))\nGENERAL.CON-UUID:" + A + "\n"), uuid);
    CHECK(l.state == WifiState::Connecting && l.ip.empty() && uuid == A, "connecting");
    l = ParseLink(Ok("GENERAL.STATE:30 (disconnected)\nGENERAL.CON-UUID:\n"), uuid);
    CHECK(l.state == WifiState::Disconnected && uuid.empty(), "disconnected");
    l = ParseLink(Ok("GENERAL.STATE:20 (unavailable)\nGENERAL.CON-UUID:\n"), uuid);
    CHECK(l.state == WifiState::Unavailable, "radio off");
    l = ParseLink(Fail(10, "Error: Device 'wlan0' not found."), uuid);
    CHECK(l.state == WifiState::Unavailable && uuid.empty(), "no device");

    std::vector<Profile> p = ParseProfiles(A + ":802-11-wireless\n" + L + ":802-3-ethernet\n");
    CHECK(p.size() == 2 && p[0].uuid == A && p[0].type == "802-11-wireless" && p[1].type == "802-3-ethernet", "profiles");
    CHECK(UnescapeTerse("a\\:b\\\\c") == "a:b\\c", "unescape");
    CHECK(AddressOnly("192.168.1.106/24") == "192.168.1.106" && AddressOnly("") == "", "address");
    CHECK(Scrub("bad psk " + kMarker + " and " + kMarker, kMarker) == "bad psk *** and ***" && Scrub("x", "") == "x",
          "scrub");
}

void TestJoined()
{
    printf("joined: saved, the older profile of that name replaced, nothing else touched\n");
    Rig r;
    WifiResult res = r.Join(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker});
    CHECK(res.outcome == WifiOutcome::Connected && res.ip == "192.168.1.50", "connected: %s", res.message.c_str());
    auto calls = r.runner->Calls();
    CheckCalls(calls, "joined", true);
    std::vector<std::string> up = Find(calls, {"nmcli", "--wait"});
    CHECK(up.size() == 9 && up[3] == "connection" && up[4] == "up" && up[6] == NEW && up[8] == "wlan0", "up the attempt");
    int wait_s = up.size() > 2 ? atoi(up[2].c_str()) : 0;
    CHECK(wait_s >= 40 && wait_s <= 45, "waits %d s of the 60 s limit", wait_s);
    std::vector<std::string> save = Find(calls, {"nmcli", "connection", "modify"});
    CHECK(save.size() == 7 && save[4] == NEW && save[5] == "connection.autoconnect" && save[6] == "yes", "saved");
    std::vector<std::string> del = Find(calls, {"nmcli", "connection", "delete"});
    CHECK(del.size() == 5 && del[4] == H, "the older Home replaced");
    int deletes = 0;
    for (const auto &c : calls)
        deletes += c.size() > 2 && c[2] == "delete";
    CHECK(deletes == 1, "one delete");
    CHECK(r.wifi->Link().state == WifiState::Connected && r.wifi->Link().ssid == "TP-Link_B8CB", "link refreshed");
}

void TestNotJoined()
{
    printf("not joined: the attempt deleted, the previous network rejoined\n");
    struct Case
    {
        const char *what;
        CommandOutput up;
        WifiOutcome outcome;
    } cases[] = {
        {"wrong password", Fail(4, "Error: Connection activation failed: Secrets were required, but not provided\n"),
         WifiOutcome::WrongPassword},
        {"not found", Fail(4, "Error: Connection activation failed: The Wi-Fi network could not be found\n"),
         WifiOutcome::NotFound},
        {"timeout", Fail(3, "Error: Timeout expired (44 seconds)\n"), WifiOutcome::Timeout},
        {"other", Fail(4, "Error: Connection activation failed: (5) IP configuration could not be reserved\n"),
         WifiOutcome::Failed},
    };
    for (const Case &k : cases)
    {
        Rig r;
        r.runner->up = k.up;
        WifiResult res = r.Join(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker});
        CHECK(res.outcome == k.outcome, "%s: outcome %d", k.what, (int)res.outcome);
        auto calls = r.runner->Calls();
        CheckCalls(calls, k.what, false);
        std::vector<std::string> del = Find(calls, {"nmcli", "connection", "delete"});
        CHECK(del.size() == 5 && del[4] == NEW, "%s: the attempt deleted", k.what);
        CHECK(Find(calls, {"nmcli", "connection", "modify"}).empty(), "%s: nothing saved", k.what);
        std::vector<std::string> back = Find(calls, {"nmcli", "--wait", "10"});
        CHECK(back.size() == 9 && back[6] == A, "%s: rejoined TP-Link_B8CB", k.what);
    }

    Rig r;
    r.runner->active = false;
    r.runner->up = cases[0].up;
    CHECK(r.Join(WifiNetwork{"Home", WifiSecurity::Wpa3Sae, kMarker}).outcome == WifiOutcome::WrongPassword, "wrong");
    CHECK(Find(r.runner->Calls(), {"nmcli", "--wait", "10"}).empty(), "no previous network: nothing to rejoin");

    Rig f;
    f.runner->up = cases[0].up;
    f.runner->restore = Fail(4, "Error: Connection activation failed: The Wi-Fi network could not be found\n");
    std::ostringstream err;
    std::streambuf *old = std::cerr.rdbuf(err.rdbuf());
    CHECK(f.Join(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker}).outcome == WifiOutcome::WrongPassword,
          "the outcome stays");
    std::cerr.rdbuf(old);
    CHECK(err.str().find("could not rejoin") != std::string::npos, "a failed rejoin is a warning: %s", err.str().c_str());
}

void TestNothingStarted()
{
    printf("failures before the attempt: permission, no nmcli, no device; a password echoed\n");
    Rig p;
    p.runner->add = Fail(4, "Error: Failed to add 'Home' connection: Insufficient privileges\n");
    WifiResult res = p.Join(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker});
    CHECK(res.outcome == WifiOutcome::Failed && res.message.find("40-dlidar-network.rules") != std::string::npos,
          "permission: %s", res.message.c_str());
    auto calls = p.runner->Calls();
    CHECK(Find(calls, {"nmcli", "--wait"}).empty() && Find(calls, {"nmcli", "connection", "delete"}).empty(),
          "nothing activated or deleted");

    Rig m;
    m.runner->first = Fail(127, "cannot start nmcli: No such file or directory");
    res = m.Join(WifiNetwork{"Home", WifiSecurity::Open, ""});
    CHECK(res.outcome == WifiOutcome::Failed && res.message.find("nmcli cannot be run") == 0, "no nmcli: %s",
          res.message.c_str());
    CHECK(Find(m.runner->Calls(), {"nmcli", "connection", "add"}).empty(), "no add");

    Rig d;
    d.runner->first = Fail(10, "Error: Device 'wlan0' not found.\n");
    res = d.Join(WifiNetwork{"Home", WifiSecurity::Open, ""});
    CHECK(res.message == "no Wi-Fi device wlan0", "no device: %s", res.message.c_str());

    Rig e;
    e.runner->add = Fail(2, "Error: invalid 'psk': '" + kMarker + "' is not valid\n");
    res = e.Join(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker});
    CHECK(res.outcome == WifiOutcome::Failed && res.message.find(kMarker) == std::string::npos &&
              res.message.find("***") != std::string::npos,
          "the password scrubbed: %s", res.message.c_str());
}

void TestStop()
{
    printf("stop during a join: no done, the attempt deleted\n");
    Rig r;
    r.runner->block_up = true;
    r.wifi->Start();
    bool called = false;
    r.wifi->Connect(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker}, 60s, [&](const WifiResult &) { called = true; });
    for (int i = 0; i < 500 && Find(r.runner->Calls(), {"nmcli", "--wait"}).empty(); i++)
        std::this_thread::sleep_for(1ms);
    auto t0 = std::chrono::steady_clock::now();
    r.wifi->Stop();
    CHECK(std::chrono::steady_clock::now() - t0 < 2s, "stopped at once");
    CHECK(!called, "done not called");
    std::vector<std::string> del = Find(r.runner->Calls(), {"nmcli", "connection", "delete"});
    CHECK(del.size() == 5 && del[4] == NEW, "the attempt deleted");
    CHECK(Find(r.runner->Calls(), {"nmcli", "--wait", "10"}).empty(), "no rejoin while stopping");

    bool late = false;
    r.wifi->Connect(WifiNetwork{"Home", WifiSecurity::Open, ""}, 60s, [&](const WifiResult &x) {
        late = x.outcome == WifiOutcome::Failed;
    });
    CHECK(late, "a join after Stop fails at once");
}

void TestStopBetweenSteps()
{
    printf("stop between two nmcli steps: the next step is cancelled, the cleanup still runs\n");
    Rig r;
    r.runner->block_up = true; // were the cancel lost, the up would wait 10 s
    std::thread stopper;
    std::atomic<bool> stopping{false};
    r.runner->during_add = [&] {
        stopper = std::thread([&] { r.wifi->Stop(); });
        stopping = true;
        for (int i = 0; i < 2000 && !r.runner->CancelPending(); i++)
            std::this_thread::sleep_for(1ms);
    };
    r.wifi->Start();
    bool called = false;
    auto t0 = std::chrono::steady_clock::now();
    r.wifi->Connect(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, kMarker}, 60s, [&](const WifiResult &) { called = true; });
    for (int i = 0; i < 2000 && !stopping; i++)
        std::this_thread::sleep_for(1ms);
    CHECK(stopping, "the add ran");
    if (stopping)
        stopper.join();
    CHECK(std::chrono::steady_clock::now() - t0 < 2s, "stopped at once");
    CHECK(!called, "done not called");
    std::vector<std::vector<std::string>> skipped;
    {
        std::lock_guard<std::mutex> lock(r.runner->mutex);
        skipped = r.runner->skipped;
    }
    CHECK(skipped.size() == 1 && skipped[0].size() > 1 && skipped[0][1] == "--wait", "the up was cancelled before it ran");
    std::vector<std::string> del = Find(r.runner->Calls(), {"nmcli", "connection", "delete"});
    CHECK(del.size() == 5 && del[4] == NEW, "the attempt deleted");
}

void TestLink()
{
    printf("the link: unknown before the start, refreshed at start\n");
    Rig r;
    CHECK(r.wifi->Link().state == WifiState::Unavailable, "unknown before the start");
    r.wifi->Start();
    for (int i = 0; i < 500 && r.wifi->Link().state == WifiState::Unavailable; i++)
        std::this_thread::sleep_for(1ms);
    WifiLink l = r.wifi->Link();
    CHECK(l.state == WifiState::Connected && l.ssid == "TP-Link_B8CB" && l.ip == "192.168.1.106", "connected: %s %s",
          l.ssid.c_str(), l.ip.c_str());
}

void TestProcessRunner()
{
    printf("the real runner: output, exit code, LC_ALL=C, timeout, cancel, no inherited descriptors\n");
    ProcessRunner p;
    CommandOutput o = p.Run({"sh", "-c", "echo out; echo err >&2; exit 4"}, 5s);
    CHECK(o.exit_code == 4 && o.out == "out\n" && o.err == "err\n", "exit %d '%s' '%s'", o.exit_code, o.out.c_str(),
          o.err.c_str());
    setenv("LC_ALL", "hu_HU.UTF-8", 1);
    o = p.Run({"sh", "-c", "echo $LC_ALL"}, 5s);
    unsetenv("LC_ALL");
    CHECK(o.out == "C\n", "LC_ALL: %s", o.out.c_str());
    o = p.Run({"cat"}, 5s);
    CHECK(o.exit_code == 0 && o.out.empty(), "stdin is /dev/null");
    o = p.Run({"no-such-program-dlidar"}, 5s);
    CHECK(o.exit_code == 127, "missing program: %d", o.exit_code);
    auto t0 = std::chrono::steady_clock::now();
    o = p.Run({"sleep", "10"}, 200ms);
    CHECK(o.timed_out && std::chrono::steady_clock::now() - t0 < 2s, "timeout");
    std::thread canceller([&] {
        std::this_thread::sleep_for(200ms);
        p.Cancel();
    });
    t0 = std::chrono::steady_clock::now();
    o = p.Run({"sleep", "10"}, 10s);
    canceller.join();
    CHECK(std::chrono::steady_clock::now() - t0 < 2s, "cancelled");
    o = p.Run({"true"}, 5s);
    CHECK(o.exit_code == 0 && !o.cancelled, "a cancel is used up by the command it killed");
    p.Cancel(); // between two commands: the next one is cancelled, the one after runs
    t0 = std::chrono::steady_clock::now();
    o = p.Run({"sleep", "10"}, 10s);
    CHECK(o.cancelled && std::chrono::steady_clock::now() - t0 < 1s, "cancel between commands");
    o = p.Run({"true"}, 5s);
    CHECK(o.exit_code == 0 && !o.cancelled, "the command after it runs");
    int fd = open("/dev/null", O_RDONLY); // no O_CLOEXEC, like a vendor SDK's socket
    dup2(fd, 50);
    close(fd);
    o = p.Run({"sh", "-c", "test -e /proc/self/fd/50 && echo inherited || echo closed"}, 5s);
    CHECK(o.out == "closed\n", "descriptor 50: %s", o.out.c_str());
    close(50);
    o = p.Run({"sh", "-c", "head -c 100000 /dev/zero"}, 5s);
    CHECK(o.out.size() == ProcessRunner::kMaxOutput && o.exit_code == 0, "output capped: %zu", o.out.size());
}
// Read-only, against this machine's NetworkManager when it has one.
void TestRealNmcli()
{
    ProcessRunner probe;
    CommandOutput o = probe.Run({"nmcli", "-t", "-f", "GENERAL.STATE", "device", "show", "wlan0"}, 5s);
    if (o.exit_code != 0)
    {
        printf("real nmcli: skipped (exit %d)\n", o.exit_code);
        return;
    }
    printf("real nmcli: the link as the service would see it\n");
    CHECK(o.out.compare(0, 14, "GENERAL.STATE:") == 0, "real runner output: %s", o.out.c_str());
    NmcliWifi::Options opt;
    opt.refresh = std::chrono::hours(1);
    NmcliWifi wifi(std::make_unique<ProcessRunner>(), opt);
    wifi.Start();
    for (int i = 0; i < 300 && wifi.Link().state == WifiState::Unavailable; i++)
        std::this_thread::sleep_for(10ms);
    WifiLink l = wifi.Link();
    const char *names[] = {"unavailable", "disconnected", "connecting", "connected"};
    printf("  wlan0: %s, ssid '%s', ip '%s'\n", names[(int)l.state], l.ssid.c_str(), l.ip.c_str());
    CHECK(l.state != WifiState::Unavailable, "NetworkManager answers");
    CHECK(l.state != WifiState::Connected || (!l.ssid.empty() && !l.ip.empty()), "connected with a name and an address");
    wifi.Stop();
}
} // namespace

int main()
{
    TestHelpers();
    TestJoined();
    TestNotJoined();
    TestNothingStarted();
    TestStop();
    TestStopBetweenSteps();
    TestLink();
    TestProcessRunner();
    TestRealNmcli();
    return FinishTests("nmcli_wifi");
}
