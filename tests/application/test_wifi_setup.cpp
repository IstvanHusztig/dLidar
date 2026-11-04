// WifiSetup (openspec phone-link-wifi, design D2): wifi.connect's argument
// rules at their boundaries, one join at a time, the status while a join runs
// and "last" after each outcome. A fake WifiManager only.
#include "application/remote/wifi_setup.h"
#include "../check.h"

#include <string>
#include <vector>

using namespace dlidar;

namespace
{
struct FakeWifi : WifiManager
{
    WifiLink link{WifiState::Connected, "Home", "192.168.1.106"};
    std::vector<WifiNetwork> connects;
    std::vector<std::chrono::seconds> limits;
    std::function<void(const WifiResult &)> pending;
    bool answer_at_once = false; // calls done inside Connect
    WifiResult at_once;

    WifiLink Link() const override { return link; }
    void Connect(const WifiNetwork &n, std::chrono::seconds limit,
                 std::function<void(const WifiResult &)> done) override
    {
        connects.push_back(n);
        limits.push_back(limit);
        if (answer_at_once)
            done(at_once);
        else
            pending = std::move(done);
    }
    void Finish(WifiOutcome o, const std::string &ip = "")
    {
        auto done = std::move(pending);
        pending = nullptr;
        done(WifiResult{o, ip, ""});
    }
};

std::optional<WifiArgError> Check(std::optional<std::string> ssid, std::optional<WifiSecurity> sec,
                                  std::optional<std::string> psk)
{
    WifiNetwork n;
    return WifiSetup::Check(ssid, sec, psk, n);
}

void ExpectBad(const char *what, std::optional<WifiArgError> e, const std::string &argument,
               const std::string &value = "")
{
    CHECK(e, "%s: accepted", what);
    if (!e)
        return;
    CHECK(e->argument == argument, "%s: names \"%s\"", what, e->argument.c_str());
    CHECK(value.empty() || e->message.find(value) == std::string::npos, "%s: message shows the value", what);
}

void TestArguments()
{
    printf("argument rules and boundaries\n");
    const WifiSecurity wpa2 = WifiSecurity::Wpa2Psk, sae = WifiSecurity::Wpa3Sae, open = WifiSecurity::Open;
    const std::string p8 = "abcdefgh", p63(63, 'x'), p64hex(64, 'a'), p65(65, 'a');
    std::string p64nonhex(64, 'a');
    p64nonhex[10] = 'g';

    WifiNetwork n;
    CHECK(!WifiSetup::Check(std::string("Home"), wpa2, p8, n), "8 characters");
    CHECK(n.ssid == "Home" && n.security == wpa2 && n.psk == p8, "filled in");
    CHECK(!Check(std::string("Home"), wpa2, p63), "63 characters");
    CHECK(!Check(std::string("Home"), wpa2, p64hex), "64 hex digits");
    CHECK(!Check(std::string("Home"), wpa2, std::string(64, 'F')), "64 upper-case hex digits");
    CHECK(!Check(std::string("Home"), sae, std::string("pass word ~!")), "WPA3, spaces and symbols");
    CHECK(!Check(std::string("Home"), open, std::nullopt), "open without psk");
    CHECK(!Check(std::string(32, 's'), wpa2, p8), "32-byte ssid");
    CHECK(!Check(std::string("Kávé ☕"), wpa2, p8), "UTF-8 ssid");

    ExpectBad("7 characters", Check(std::string("Home"), wpa2, std::string("abcdefg")), "psk", "abcdefg");
    ExpectBad("65 characters", Check(std::string("Home"), wpa2, p65), "psk", p65);
    ExpectBad("64 non-hex", Check(std::string("Home"), wpa2, p64nonhex), "psk", p64nonhex);
    ExpectBad("control character", Check(std::string("Home"), wpa2, std::string("abc\tdefgh")), "psk", "abc\tdefgh");
    ExpectBad("not ASCII", Check(std::string("Home"), wpa2, std::string("jelszó123")), "psk", "jelszó123");
    ExpectBad("psk missing", Check(std::string("Home"), sae, std::nullopt), "psk");
    ExpectBad("psk with open", Check(std::string("Home"), open, std::string("secret123")), "psk", "secret123");
    ExpectBad("empty psk with open", Check(std::string("Home"), open, std::string("")), "psk");
    ExpectBad("ssid missing", Check(std::nullopt, wpa2, p8), "ssid");
    ExpectBad("empty ssid", Check(std::string(""), wpa2, p8), "ssid");
    ExpectBad("33-byte ssid", Check(std::string(33, 's'), wpa2, p8), "ssid");
    ExpectBad("security missing", Check(std::string("Home"), std::nullopt, p8), "security");
}

void TestOneAtATime()
{
    printf("one join at a time, status and last\n");
    FakeWifi wifi;
    WifiSetup setup(wifi);

    WifiStatusView v = setup.Status();
    CHECK(v.link.state == WifiState::Connected && v.link.ssid == "Home", "the adapter's link");
    CHECK(!v.last, "no join yet");

    int calls = 0;
    WifiResult got;
    WifiNetwork site{"Site", WifiSecurity::Wpa2Psk, "secret123"};
    CHECK(setup.Begin(site, [&](const WifiResult &r) { calls++; got = r; }) == WifiSetup::Begun::Accepted, "accepted");
    CHECK(wifi.connects.size() == 1 && wifi.connects[0].ssid == "Site" && wifi.connects[0].psk == "secret123",
          "passed on");
    CHECK(wifi.limits.size() == 1 && wifi.limits[0] == kWifiConnectLimit, "with the limit");

    v = setup.Status();
    CHECK(v.link.state == WifiState::Connecting && v.link.ssid == "Site" && v.link.ip.empty(), "connecting to Site");
    CHECK(v.last && v.last->ssid == "Site" && !v.last->outcome, "last pending");

    CHECK(setup.Begin(WifiNetwork{"Other", WifiSecurity::Open, ""}, nullptr) == WifiSetup::Begun::Busy, "busy");
    CHECK(wifi.connects.size() == 1, "the second join is not started");

    wifi.Finish(WifiOutcome::WrongPassword);
    CHECK(calls == 1 && got.outcome == WifiOutcome::WrongPassword, "done forwarded once");
    v = setup.Status();
    CHECK(v.link.state == WifiState::Connected && v.link.ssid == "Home", "the adapter's link again");
    CHECK(v.last && v.last->ssid == "Site" && v.last->outcome == WifiOutcome::WrongPassword, "last wrong_password");

    const WifiOutcome outcomes[] = {WifiOutcome::NotFound, WifiOutcome::Timeout, WifiOutcome::Failed,
                                    WifiOutcome::Connected};
    for (WifiOutcome o : outcomes)
    {
        CHECK(setup.Begin(site, nullptr) == WifiSetup::Begun::Accepted, "accepted again");
        wifi.Finish(o, "192.168.1.50");
        v = setup.Status();
        CHECK(v.last && v.last->outcome == o, "last after outcome %d", static_cast<int>(o));
    }
}

void TestAnswerAtOnce()
{
    printf("a join that ends inside Connect (no NetworkManager)\n");
    FakeWifi wifi;
    wifi.link = WifiLink{WifiState::Unavailable, "", ""};
    wifi.answer_at_once = true;
    wifi.at_once = WifiResult{WifiOutcome::Failed, "", "NetworkManager is not running"};
    WifiSetup setup(wifi);
    int calls = 0;
    CHECK(setup.Begin(WifiNetwork{"Home", WifiSecurity::Wpa2Psk, "secret123"},
                      [&](const WifiResult &r) { calls++; CHECK(r.outcome == WifiOutcome::Failed, "failed"); }) ==
              WifiSetup::Begun::Accepted,
          "accepted");
    CHECK(calls == 1, "done once");
    WifiStatusView v = setup.Status();
    CHECK(v.link.state == WifiState::Unavailable, "unavailable");
    CHECK(v.last && v.last->outcome == WifiOutcome::Failed, "last failed");
    CHECK(setup.Begin(WifiNetwork{"Home", WifiSecurity::Open, ""}, nullptr) == WifiSetup::Begun::Accepted,
          "not left busy");
}
}

int main()
{
    TestArguments();
    TestOneAtATime();
    TestAnswerAtOnce();
    return FinishTests("wifi_setup");
}
