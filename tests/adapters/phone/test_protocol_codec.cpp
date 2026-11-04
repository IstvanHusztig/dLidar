// The protocol v1 codec (openspec phone-link-core, specs/phone-api): request
// decoding and every error path, the message shapes, UTF-8, no raw newline,
// every WarningKind named, every message at most 16384 bytes.
#include "adapters/phone/protocol_codec.h"
#include "../../check.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <set>
#include <string>
#include <vector>

using namespace dlidar;
using namespace dlidar::phone;
using Json = nlohmann::json;

namespace
{
const std::vector<std::string> kNoArgs;
const std::vector<std::string> kWifiArgs{"ssid", "psk"};

const std::vector<std::string> *Args(const std::string &cmd)
{
    if (cmd == "hello" || cmd == "status.get" || cmd == "info.get" || cmd == "recording.start" ||
        cmd == "recording.stop")
        return &kNoArgs;
    if (cmd == "test.args")
        return &kWifiArgs;
    return nullptr;
}

// One message: ends in "\n", no other newline, valid JSON, within the limit.
Json Check(const std::string &line, const char *what)
{
    CHECK(!line.empty() && line.back() == '\n', "%s: ends in a newline", what);
    CHECK(line.find('\n') == line.size() - 1, "%s: one newline only", what);
    CHECK(line.size() <= kMaxMessageBytes, "%s: %zu bytes", what, line.size());
    Json j = Json::parse(line, nullptr, false);
    CHECK(j.is_object(), "%s: JSON object: %s", what, line.c_str());
    return j;
}

struct Decoded
{
    bool ok;
    Request r;
    DecodeError e;
};

Decoded Decode(const std::string &line)
{
    Decoded d;
    d.ok = DecodeRequest(line, Args, d.r, d.e);
    return d;
}

void TestRequests()
{
    printf("valid requests\n");
    Decoded d = Decode(R"({"v":1,"id":7,"cmd":"status.get"})");
    CHECK(d.ok && d.r.id == 7 && d.r.cmd == "status.get" && d.r.args_json == "{}", "status.get");
    d = Decode(R"({"cmd":"hello","id":0,"v":1,"args":{},"extra":true})");
    CHECK(d.ok && d.r.id == 0 && d.r.cmd == "hello", "any key order, empty args, unknown top-level field ignored");
    d = Decode(R"({"v":1,"id":2147483647,"cmd":"test.args","args":{"ssid":"Café"}})");
    CHECK(d.ok && d.r.id == 2147483647 && Json::parse(d.r.args_json)["ssid"] == "Café", "largest id, UTF-8 args");
}

void TestErrors()
{
    printf("every decode error\n");
    struct Case
    {
        const char *line;
        const char *code;
        bool has_id;
        const char *text;
    } cases[] = {
        {"not json", "bad_request", false, "JSON"},
        {"", "bad_request", false, "JSON"},
        {R"([1,2])", "bad_request", false, "object"},
        {R"({"v":1,"cmd":"hello"})", "bad_request", false, "id"},
        {R"({"v":1,"id":null,"cmd":"hello"})", "bad_request", false, "id"},
        {R"({"v":1,"id":"7","cmd":"hello"})", "bad_request", false, "id"},
        {R"({"v":1,"id":1.5,"cmd":"hello"})", "bad_request", false, "id"},
        {R"({"v":1,"id":-1,"cmd":"hello"})", "bad_request", false, "id"},
        {R"({"v":1,"id":2147483648,"cmd":"hello"})", "bad_request", false, "id"},
        {R"({"v":1,"id":18446744073709551616,"cmd":"hello"})", "bad_request", false, "id"},
        {R"({"id":3,"cmd":"hello"})", "bad_request", true, "\"v\""},
        {R"({"v":"1","id":3,"cmd":"hello"})", "bad_request", true, "\"v\""},
        {R"({"v":2,"id":3,"cmd":"hello"})", "unsupported_version", true, "version 2"},
        {R"({"v":1,"id":3})", "bad_request", true, "cmd"},
        {R"({"v":1,"id":3,"cmd":5})", "bad_request", true, "cmd"},
        {R"({"v":1,"id":3,"cmd":"hello","args":[]})", "bad_request", true, "args"},
        {R"({"v":1,"id":3,"cmd":"reboot"})", "unknown_command", true, "reboot"},
        {R"({"v":1,"id":3,"cmd":"status.get","args":{"verbose":true}})", "bad_request", true, "verbose"},
        {R"({"v":1,"id":3,"cmd":"test.args","args":{"ssid":"x","channel":6}})", "bad_request", true, "channel"},
    };
    for (const Case &c : cases)
    {
        Decoded d = Decode(c.line);
        CHECK(!d.ok && d.e.code == c.code, "%s: code %s", c.line, d.e.code.c_str());
        CHECK(d.e.id.has_value() == c.has_id && (!c.has_id || *d.e.id == 3), "%s: id", c.line);
        CHECK(d.e.message.find(c.text) != std::string::npos, "%s: message %s", c.line, d.e.message.c_str());

        Json j = Check(EncodeError(d.e), c.line);
        CHECK(j["ok"] == false && j["error"]["code"] == c.code, "%s: encoded", c.line);
        CHECK(c.has_id ? j["id"] == 3 : j["id"].is_null(), "%s: encoded id", c.line);
        CHECK(j["error"].contains("supported") == (std::string(c.code) == "unsupported_version"), "%s: supported",
              c.line);
    }
    Json j = Check(EncodeError(Decode(R"({"v":2,"id":3,"cmd":"hello"})").e), "v2");
    CHECK(j["error"]["supported"] == Json::array({1}), "supported [1]");

    j = Check(EncodeError(std::nullopt, kTooManyClients, "another client is connected"), "too_many_clients");
    CHECK(j["id"].is_null() && j["error"]["code"] == "too_many_clients", "too_many_clients");
    j = Check(EncodeError(std::nullopt, kTooLarge, "a request is at most 4096 bytes"), "too_large");
    CHECK(j["id"].is_null() && j["error"]["code"] == "too_large", "too_large");
}

void TestResponses()
{
    printf("responses\n");
    Json j = Check(EncodeHello(1, {"hello", "status.get", "info.get"}, {"state", "warning", "progress", "summary"}),
                   "hello");
    CHECK(j["id"] == 1 && j["ok"] == true, "hello response");
    Json r = j["result"];
    CHECK(r["protocol"] == 1 && r["server"] == "dLidar" && r["commands"].size() == 3 && r["events"].size() == 4,
          "hello fields");
    CHECK(r["limits"]["max_request_bytes"] == 4096 && r["limits"]["max_message_bytes"] == 16384, "limits");

    StatusView v;
    v.status.state = DeviceState::Recording;
    v.status.chunks = 12;
    v.status.points = 5000000000ull;
    v.status.location = "/home/pi/PointCloudDump/";
    j = Check(EncodeStatus(2, v, {}, std::nullopt), "status");
    r = j["result"];
    CHECK(r["state"] == "recording" && r["chunks"] == 12 && r["points"] == 5000000000ull && r["last_ok"] == true &&
              r["location"] == "/home/pi/PointCloudDump/",
          "status fields");
    CHECK(r["free_bytes"].is_null() && r["warnings"] == Json::array() && r["summary"].is_null(), "unknown free, none");

    v.free_bytes = 64ull << 30;
    RecordingSummary s;
    s.mode = ImuTimeline::Mode::HostAligned;
    s.decision = "IMU stamps do not share the points' clock";
    s.stats.imu_in = 1000;
    s.stats.imu_out = 998;
    s.stats.lost_samples = 2;
    j = Check(EncodeStatus(3, v, {{WarningKind::LensDirty, "lens 7% dirty"}}, s), "status with summary");
    r = j["result"];
    CHECK(r["free_bytes"] == (64ull << 30), "free bytes");
    CHECK(r["warnings"].size() == 1 && r["warnings"][0]["kind"] == "lens_dirty" &&
              r["warnings"][0]["text"] == "lens 7% dirty",
          "warnings");
    CHECK(r["summary"]["imu_mode"] == "host_aligned" && r["summary"]["imu_in"] == 1000 &&
              r["summary"]["imu_out"] == 998 && r["summary"]["lost_samples"] == 2 &&
              r["summary"]["decision"] == s.decision,
          "summary");

    j = Check(EncodeInfo(4, {"0e1e413", "livox_mid360s", "livox_mid360s", 3}), "info");
    r = j["result"];
    CHECK(r["version"] == "0e1e413" && r["sensor"] == "livox_mid360s" && r["sensor_model"] == "livox_mid360s" &&
              r["recording_format"] == 3 && r["protocol"] == 1 && !r.contains("hostname"),
          "info fields");
    j = Check(EncodeInfo(5, {"0e1e413", "unitree_l2", "unitree_l2", 3, "dlidar-01"}), "info with hostname");
    CHECK(j["result"]["hostname"] == "dlidar-01", "hostname");
}

void TestEvents()
{
    printf("events\n");
    DeviceStatus st;
    st.state = DeviceState::Finalizing;
    st.chunks = 3;
    st.points = 42;
    st.location = "/x";
    Json j = Check(EncodeStateEvent(1, 0, st), "state");
    CHECK(j["event"] == "state" && j["seq"] == 1 && !j.contains("dropped") && !j.contains("id"), "state shape");
    CHECK(j["data"]["state"] == "finalizing" && j["data"]["chunks"] == 3 && j["data"]["points"] == 42 &&
              j["data"]["last_ok"] == true && j["data"]["location"] == "/x" && !j["data"].contains("free_bytes"),
          "state data");

    j = Check(EncodeProgressEvent(2, 5, 9, 1234), "progress");
    CHECK(j["seq"] == 2 && j["dropped"] == 5 && j["data"]["chunks"] == 9 && j["data"]["points"] == 1234, "progress");

    j = Check(EncodeWarningEvent(3, 0, {WarningKind::StorageSlow, "chunks pile up"}), "warning");
    CHECK(j["event"] == "warning" && j["data"]["kind"] == "storage_slow" && j["data"]["text"] == "chunks pile up",
          "warning");

    RecordingSummary s;
    s.mode = ImuTimeline::Mode::Sensor;
    s.decision = "sensor stamps good";
    j = Check(EncodeSummaryEvent(4, 0, s), "summary");
    CHECK(j["event"] == "summary" && j["data"]["imu_mode"] == "sensor", "summary");

    j = Check(EncodeResultEvent(5, 0, 12, R"({"connected":true})"), "result ok");
    CHECK(j["event"] == "result" && j["op"] == 12 && j["ok"] == true && j["result"]["connected"] == true, "result ok");
    j = Check(EncodeResultEvent(6, 1, 13, "wrong_password", "the password was rejected"), "result error");
    CHECK(j["op"] == 13 && j["ok"] == false && j["error"]["code"] == "wrong_password" && j["dropped"] == 1,
          "result error");
}

void TestWarningKinds()
{
    printf("every WarningKind named\n");
    const WarningKind kinds[] = {WarningKind::LensDirty,     WarningKind::StorageFailed, WarningKind::StorageSlow,
                                 WarningKind::StorageLowSpace, WarningKind::SensorFailed, WarningKind::SensorSilent,
                                 WarningKind::RecordingFailed};
    const char *names[] = {"lens_dirty",    "storage_failed", "storage_slow",    "storage_low_space",
                           "sensor_failed", "sensor_silent",  "recording_failed"};
    for (size_t i = 0; i < sizeof kinds / sizeof kinds[0]; i++)
        CHECK(std::string(WarningKindName(kinds[i])) == names[i], "%s", names[i]);
}

void TestTextAndLimits()
{
    printf("UTF-8, newlines in text, the 16384-byte limit\n");
    Json j = Check(EncodeWarningEvent(1, 0, {WarningKind::LensDirty, "line one\nline two: ű é ✓"}), "newline in text");
    CHECK(j["data"]["text"] == "line one\nline two: ű é ✓", "text kept, newline escaped");

    j = Check(EncodeWarningEvent(1, 0, {WarningKind::LensDirty, std::string("bad \xff\xfe byte")}), "invalid UTF-8");
    CHECK(j["data"]["text"].get<std::string>().find("bad") == 0, "invalid UTF-8 replaced, not thrown");

    // The largest status: 16 long warnings (2-byte characters, so clipping
    // must not split one), a long decision and location.
    std::string long_text;
    for (int i = 0; i < 3000; i++)
        long_text += "é";
    StatusView v;
    v.status.location = long_text;
    v.free_bytes = UINT64_MAX - 1;
    std::vector<DeviceWarning> ws(16, DeviceWarning{WarningKind::StorageLowSpace, long_text});
    RecordingSummary s;
    s.decision = long_text;
    s.stats.imu_in = UINT64_MAX;
    j = Check(EncodeStatus(kMaxId, v, ws, s), "largest status");
    CHECK(j["result"]["warnings"].size() == 16, "all 16 warnings kept");
    CHECK(Json::parse(EncodeWarningEvent(1, 0, ws[0]))["data"]["text"].get<std::string>().size() <= 512,
          "warning text clipped");
    Check(EncodeError(kMaxId, kBadRequest, long_text), "long error");
    Check(EncodeInfo(1, {long_text, "unitree_l2", "unitree_l2", 3}), "long version");
}
// docs/phone-protocol.md: every example request decodes (or its error is
// shown in the document, byte for byte), every example message from dLidar
// is a valid message, and the encoders produce the document's hello and
// error lines exactly.
void TestDocumentExamples()
{
    printf("docs/phone-protocol.md examples\n");
    std::ifstream in(DLIDAR_SOURCE_DIR "/docs/phone-protocol.md");
    CHECK(in.good(), "cannot read docs/phone-protocol.md");
    std::vector<std::string> requests;
    std::set<std::string> replies;
    std::string line;
    const std::string to = "\xe2\x86\x92 ", from = "\xe2\x86\x90 "; // "→ ", "← "
    while (std::getline(in, line))
    {
        if (line.compare(0, to.size(), to) == 0 && line.compare(to.size(), 1, "{") == 0)
            requests.push_back(line.substr(to.size()));
        else if (line.compare(0, from.size(), from) == 0)
            replies.insert(line.substr(from.size()) + "\n");
    }
    CHECK(requests.size() >= 8 && replies.size() >= 15, "%zu requests, %zu replies", requests.size(), replies.size());

    for (const std::string &r : replies)
    {
        Json j = Check(r, r.c_str());
        CHECK((j.contains("id") && j.contains("ok")) || (j.contains("event") && j.contains("seq")), "shape: %s",
              r.c_str());
        if (j.contains("event") && j["event"] != "result")
            CHECK(j["data"].is_object(), "data: %s", r.c_str());
    }

    static const std::vector<std::string> wifi{"ssid", "psk", "security"}, log{"before", "limit", "level"};
    auto args = [](const std::string &cmd) -> const std::vector<std::string> * {
        return cmd == "wifi.connect"  ? &wifi
               : cmd == "wifi.status" ? &kNoArgs
               : cmd == "log.get"     ? &log
                                      : Args(cmd);
    };
    for (const std::string &q : requests)
    {
        Request r;
        DecodeError e;
        if (!DecodeRequest(q, args, r, e))
            CHECK(replies.count(EncodeError(e)), "%s: its error %s is not in the document", q.c_str(),
                  EncodeError(e).c_str());
    }
    Request r;
    DecodeError e;
    CHECK(!DecodeRequest("this is not JSON", args, r, e) && replies.count(EncodeError(e)), "not JSON example");
    CHECK(replies.count(EncodeHello(1,
                                    {"hello", "status.get", "info.get", "recording.start", "recording.stop",
                                     "wifi.connect", "wifi.status", "log.get"},
                                    {"state", "warning", "progress", "summary"})),
          "hello example");
    CHECK(replies.count(EncodeCommandResult(5, true, DeviceState::Idle)), "recording.start example");
    CHECK(replies.count(EncodeCommandResult(6, false, DeviceState::Recording)), "start ignored example");
    CHECK(replies.count(EncodeCommandResult(7, true, DeviceState::Recording)), "recording.stop example");
    CHECK(replies.count(EncodeCommandResult(8, false, DeviceState::Idle)), "stop ignored example");
    CHECK(replies.count(EncodeError(9, kBusy, "the last recording is still being saved; try again when it is idle")),
          "busy example");
    CHECK(replies.count(EncodeError(std::nullopt, kTooManyClients, "another client is connected")),
          "too_many_clients example");

    // Wi-Fi (openspec phone-link-wifi): the answers and results as the code makes them.
    CHECK(replies.count(EncodeWifiAccepted(12, kWifiConnectLimit)), "results-later example: accepted");
    CHECK(replies.count(EncodeWifiResultEvent(14, 0, 12, "Home", WifiResult{WifiOutcome::WrongPassword, "", ""})),
          "results-later example: wrong_password");
    CHECK(replies.count(EncodeWifiAccepted(20, kWifiConnectLimit)), "wifi.connect example: accepted");
    CHECK(replies.count(EncodeWifiResultEvent(4, 0, 20, "Home", WifiResult{WifiOutcome::Connected, "192.168.1.50", ""})),
          "wifi.connect example: connected");
    CHECK(replies.count(EncodeError(21, kBusy, "another Wi-Fi join is still running; try again when it has ended")),
          "wifi.connect example: busy");
    WifiNetwork n;
    std::optional<WifiArgError> bad = WifiSetup::Check(std::string("Home"), WifiSecurity::Wpa2Psk, std::string("short"), n);
    CHECK(bad && replies.count(EncodeError(22, kBadRequest, bad->message)), "wifi.connect example: bad psk");
    WifiStatusView v;
    v.link = WifiLink{WifiState::Connecting, "Home", ""};
    v.last = WifiLast{"Home", std::nullopt};
    CHECK(replies.count(EncodeWifiStatus(23, v)), "wifi.status example: pending");
    v.link = WifiLink{WifiState::Connected, "Home", "192.168.1.50"};
    v.last = WifiLast{"Home", WifiOutcome::Connected};
    CHECK(replies.count(EncodeWifiStatus(24, v)), "wifi.status example: connected");

    // log.get (openspec phone-link-logs): the pages, the bad limit, unavailable.
    const std::string c1 = "s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fa91;b=36276c9730b3433b91d86044fa35fc8a;m=8f1e2a;t=65f6ee2e1c4a2;x=7d1c0e5f3a9b2c44",
                      c2 = "s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fa8e;b=36276c9730b3433b91d86044fa35fc8a;m=8b0c11;t=65f6ee2d4f8a0;x=1a2b3c4d5e6f7081",
                      c3 = "s=3e6faae9ca9e45fb808bd1e603aa7e2d;i=3fac7;b=36276c9730b3433b91d86044fa35fc8a;m=c1efbf;t=65f6ee330bd6f;x=22fe7aa7b597862e";
    const LogEntry ready{1793781326123, "36276c97", LogLevel::Info, LogSource::Dlidar,
                         "Ready: press the button (GPIO 17) to start a recording, again to save it; hold it 5 s to power down.", "c4"},
        gone{1793781325870, "36276c97", LogLevel::Warning, LogSource::Dlidar,
             "WARNING: Bluetooth (bluetoothd) is gone; the phone link waits for it", c3},
        started{1793781320410, "36276c97", LogLevel::Info, LogSource::System,
                "Started dlidar.service - dLidar LiDAR scanner (push button recording, phone link).", c1},
        scheduled{1793781315402, "36276c97", LogLevel::Info, LogSource::System,
                  "dlidar.service: Scheduled restart job, restart counter is at 1.", "c5"},
        failed{1793781310398, "36276c97", LogLevel::Warning, LogSource::System,
               "dlidar.service: Failed with result 'signal'.", c2};
    CHECK(replies.count(EncodeLogPage(30, LogPage{{ready, gone, started}, c1})), "log.get example: first page");
    CHECK(replies.count(EncodeLogPage(31, LogPage{{scheduled, failed}, c2})), "log.get example: next page");
    CHECK(replies.count(EncodeLogPage(32, LogPage{{gone}, c3})), "log.get example: warnings");
    CHECK(DecodeRequest(R"({"v":1,"id":33,"cmd":"log.get","args":{"limit":500}})", args, r, e), "decoded");
    LogQuery q;
    std::string message;
    CHECK(!DecodeLogGetArgs(r, q, message) && replies.count(EncodeError(33, kBadRequest, message)),
          "log.get example: bad limit");
    CHECK(replies.count(EncodeError(34, kUnavailable, "cannot open the journal: Permission denied")),
          "log.get example: unavailable");
}
void TestWifi()
{
    printf("wifi.connect arguments, wifi.status, the result event\n");
    static const std::vector<std::string> wifi{"ssid", "psk", "security"};
    auto args = [](const std::string &cmd) -> const std::vector<std::string> * {
        return cmd == "wifi.connect" ? &wifi : Args(cmd);
    };
    Request r;
    DecodeError e;
    WifiConnectArgs a;
    std::string message;
    CHECK(DecodeRequest(R"({"v":1,"id":12,"cmd":"wifi.connect","args":{"ssid":"Home","security":"wpa2-psk","psk":"secret123"}})",
                        args, r, e),
          "decoded");
    CHECK(DecodeWifiConnectArgs(r, a, message), "args: %s", message.c_str());
    CHECK(a.ssid == std::string("Home") && a.security == std::string("wpa2-psk") && a.psk == std::string("secret123"),
          "the three arguments");
    CHECK(DecodeRequest(R"({"v":1,"id":13,"cmd":"wifi.connect","args":{"ssid":"Café","security":"open"}})", args, r, e),
          "decoded");
    CHECK(DecodeWifiConnectArgs(r, a, message) && a.ssid == std::string("Café") && !a.psk, "psk left out");
    CHECK(DecodeRequest(R"({"v":1,"id":14,"cmd":"wifi.connect","args":{"ssid":"Home","psk":12345678}})", args, r, e),
          "decoded");
    CHECK(!DecodeWifiConnectArgs(r, a, message) && message == "\"psk\" must be a string" &&
              message.find("12345678") == std::string::npos,
          "not a string: %s", message.c_str());

    CHECK(WifiSecurityFromName("wpa2-psk") == WifiSecurity::Wpa2Psk && WifiSecurityFromName("wpa3-sae") == WifiSecurity::Wpa3Sae &&
              WifiSecurityFromName("open") == WifiSecurity::Open && !WifiSecurityFromName("wep") &&
              !WifiSecurityFromName("WPA2-PSK"),
          "security names");

    CHECK(EncodeWifiAccepted(12, std::chrono::seconds(60)) == "{\"id\":12,\"ok\":true,\"result\":{\"accepted\":true,\"timeout_s\":60}}\n",
          "accepted: %s", EncodeWifiAccepted(12, std::chrono::seconds(60)).c_str());

    WifiStatusView v;
    v.link = WifiLink{WifiState::Connected, "Home", "192.168.1.50"};
    v.last = WifiLast{"Home", WifiOutcome::Connected};
    CHECK(EncodeWifiStatus(15, v) ==
              "{\"id\":15,\"ok\":true,\"result\":{\"state\":\"connected\",\"ssid\":\"Home\",\"ip\":\"192.168.1.50\",\"last\":{\"ssid\":\"Home\",\"result\":\"connected\"}}}\n",
          "status connected: %s", EncodeWifiStatus(15, v).c_str());
    v.link = WifiLink{WifiState::Connecting, "Site", ""};
    v.last = WifiLast{"Site", std::nullopt};
    CHECK(EncodeWifiStatus(16, v) ==
              "{\"id\":16,\"ok\":true,\"result\":{\"state\":\"connecting\",\"ssid\":\"Site\",\"ip\":null,\"last\":{\"ssid\":\"Site\",\"result\":\"pending\"}}}\n",
          "status pending: %s", EncodeWifiStatus(16, v).c_str());
    v.link = WifiLink{WifiState::Unavailable, "", ""};
    v.last.reset();
    CHECK(EncodeWifiStatus(17, v) ==
              "{\"id\":17,\"ok\":true,\"result\":{\"state\":\"unavailable\",\"ssid\":null,\"ip\":null,\"last\":null}}\n",
          "status unavailable: %s", EncodeWifiStatus(17, v).c_str());
    v.link = WifiLink{WifiState::Disconnected, "", ""};
    v.last = WifiLast{"Site", WifiOutcome::WrongPassword};
    Json s = Check(EncodeWifiStatus(18, v), "status disconnected");
    CHECK(s["result"]["state"] == "disconnected" && s["result"]["last"]["result"] == "wrong_password", "disconnected");

    CHECK(EncodeWifiResultEvent(14, 0, 12, "Home", WifiResult{WifiOutcome::Connected, "192.168.1.50", ""}) ==
              "{\"event\":\"result\",\"seq\":14,\"op\":12,\"ok\":true,\"result\":{\"ssid\":\"Home\",\"ip\":\"192.168.1.50\"}}\n",
          "result connected");
    CHECK(EncodeWifiResultEvent(3, 2, 12, "Home", WifiResult{WifiOutcome::Connected, "", ""}) ==
              "{\"event\":\"result\",\"seq\":3,\"dropped\":2,\"op\":12,\"ok\":true,\"result\":{\"ssid\":\"Home\",\"ip\":null}}\n",
          "result connected, no address");
    CHECK(EncodeWifiResultEvent(14, 0, 12, "Home", WifiResult{WifiOutcome::WrongPassword, "", ""}) ==
              "{\"event\":\"result\",\"seq\":14,\"op\":12,\"ok\":false,\"error\":{\"code\":\"wrong_password\",\"message\":\"the network rejected the password\"}}\n",
          "result wrong_password");
    const WifiOutcome errors[] = {WifiOutcome::NotFound, WifiOutcome::Timeout, WifiOutcome::Failed};
    const char *codes[] = {"not_found", "timeout", "failed"};
    for (int i = 0; i < 3; i++)
    {
        Json j = Check(EncodeWifiResultEvent(1, 0, 5, "Home", WifiResult{errors[i], "", ""}), codes[i]);
        CHECK(j["ok"] == false && j["error"]["code"] == codes[i] && !j["error"]["message"].get<std::string>().empty(),
              "%s", codes[i]);
    }
    Json f = Check(EncodeWifiResultEvent(1, 0, 5, "Home", WifiResult{WifiOutcome::Failed, "", std::string(5000, 'm')}),
                   "long message");
    CHECK(f["error"]["message"].get<std::string>().size() <= 1024, "message clipped");
}

LogEntry Entry(int64_t t, LogLevel level, LogSource source, const std::string &text, const std::string &cursor)
{
    return LogEntry{t, "36276c97", level, source, text, cursor};
}

void TestLogGet()
{
    printf("log.get arguments and pages\n");
    static const std::vector<std::string> log{"before", "limit", "level"};
    auto args = [](const std::string &cmd) -> const std::vector<std::string> * {
        return cmd == "log.get" ? &log : Args(cmd);
    };
    auto decode = [&](const std::string &a, LogQuery &q, std::string &message) {
        Request r;
        DecodeError e;
        CHECK(DecodeRequest(R"({"v":1,"id":30,"cmd":"log.get","args":)" + a + "}", args, r, e), "decoded %s", a.c_str());
        return DecodeLogGetArgs(r, q, message);
    };
    LogQuery q;
    std::string message;
    CHECK(decode("{}", q, message) && !q.before && q.limit == 50 && q.min_level == LogLevel::Info, "defaults");
    CHECK(decode(R"({"before":"s=1;i=2","limit":200,"level":"warning"})", q, message) &&
              q.before == std::string("s=1;i=2") && q.limit == 200 && q.min_level == LogLevel::Warning,
          "all three");
    CHECK(decode(R"({"limit":1,"level":"error"})", q, message) && q.limit == 1 && q.min_level == LogLevel::Error,
          "limit 1, error");
    CHECK(decode(R"({"level":"info"})", q, message) && q.min_level == LogLevel::Info, "info");
    for (const char *bad : {R"({"limit":0})", R"({"limit":201})", R"({"limit":-5})", R"({"limit":50.5})",
                            R"({"limit":"50"})", R"({"limit":null})"})
    {
        CHECK(!decode(bad, q, message) && message == "\"limit\" must be an integer from 1 to 200", "%s: %s", bad,
              message.c_str());
    }
    for (const char *bad : {R"({"level":"debug"})", R"({"level":"WARNING"})", R"({"level":2})"})
        CHECK(!decode(bad, q, message) && message == "\"level\" must be info, warning or error", "%s: %s", bad,
              message.c_str());
    for (const char *bad : {R"({"before":12})", R"({"before":null})", R"({"before":["a"]})"})
        CHECK(!decode(bad, q, message) && message == "\"before\" must be a string", "%s: %s", bad, message.c_str());

    LogPage page;
    page.entries.push_back(Entry(1762246526123, LogLevel::Warning, LogSource::Dlidar,
                                 "WARNING: Bluetooth (bluetoothd) is gone; the phone link waits for it", "c2"));
    page.entries.push_back(Entry(1762246520000, LogLevel::Info, LogSource::System, "Started dlidar.service.", "c1"));
    page.more = "c1";
    CHECK(EncodeLogPage(31, page) ==
              "{\"id\":31,\"ok\":true,\"result\":{\"entries\":[{\"t\":1762246526123,\"boot\":\"36276c97\",\"level\":\"warning\","
              "\"source\":\"dlidar\",\"text\":\"WARNING: Bluetooth (bluetoothd) is gone; the phone link waits for it\"},"
              "{\"t\":1762246520000,\"boot\":\"36276c97\",\"level\":\"info\",\"source\":\"system\",\"text\":\"Started "
              "dlidar.service.\"}],\"more\":\"c1\"}}\n",
          "page: %s", EncodeLogPage(31, page).c_str());
    CHECK(EncodeLogPage(32, LogPage{}) == "{\"id\":32,\"ok\":true,\"result\":{\"entries\":[],\"more\":null}}\n",
          "empty page");

    // D5: 200 long lines are cut to fit, `more` at the last entry kept.
    LogPage big;
    const std::string cursor(130, 'x');
    for (int i = 0; i < 200; i++)
        big.entries.push_back(Entry(1762246526123 - i, LogLevel::Info, LogSource::Dlidar, std::string(1024, 'a' + i % 26),
                                    cursor + std::to_string(i)));
    big.more = "end";
    Json j = Check(EncodeLogPage(33, big), "big page");
    size_t kept = j["result"]["entries"].size();
    CHECK(kept >= 13 && kept < 200, "kept %zu", kept);
    CHECK(j["result"]["more"] == cursor + std::to_string(kept - 1), "more at the cut");
    // Control characters escape to 6 bytes each: still within the limit.
    for (LogEntry &e : big.entries)
        e.text = std::string(1024, '\x01');
    j = Check(EncodeLogPage(34, big), "escaped page");
    kept = j["result"]["entries"].size();
    CHECK(kept >= 2 && j["result"]["more"] == cursor + std::to_string(kept - 1), "escaped: kept %zu", kept);
    // Everything fits: `more` is the page's own.
    big.entries.resize(3);
    for (LogEntry &e : big.entries)
        e.text = "short";
    j = Check(EncodeLogPage(35, big), "small page");
    CHECK(j["result"]["entries"].size() == 3 && j["result"]["more"] == "end", "page's more");
    big.more.reset();
    j = Check(EncodeLogPage(36, big), "last page");
    CHECK(j["result"]["more"].is_null(), "head of the log");
}
} // namespace

int main()
{
    TestWifi();
    TestLogGet();
    TestRequests();
    TestErrors();
    TestResponses();
    TestEvents();
    TestWarningKinds();
    TestTextAndLimits();
    TestDocumentExamples();
    return FinishTests("protocol_codec");
}
