#pragma once

#include "application/ports/device_events.h"
#include "application/remote/device_api.h"
#include "application/remote/wifi_setup.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The phone protocol v1 on the wire (docs/phone-protocol.md): one compact
// UTF-8 JSON object per line. Every Encode* returns one message ending in
// "\n" with no raw newline inside, at most kMaxMessageBytes. nlohmann/json is
// used only in protocol_codec.cpp.
namespace dlidar::phone
{
    const size_t kMaxRequestBytes = 4096;  // including the "\n"
    const size_t kMaxMessageBytes = 16384; // including the "\n"
    const int64_t kMaxId = 2147483647;

    // Error codes (specs/phone-api).
    extern const char *const kBadRequest;
    extern const char *const kUnknownCommand;
    extern const char *const kUnsupportedVersion;
    extern const char *const kTooLarge;
    extern const char *const kTooManyClients;
    extern const char *const kBusy; // not now: the last recording is being saved

    struct Request
    {
        int64_t id = 0;
        std::string cmd;
        std::string args_json = "{}"; // the args object, compact (decoded by the command)
    };

    struct DecodeError
    {
        std::optional<int64_t> id; // empty: no valid id could be read ("id":null)
        std::string code;
        std::string message;
    };

    // The argument names a command accepts, or null if the command is unknown.
    using ArgNames = std::function<const std::vector<std::string> *(const std::string &cmd)>;

    // Decodes one line (without its "\n"). Never throws. False with `error`
    // set: not JSON or not an object, `id` missing or not an integer in
    // 0..kMaxId, `v` missing or not an integer, `v` not 1
    // (unsupported_version), `cmd` missing or not a string, `args` not an
    // object, an unknown command, an argument the command does not define.
    bool DecodeRequest(std::string_view line, const ArgNames &arg_names, Request &request, DecodeError &error);

    std::string EncodeError(std::optional<int64_t> id, const std::string &code, const std::string &message);
    std::string EncodeError(const DecodeError &error); // unsupported_version carries "supported":[1]

    std::string EncodeHello(int64_t id, const std::vector<std::string> &commands, const std::vector<std::string> &events);
    std::string EncodeStatus(int64_t id, const StatusView &status, const std::vector<DeviceWarning> &warnings,
                             const std::optional<RecordingSummary> &summary);
    std::string EncodeInfo(int64_t id, const DeviceInfo &info);
    // recording.start / recording.stop: {"accepted":…,"state":…} (busy is an
    // error, EncodeError with kBusy).
    std::string EncodeCommandResult(int64_t id, bool accepted, DeviceState state_seen);

    // Events: `dropped` > 0 adds "dropped".
    std::string EncodeStateEvent(uint64_t seq, uint64_t dropped, const DeviceStatus &status);
    std::string EncodeWarningEvent(uint64_t seq, uint64_t dropped, const DeviceWarning &warning);
    std::string EncodeProgressEvent(uint64_t seq, uint64_t dropped, int chunks, uint64_t points);
    std::string EncodeSummaryEvent(uint64_t seq, uint64_t dropped, const RecordingSummary &summary);
    // The later result of request `op` (specs/phone-api "Results that come
    // later"): `result_json` is the compact result object.
    std::string EncodeResultEvent(uint64_t seq, uint64_t dropped, int64_t op, const std::string &result_json);
    std::string EncodeResultEvent(uint64_t seq, uint64_t dropped, int64_t op, const std::string &code,
                                  const std::string &message);

    const char *WarningKindName(WarningKind kind); // lens_dirty, storage_failed, ...

    // wifi.connect and wifi.status (openspec phone-link-wifi). The password
    // is in no encoded message and no error text.
    struct WifiConnectArgs
    {
        std::optional<std::string> ssid, security, psk; // empty: not sent
    };
    // False with `message` set when an argument is not a string.
    bool DecodeWifiConnectArgs(const Request &request, WifiConnectArgs &args, std::string &message);

    std::optional<WifiSecurity> WifiSecurityFromName(const std::string &name); // wpa2-psk, wpa3-sae, open
    const char *WifiSecurityName(WifiSecurity security);
    const char *WifiOutcomeName(WifiOutcome outcome); // connected, or the result's error code
    const char *WifiStateName(WifiState state);       // connected, connecting, disconnected, unavailable

    std::string EncodeWifiAccepted(int64_t id, std::chrono::seconds timeout); // {"accepted":true,"timeout_s":…}
    std::string EncodeWifiStatus(int64_t id, const WifiStatusView &status);
    // The result event of the wifi.connect `op` for network `ssid`.
    std::string EncodeWifiResultEvent(uint64_t seq, uint64_t dropped, int64_t op, const std::string &ssid,
                                      const WifiResult &result);
}
