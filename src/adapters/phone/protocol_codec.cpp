#include "adapters/phone/protocol_codec.h"

#include <nlohmann/json.hpp>

namespace dlidar::phone
{
    using Json = nlohmann::ordered_json; // keys in the order written

    const char *const kBadRequest = "bad_request";
    const char *const kUnknownCommand = "unknown_command";
    const char *const kUnsupportedVersion = "unsupported_version";
    const char *const kTooLarge = "too_large";
    const char *const kTooManyClients = "too_many_clients";
    const char *const kBusy = "busy";

    namespace
    {
        // Longest text fields, so every message stays under kMaxMessageBytes
        // (status.get carries up to 16 warnings).
        const size_t kMaxWarningText = 512;
        const size_t kMaxText = 1024;

        // Cut at `max` bytes without splitting a UTF-8 sequence.
        std::string Clip(const std::string &s, size_t max)
        {
            if (s.size() <= max)
                return s;
            size_t n = max;
            while (n > 0 && ((unsigned char)s[n] & 0xC0) == 0x80)
                n--;
            return s.substr(0, n);
        }

        // Compact, UTF-8 (invalid bytes replaced, never throws); JSON escapes
        // every newline inside strings, so the only raw "\n" is the end.
        std::string Line(const Json &j)
        {
            return j.dump(-1, ' ', false, Json::error_handler_t::replace) + "\n";
        }

        Json StatusFields(const DeviceStatus &s)
        {
            Json j;
            j["state"] = StateName(s.state);
            j["chunks"] = s.chunks;
            j["points"] = s.points;
            j["last_ok"] = s.last_ok;
            j["location"] = Clip(s.location, kMaxText);
            return j;
        }

        Json WarningFields(const DeviceWarning &w)
        {
            Json j;
            j["kind"] = WarningKindName(w.kind);
            j["text"] = Clip(w.text, kMaxWarningText);
            return j;
        }

        Json SummaryFields(const RecordingSummary &r)
        {
            Json j;
            j["imu_mode"] = ImuTimeline::ModeName(r.mode);
            j["decision"] = Clip(r.decision, kMaxText);
            j["imu_in"] = r.stats.imu_in;
            j["imu_out"] = r.stats.imu_out;
            j["lost_samples"] = r.stats.lost_samples;
            return j;
        }

        Json Response(int64_t id, Json result)
        {
            Json j;
            j["id"] = id;
            j["ok"] = true;
            j["result"] = std::move(result);
            return j;
        }

        Json Event(const char *name, uint64_t seq, uint64_t dropped)
        {
            Json j;
            j["event"] = name;
            j["seq"] = seq;
            if (dropped > 0)
                j["dropped"] = dropped;
            return j;
        }

        // An integer in [lo, hi], whatever JSON number type it was read as.
        bool IntegerIn(const Json &v, int64_t lo, int64_t hi, int64_t &out)
        {
            if (v.is_number_unsigned())
            {
                uint64_t u = v.get<uint64_t>();
                if (u > (uint64_t)hi)
                    return false;
                out = (int64_t)u;
                return out >= lo;
            }
            if (v.is_number_integer())
            {
                out = v.get<int64_t>();
                return out >= lo && out <= hi;
            }
            return false;
        }
    }

    const char *WarningKindName(WarningKind kind)
    {
        switch (kind)
        {
        case WarningKind::LensDirty: return "lens_dirty";
        case WarningKind::StorageFailed: return "storage_failed";
        case WarningKind::StorageSlow: return "storage_slow";
        case WarningKind::StorageLowSpace: return "storage_low_space";
        case WarningKind::SensorFailed: return "sensor_failed";
        case WarningKind::SensorSilent: return "sensor_silent";
        case WarningKind::RecordingFailed: return "recording_failed";
        }
        return "unknown";
    }

    bool DecodeRequest(std::string_view line, const ArgNames &arg_names, Request &request, DecodeError &error)
    {
        error = DecodeError();
        auto fail = [&](const char *code, std::string message) {
            error.code = code;
            error.message = std::move(message);
            return false;
        };
        Json j = Json::parse(line.begin(), line.end(), nullptr, false);
        if (j.is_discarded())
            return fail(kBadRequest, "not valid JSON");
        if (!j.is_object())
            return fail(kBadRequest, "a request must be a JSON object");

        auto id = j.find("id");
        int64_t id_value = 0;
        if (id == j.end() || !IntegerIn(*id, 0, kMaxId, id_value))
            return fail(kBadRequest, "\"id\" must be an integer from 0 to 2147483647");
        error.id = id_value;

        auto v = j.find("v");
        int64_t version = 0;
        if (v == j.end() || !IntegerIn(*v, INT64_MIN, INT64_MAX, version))
            return fail(kBadRequest, "\"v\" (the protocol version) must be an integer");
        if (version != kProtocolVersion)
            return fail(kUnsupportedVersion, "protocol version " + std::to_string(version) + " is not supported");

        auto cmd = j.find("cmd");
        if (cmd == j.end() || !cmd->is_string())
            return fail(kBadRequest, "\"cmd\" must be a string");
        std::string name = cmd->get<std::string>();

        Json args = Json::object();
        auto a = j.find("args");
        if (a != j.end())
        {
            if (!a->is_object())
                return fail(kBadRequest, "\"args\" must be an object");
            args = *a;
        }

        const std::vector<std::string> *known = arg_names(name);
        if (!known)
            return fail(kUnknownCommand, "unknown command \"" + Clip(name, 64) + "\"");
        for (auto it = args.begin(); it != args.end(); ++it)
        {
            bool ok = false;
            for (const std::string &k : *known)
                ok = ok || k == it.key();
            if (!ok)
                return fail(kBadRequest, "unknown argument \"" + Clip(it.key(), 64) + "\" for " + name);
        }

        request.id = id_value;
        request.cmd = std::move(name);
        request.args_json = args.dump(-1, ' ', false, Json::error_handler_t::replace);
        return true;
    }

    std::string EncodeError(std::optional<int64_t> id, const std::string &code, const std::string &message)
    {
        Json j;
        j["id"] = id ? Json(*id) : Json(nullptr);
        j["ok"] = false;
        j["error"]["code"] = code;
        j["error"]["message"] = Clip(message, kMaxText);
        if (code == kUnsupportedVersion)
            j["error"]["supported"] = Json::array({kProtocolVersion});
        return Line(j);
    }

    std::string EncodeError(const DecodeError &error) { return EncodeError(error.id, error.code, error.message); }

    std::string EncodeHello(int64_t id, const std::vector<std::string> &commands, const std::vector<std::string> &events)
    {
        Json r;
        r["protocol"] = kProtocolVersion;
        r["server"] = "dLidar";
        r["commands"] = commands;
        r["events"] = events;
        r["limits"]["max_request_bytes"] = kMaxRequestBytes;
        r["limits"]["max_message_bytes"] = kMaxMessageBytes;
        return Line(Response(id, std::move(r)));
    }

    std::string EncodeStatus(int64_t id, const StatusView &status, const std::vector<DeviceWarning> &warnings,
                             const std::optional<RecordingSummary> &summary)
    {
        Json r = StatusFields(status.status);
        r["free_bytes"] = status.free_bytes ? Json(*status.free_bytes) : Json(nullptr);
        r["warnings"] = Json::array();
        for (const DeviceWarning &w : warnings)
            r["warnings"].push_back(WarningFields(w));
        r["summary"] = summary ? SummaryFields(*summary) : Json(nullptr);
        return Line(Response(id, std::move(r)));
    }

    std::string EncodeCommandResult(int64_t id, bool accepted, DeviceState state_seen)
    {
        Json r;
        r["accepted"] = accepted;
        r["state"] = StateName(state_seen);
        return Line(Response(id, std::move(r)));
    }

    std::string EncodeInfo(int64_t id, const DeviceInfo &info)
    {
        Json r;
        r["version"] = Clip(info.version, 256);
        r["sensor"] = info.sensor;
        r["sensor_model"] = info.sensor_model;
        r["recording_format"] = info.recording_format;
        r["protocol"] = kProtocolVersion;
        return Line(Response(id, std::move(r)));
    }

    std::string EncodeStateEvent(uint64_t seq, uint64_t dropped, const DeviceStatus &status)
    {
        Json j = Event("state", seq, dropped);
        j["data"] = StatusFields(status);
        return Line(j);
    }

    std::string EncodeWarningEvent(uint64_t seq, uint64_t dropped, const DeviceWarning &warning)
    {
        Json j = Event("warning", seq, dropped);
        j["data"] = WarningFields(warning);
        return Line(j);
    }

    std::string EncodeProgressEvent(uint64_t seq, uint64_t dropped, int chunks, uint64_t points)
    {
        Json j = Event("progress", seq, dropped);
        j["data"]["chunks"] = chunks;
        j["data"]["points"] = points;
        return Line(j);
    }

    std::string EncodeSummaryEvent(uint64_t seq, uint64_t dropped, const RecordingSummary &summary)
    {
        Json j = Event("summary", seq, dropped);
        j["data"] = SummaryFields(summary);
        return Line(j);
    }

    std::string EncodeResultEvent(uint64_t seq, uint64_t dropped, int64_t op, const std::string &result_json)
    {
        Json j = Event("result", seq, dropped);
        j["op"] = op;
        j["ok"] = true;
        Json result = Json::parse(result_json, nullptr, false);
        j["result"] = result.is_object() ? result : Json::object();
        return Line(j);
    }

    std::string EncodeResultEvent(uint64_t seq, uint64_t dropped, int64_t op, const std::string &code,
                                  const std::string &message)
    {
        Json j = Event("result", seq, dropped);
        j["op"] = op;
        j["ok"] = false;
        j["error"]["code"] = code;
        j["error"]["message"] = Clip(message, kMaxText);
        return Line(j);
    }
}
