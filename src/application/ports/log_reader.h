#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace dlidar
{
    enum class LogLevel
    {
        Info,
        Warning,
        Error,
    };

    enum class LogSource
    {
        Dlidar, // the service's own lines
        System, // systemd's lines about the service
    };

    // One log line (openspec phone-link-logs).
    struct LogEntry
    {
        int64_t time_ms = 0; // Unix time, the Pi's clock
        std::string boot;    // short id of the boot it was written in
        LogLevel level = LogLevel::Info;
        LogSource source = LogSource::Dlidar;
        std::string text;   // at most 1024 bytes, valid UTF-8
        std::string cursor; // where this line is; a page cut here continues from it
    };

    struct LogQuery
    {
        std::optional<std::string> before; // a page's `more`; none: from the newest line
        int limit = 50;
        LogLevel min_level = LogLevel::Info;
    };

    // Newest first. `more`: where the next, older page starts; none at the
    // head of the log. May hold fewer than `limit` entries while `more` is set.
    struct LogPage
    {
        std::vector<LogEntry> entries;
        std::optional<std::string> more;
    };

    // `before` is not a cursor.
    struct LogCursorError : std::runtime_error
    {
        using std::runtime_error::runtime_error;
    };

    // The log cannot be read; what() is the reason.
    struct LogUnavailable : std::runtime_error
    {
        using std::runtime_error::runtime_error;
    };

    // Reads the scanner's log, one bounded page per call (D2). Called from
    // one thread.
    class LogReader
    {
    public:
        virtual ~LogReader() = default;

        // Throws LogCursorError or LogUnavailable.
        virtual LogPage Read(const LogQuery &query) = 0;
    };
}
