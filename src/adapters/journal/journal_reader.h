#pragma once

#include "application/ports/log_reader.h"

#include <string>

namespace dlidar::journal
{
    // The LogReader port over the systemd journal (libsystemd's sd-journal;
    // openspec phone-link-logs, D1, D3-D6): the lines of `unit` and systemd's
    // lines about it, newest first, paged by journal cursors. The journal is
    // opened for each Read; one call reads at most `max_read` entries.
    class JournalReader : public LogReader
    {
    public:
        struct Options
        {
            std::string unit = "dlidar.service";
            int max_read = 5000;
        };

        explicit JournalReader(Options options);

        LogPage Read(const LogQuery &query) override;

    private:
        Options options_;
    };
}
