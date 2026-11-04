#include "adapters/journal/journal_reader.h"

#include "adapters/journal/journal_text.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <systemd/sd-journal.h>

namespace dlidar::journal
{
    namespace
    {
        using Journal = std::unique_ptr<sd_journal, decltype(&sd_journal_close)>;

        // The value of `field` in the current entry, or empty. A copy: the
        // journal's buffer is reused by the next sd_journal_get_data.
        std::string Field(sd_journal *j, const char *field)
        {
            const void *data;
            size_t length;
            if (sd_journal_get_data(j, field, &data, &length) < 0)
                return {};
            size_t skip = strlen(field) + 1; // "FIELD="
            if (length < skip)
                return {};
            return std::string((const char *)data + skip, length - skip);
        }

        std::string Cursor(sd_journal *j)
        {
            char *c = nullptr;
            if (sd_journal_get_cursor(j, &c) < 0 || !c)
                return "";
            std::string s = c;
            free(c);
            return s;
        }

        std::string Reason(const char *what, int r) { return std::string(what) + ": " + strerror(-r); }
    }

    JournalReader::JournalReader(Options options) : options_(std::move(options)) {}

    LogPage JournalReader::Read(const LogQuery &query)
    {
        sd_journal *raw = nullptr;
        int r = sd_journal_open(&raw, SD_JOURNAL_LOCAL_ONLY);
        if (r < 0)
            throw LogUnavailable(Reason("cannot open the journal", r));
        Journal j(raw, sd_journal_close);
        // A text is cut at kMaxText anyway; no need to decompress more.
        sd_journal_set_data_threshold(j.get(), 4 * kMaxText);

        // D3: the unit's own lines, or systemd's lines about it.
        const std::string own = "_SYSTEMD_UNIT=" + options_.unit, about = "UNIT=" + options_.unit;
        if ((r = sd_journal_add_match(j.get(), own.c_str(), 0)) < 0 || (r = sd_journal_add_disjunction(j.get())) < 0 ||
            (r = sd_journal_add_match(j.get(), "_PID=1", 0)) < 0 ||
            (r = sd_journal_add_match(j.get(), about.c_str(), 0)) < 0)
            throw LogUnavailable(Reason("cannot filter the journal", r));

        // D4: from the newest line, or from the cursor. After a cursor seek
        // the first step lands on the cursor's own line (skipped), or on the
        // nearest older one if it was removed.
        if (query.before)
        {
            if (sd_journal_seek_cursor(j.get(), query.before->c_str()) < 0)
                throw LogCursorError("\"before\" is not a log cursor");
        }
        else if ((r = sd_journal_seek_tail(j.get())) < 0)
            throw LogUnavailable(Reason("cannot read the journal", r));

        const int limit = std::max(1, query.limit);
        LogPage page;
        std::string last; // the cursor of the last line read
        bool first = true, past = false; // past: a line other than `before`'s own was read
        for (int read = 0;; read++)
        {
            if ((int)page.entries.size() >= limit)
            {
                // Full: `more` only if an older line exists.
                r = sd_journal_previous(j.get());
                if (r != 0)
                    page.more = last;
                break;
            }
            if (read >= options_.max_read)
            {
                page.more = last; // D5: continue from here
                break;
            }
            r = sd_journal_previous(j.get());
            if (r == 0)
                break; // the head of the log
            if (r < 0)
            {
                // Nothing read past the cursor: `more` would repeat `before`,
                // and a client paging to the end would never stop.
                if (!past)
                    throw LogUnavailable(Reason("cannot read the journal", r));
                page.more = last;
                break;
            }
            last = Cursor(j.get());
            if (first)
            {
                first = false;
                if (query.before && sd_journal_test_cursor(j.get(), query.before->c_str()) > 0)
                    continue;
            }
            past = true;

            LogEntry e;
            e.source = Field(j.get(), "_SYSTEMD_UNIT") == options_.unit ? LogSource::Dlidar : LogSource::System;
            std::string message = Field(j.get(), "MESSAGE");
            if (e.source == LogSource::Dlidar)
                e.level = LevelOfLine(message);
            else
            {
                std::string p = Field(j.get(), "PRIORITY");
                e.level = LevelOfPriority(p.size() == 1 ? p[0] - '0' : 6);
            }
            if (e.level < query.min_level)
                continue;
            uint64_t usec = 0;
            sd_journal_get_realtime_usec(j.get(), &usec);
            e.time_ms = (int64_t)(usec / 1000);
            e.boot = ShortBoot(Field(j.get(), "_BOOT_ID"));
            e.text = CleanText(message);
            e.cursor = last;
            page.entries.push_back(std::move(e));
        }
        return page;
    }
}
