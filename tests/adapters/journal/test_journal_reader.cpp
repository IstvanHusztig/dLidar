// The journal adapter (openspec phone-link-logs, design D3-D5, D10): the pure
// helpers, and JournalReader against this machine's journal, read-only
// (skipped when it has no dlidar.service lines). Each returned line is
// checked against the journal directly through sd-journal.
#include "adapters/journal/journal_reader.h"
#include "adapters/journal/journal_text.h"
#include "../../check.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <systemd/sd-journal.h>

using namespace dlidar;
using namespace dlidar::journal;

namespace
{
const std::string kUnit = "dlidar.service";

void TestHelpers()
{
    printf("helpers: levels, the UTF-8 cut, the boot id\n");
    CHECK(LevelOfLine("WARNING: lens 7% dirty") == LogLevel::Warning, "warning");
    CHECK(LevelOfLine("ERROR: cannot write") == LogLevel::Error, "error");
    CHECK(LevelOfLine("Ready: press the button") == LogLevel::Info, "info");
    CHECK(LevelOfLine(" WARNING: indented") == LogLevel::Info, "only at the start");
    CHECK(LevelOfLine("") == LogLevel::Info, "empty");
    for (int p = 0; p <= 3; p++)
        CHECK(LevelOfPriority(p) == LogLevel::Error, "priority %d", p);
    CHECK(LevelOfPriority(4) == LogLevel::Warning, "priority 4");
    for (int p = 5; p <= 7; p++)
        CHECK(LevelOfPriority(p) == LogLevel::Info, "priority %d", p);

    const std::string fffd = "\xEF\xBF\xBD";
    CHECK(CleanText("plain") == "plain", "ascii");
    CHECK(CleanText("h\xC3\xA9 \xF0\x9F\x98\x80") == "h\xC3\xA9 \xF0\x9F\x98\x80", "2- and 4-byte characters kept");
    CHECK(CleanText("a\xFF" "b") == "a" + fffd + "b", "invalid byte");
    CHECK(CleanText("\xC0\xAF") == fffd + fffd, "overlong");
    CHECK(CleanText("\xED\xA0\x80") == fffd + fffd + fffd, "surrogate");
    CHECK(CleanText("x\xE2\x82") == "x" + fffd + fffd, "cut sequence at the end");
    std::string text(1023, 'a');
    CHECK(CleanText(text + "\xC3\xA9") == text, "a character that does not fit is left out");
    CHECK(CleanText(std::string(2000, 'b')).size() == kMaxText, "cut at 1024");
    CHECK(CleanText(text + "\xFF").size() == 1023, "a replacement that does not fit is left out");
    CHECK(CleanText("abcdef", 4) == "abcd", "max");
    CHECK(ShortBoot("36276c9730b3433b91d86044fa35fc8a") == "36276c97", "boot");
}

// Whether the line at `cursor` belongs to the unit (D3).
bool OfUnit(sd_journal *j, const std::string &cursor)
{
    if (sd_journal_seek_cursor(j, cursor.c_str()) < 0 || sd_journal_next(j) <= 0 ||
        sd_journal_test_cursor(j, cursor.c_str()) <= 0)
        return false;
    auto field = [&](const char *name) {
        const void *d;
        size_t n;
        if (sd_journal_get_data(j, name, &d, &n) < 0)
            return std::string();
        return std::string((const char *)d + strlen(name) + 1, n - strlen(name) - 1);
    };
    return field("_SYSTEMD_UNIT") == kUnit || (field("_PID") == "1" && field("UNIT") == kUnit);
}

// The unit's lines from the one at `newest` back to the head, in journal
// order, read directly (not through the reader). Lines written later are
// newer than `newest`, so dLidar logging during the test changes nothing.
std::vector<std::string> LinesBack(const std::string &newest)
{
    std::vector<std::string> v;
    sd_journal *j;
    if (sd_journal_open(&j, SD_JOURNAL_LOCAL_ONLY) < 0)
        return v;
    std::string own = "_SYSTEMD_UNIT=" + kUnit, about = "UNIT=" + kUnit;
    sd_journal_add_match(j, own.c_str(), 0);
    sd_journal_add_disjunction(j);
    sd_journal_add_match(j, "_PID=1", 0);
    sd_journal_add_match(j, about.c_str(), 0);
    if (sd_journal_seek_cursor(j, newest.c_str()) >= 0)
        while (sd_journal_previous(j) > 0)
        {
            char *c = nullptr;
            if (sd_journal_get_cursor(j, &c) < 0)
                break;
            v.push_back(c);
            free(c);
        }
    sd_journal_close(j);
    return v;
}

void TestRealJournal()
{
    JournalReader reader({});
    LogPage first = reader.Read({});
    if (first.entries.empty())
    {
        printf("real journal: skipped (no %s lines readable)\n", kUnit.c_str());
        return;
    }
    printf("real journal: the unit's lines, paging, cursors, bounds\n");
    CHECK(first.entries.size() <= 50, "default limit: %zu", first.entries.size());

    sd_journal *j;
    CHECK(sd_journal_open(&j, SD_JOURNAL_LOCAL_ONLY) >= 0, "open");
    for (const LogEntry &e : first.entries)
    {
        CHECK(OfUnit(j, e.cursor), "of the unit: %s", e.text.c_str());
        CHECK(e.boot.size() == 8 && e.time_ms > 0, "boot '%s', t %lld", e.boot.c_str(), (long long)e.time_ms);
    }
    sd_journal_close(j);

    // Paging to the head: every line once, newest first.
    auto t0 = std::chrono::steady_clock::now();
    LogPage page = reader.Read({std::nullopt, 200, LogLevel::Info});
    auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    printf("  a 200-line page took %.1f ms\n", ms);
    page = reader.Read({std::nullopt, 20, LogLevel::Info}); // small pages, so paging spans several
    // Order by journal position, not by time: the Pi's clock may jump back
    // between boots (D4).
    std::vector<std::string> paged;
    int pages = 1;
    for (;;)
    {
        for (const LogEntry &e : page.entries)
            paged.push_back(e.cursor);
        if (!page.more || pages > 10000)
            break;
        page = reader.Read({page.more, 20, LogLevel::Info});
        pages++;
    }
    std::vector<std::string> direct = paged.empty() ? paged : LinesBack(paged.front());
    const int counted = (int)direct.size();
    printf("  %zu lines in %d pages (%d read directly)\n", paged.size(), pages, counted);
    CHECK(paged == direct, "paging returns every line once, newest first: %zu vs %d", paged.size(), counted);

    // limit 1 from a cursor, then the next page starts just older.
    if (first.entries.size() >= 3)
    {
        LogPage one = reader.Read({first.entries[0].cursor, 1, LogLevel::Info});
        CHECK(one.entries.size() == 1 && one.entries[0].cursor == first.entries[1].cursor && one.more,
              "one line, just older than the cursor");
        LogPage two = reader.Read({one.more, 1, LogLevel::Info});
        CHECK(two.entries.size() == 1 && two.entries[0].cursor == first.entries[2].cursor,
              "the next page continues just older");
    }
    LogPage zero = reader.Read({std::nullopt, 0, LogLevel::Info});
    CHECK(zero.entries.size() == 1, "a limit below 1 reads as 1");

    LogPage warnings = reader.Read({std::nullopt, 200, LogLevel::Warning});
    for (const LogEntry &e : warnings.entries)
        CHECK(e.level != LogLevel::Info, "warning or error: %s", e.text.c_str());

    // D5: a rare level stops after max_read lines.
    JournalReader bounded({kUnit, 50});
    LogPage errors = bounded.Read({std::nullopt, 200, LogLevel::Error});
    for (const LogEntry &e : errors.entries)
        CHECK(e.level == LogLevel::Error, "error: %s", e.text.c_str());
    CHECK(counted <= 50 || errors.more.has_value(), "more after the bound");

    bool threw = false;
    try
    {
        reader.Read({std::string("garbage"), 50, LogLevel::Info});
    }
    catch (const LogCursorError &)
    {
        threw = true;
    }
    CHECK(threw, "a malformed cursor");

    JournalReader other({"no-such-unit-dlidar.service", 5000});
    LogPage none = other.Read({});
    CHECK(none.entries.empty() && !none.more, "another unit has no lines");
}
} // namespace

int main()
{
    TestHelpers();
    TestRealJournal();
    return FinishTests("journal_reader");
}
