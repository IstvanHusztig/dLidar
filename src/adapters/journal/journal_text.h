#pragma once

#include "application/ports/log_reader.h"

#include <cstddef>
#include <string>
#include <string_view>

// Pure helpers of the journal adapter (openspec phone-link-logs, D3, D5).
namespace dlidar::journal
{
    const size_t kMaxText = 1024;

    // A dLidar line: Warning for "WARNING:", Error for "ERROR:", else Info.
    LogLevel LevelOfLine(std::string_view text);

    // A syslog priority: 0-3 Error, 4 Warning, else Info.
    LogLevel LevelOfPriority(int priority);

    // Valid UTF-8 of at most `max` bytes: invalid bytes become U+FFFD, and the
    // cut falls on a character boundary.
    std::string CleanText(std::string_view raw, size_t max = kMaxText);

    // The first 8 hex digits of a 128-bit boot id.
    std::string ShortBoot(std::string_view boot_id);
}
