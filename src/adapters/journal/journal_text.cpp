#include "adapters/journal/journal_text.h"

namespace dlidar::journal
{
    LogLevel LevelOfLine(std::string_view text)
    {
        if (text.compare(0, 8, "WARNING:") == 0)
            return LogLevel::Warning;
        if (text.compare(0, 6, "ERROR:") == 0)
            return LogLevel::Error;
        return LogLevel::Info;
    }

    LogLevel LevelOfPriority(int priority)
    {
        if (priority >= 0 && priority <= 3)
            return LogLevel::Error;
        if (priority == 4)
            return LogLevel::Warning;
        return LogLevel::Info;
    }

    namespace
    {
        // The length of the valid UTF-8 character at s[i], or 0.
        size_t CharLength(std::string_view s, size_t i)
        {
            unsigned char c = (unsigned char)s[i];
            if (c < 0x80)
                return 1;
            size_t n;
            unsigned min;
            unsigned cp;
            if ((c & 0xE0) == 0xC0)
                n = 2, min = 0x80, cp = c & 0x1F;
            else if ((c & 0xF0) == 0xE0)
                n = 3, min = 0x800, cp = c & 0x0F;
            else if ((c & 0xF8) == 0xF0)
                n = 4, min = 0x10000, cp = c & 0x07;
            else
                return 0;
            if (i + n > s.size())
                return 0;
            for (size_t k = 1; k < n; k++)
            {
                unsigned char cc = (unsigned char)s[i + k];
                if ((cc & 0xC0) != 0x80)
                    return 0;
                cp = (cp << 6) | (cc & 0x3F);
            }
            if (cp < min || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
                return 0;
            return n;
        }
    }

    std::string CleanText(std::string_view raw, size_t max)
    {
        static const char kReplacement[] = "\xEF\xBF\xBD";
        std::string out;
        for (size_t i = 0; i < raw.size();)
        {
            size_t n = CharLength(raw, i);
            size_t add = n ? n : 3;
            if (out.size() + add > max)
                break;
            if (n)
                out.append(raw.substr(i, n));
            else
                out.append(kReplacement, 3);
            i += n ? n : 1;
        }
        return out;
    }

    std::string ShortBoot(std::string_view boot_id) { return std::string(boot_id.substr(0, 8)); }
}
