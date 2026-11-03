#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <string_view>

namespace dlidar::phone
{
    // Splits a connection's byte stream into lines ended by "\n" (the
    // protocol's framing): bytes may arrive in any pieces. A line longer than
    // `max_bytes` including its "\n" is reported once as too large and
    // discarded up to the next "\n"; the line after it is read normally.
    class LineFramer
    {
    public:
        explicit LineFramer(size_t max_bytes = 4096) : max_bytes_(max_bytes) {}

        // `on_line` gets each complete line without its "\n".
        void Feed(std::string_view data, const std::function<void(std::string_view)> &on_line,
                  const std::function<void()> &on_too_large)
        {
            while (!data.empty())
            {
                size_t nl = data.find('\n');
                std::string_view part = data.substr(0, nl);
                if (!discarding_)
                {
                    buffer_.append(part.data(), part.size());
                    // The "\n" still to come makes it one byte longer.
                    if (buffer_.size() + 1 > max_bytes_)
                    {
                        buffer_.clear();
                        discarding_ = true;
                        on_too_large();
                    }
                }
                if (nl == std::string_view::npos)
                    return;
                if (!discarding_)
                    on_line(buffer_);
                buffer_.clear();
                discarding_ = false;
                data.remove_prefix(nl + 1);
            }
        }

        // A new connection: drops a partial line.
        void Reset()
        {
            buffer_.clear();
            discarding_ = false;
        }

    private:
        size_t max_bytes_;
        std::string buffer_;
        bool discarding_ = false;
    };
}
