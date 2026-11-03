#pragma once

#include <cstdint>
#include <optional>

namespace dlidar
{
    // Free space where the recordings go, for status queries (the phone).
    // FileRecordingStorage implements it. Thread-safe.
    class StorageSpace
    {
    public:
        virtual ~StorageSpace() = default;

        // Empty when it cannot be read.
        virtual std::optional<uint64_t> FreeBytes() const = 0;
    };
}
