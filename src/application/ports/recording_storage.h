#pragma once

#include "domain/recording/sensor_chunk.h"

#include <string>

namespace dlidar
{
    // Where recordings go (docs/architecture.md). One recording at a time:
    // Open, Append the chunks in order, Close. Append is called from the
    // capture's writer thread.
    class RecordingStorage
    {
    public:
        virtual ~RecordingStorage() = default;

        // Starts a new recording (the next free session). False if it cannot.
        virtual bool Open() = 0;
        virtual void Append(const SensorChunk &chunk) = 0;
        // Finishes the recording (for files: packages the LAZ). False if that
        // failed; the data is then kept for RecoverLeftovers.
        virtual bool Close() = 0;

        // After each Append, on the same thread: false (with the reason in
        // `why`) when the recording must stop to leave room for finishing it,
        // such as a nearly full disk. The recording is then stopped and
        // closed normally.
        virtual bool HasRoom(std::string &why)
        {
            (void)why;
            return true;
        }

        // Finishes recordings left unfinished by a crash, kill or power loss.
        // With move_aside each goes into its own subfolder, out of the way of
        // new recordings. False if something was found and failed.
        virtual bool RecoverLeftovers(bool move_aside) = 0;

        // For messages: where the recordings are.
        virtual std::string Location() const = 0;
    };
}
