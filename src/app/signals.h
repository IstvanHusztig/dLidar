#pragma once

#include <atomic>
#include <initializer_list>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <unistd.h>

#include "adapters/storage/crash_guard.h"

// Termination handling for a capture.
//
// SIGINT / SIGTERM / SIGHUP request a stop: the capture loop ends and the
// normal shutdown path writes the remaining data and finalizes the LAZ
// files. A second such signal kills the process; the next start then
// finalizes what was left behind (RecordingStorage::RecoverLeftovers).
//
// On a crash (SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT) the IMU CSV is
// truncated to its last complete row before the process dies, so no partial
// line is left behind. The points are finalized on the next start.

inline std::atomic<bool> &StopRequested()
{
    static std::atomic<bool> stop{false};
    return stop;
}

using dlidar::CrashTruncateFd;
using dlidar::CrashTruncateSize;

namespace shutdown_detail
{
    inline void WriteStderr(const char *msg)
    {
        ssize_t ignored = write(STDERR_FILENO, msg, strlen(msg));
        (void)ignored;
    }

    inline void OnStopSignal(int sig)
    {
        if (StopRequested().exchange(true))
        {
            WriteStderr("\nSecond stop signal, aborting. The recording is finalized on the next start.\n");
            signal(sig, SIG_DFL);
            raise(sig);
            return;
        }
        WriteStderr("\nStop requested, finishing the recording (signal again to abort)...\n");
    }

    inline void OnFatalSignal(int sig)
    {
        int fd = CrashTruncateFd().load();
        if (fd >= 0)
        {
            int ignored = ftruncate(fd, (off_t)CrashTruncateSize().load());
            (void)ignored;
        }
        signal(sig, SIG_DFL);
        raise(sig);
    }
}

inline void InstallSignalHandlers()
{
    // Initialize the function-local statics outside of any handler.
    StopRequested();
    CrashTruncateFd();
    CrashTruncateSize();

    struct sigaction stop = {};
    stop.sa_handler = shutdown_detail::OnStopSignal;
    sigemptyset(&stop.sa_mask);
    for (int sig : {SIGINT, SIGTERM, SIGHUP})
        sigaction(sig, &stop, nullptr);

    struct sigaction fatal = {};
    fatal.sa_handler = shutdown_detail::OnFatalSignal;
    sigemptyset(&fatal.sa_mask);
    fatal.sa_flags = SA_RESETHAND;
    for (int sig : {SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT})
        sigaction(sig, &fatal, nullptr);

    signal(SIGPIPE, SIG_IGN);
}
