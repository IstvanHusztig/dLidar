#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <vector>

namespace dlidar::network
{
    struct CommandOutput
    {
        int exit_code = -1;     // 127: the program could not be started
        std::string out, err;   // each cut at kMaxOutput bytes
        bool timed_out = false; // killed after the timeout
        bool cancelled = false; // killed by Cancel()
    };

    // Runs one program with an argument vector, no shell (openspec
    // phone-link-wifi, design D1). Never logs the argument vector: it may
    // hold a password. The NetworkManager adapter's only window to the
    // system, so its tests can script the answers.
    class CommandRunner
    {
    public:
        virtual ~CommandRunner() = default;

        virtual CommandOutput Run(const std::vector<std::string> &argv, std::chrono::milliseconds timeout) = 0;

        // Kills the program Run is waiting for, if any; from any thread.
        virtual void Cancel() = 0;
    };

    // The real runner: posix_spawnp with stdin from /dev/null, stdout and
    // stderr through pipes, LC_ALL=C (fixed English messages), no other file
    // descriptor inherited. One Run at a time.
    class ProcessRunner : public CommandRunner
    {
    public:
        static constexpr size_t kMaxOutput = 64 * 1024;

        CommandOutput Run(const std::vector<std::string> &argv, std::chrono::milliseconds timeout) override;
        void Cancel() override;

    private:
        std::mutex mutex_;
        pid_t pid_ = -1;        // the running child, or -1
        bool cancelled_ = false;
    };
}
