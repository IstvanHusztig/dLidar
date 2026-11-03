#include "adapters/networkmanager/command_runner.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace dlidar::network
{
    namespace
    {
        // The environment with LC_ALL=C in place of any locale setting.
        std::vector<std::string> Environment()
        {
            std::vector<std::string> env;
            for (char **e = environ; e && *e; e++)
                if (strncmp(*e, "LC_ALL=", 7) != 0 && strncmp(*e, "LANGUAGE=", 9) != 0)
                    env.push_back(*e);
            env.push_back("LC_ALL=C");
            return env;
        }

        std::vector<char *> Pointers(std::vector<std::string> &v)
        {
            std::vector<char *> p;
            for (std::string &s : v)
                p.push_back(s.data());
            p.push_back(nullptr);
            return p;
        }
    }

    CommandOutput ProcessRunner::Run(const std::vector<std::string> &argv, std::chrono::milliseconds timeout)
    {
        CommandOutput result;
        if (argv.empty())
            return result;
        int out_pipe[2], err_pipe[2];
        if (pipe2(out_pipe, O_CLOEXEC) != 0)
        {
            result.exit_code = 127;
            result.err = std::string("cannot make a pipe: ") + strerror(errno);
            return result;
        }
        if (pipe2(err_pipe, O_CLOEXEC) != 0)
        {
            close(out_pipe[0]);
            close(out_pipe[1]);
            result.exit_code = 127;
            result.err = std::string("cannot make a pipe: ") + strerror(errno);
            return result;
        }

        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
        posix_spawn_file_actions_adddup2(&actions, out_pipe[1], 1);
        posix_spawn_file_actions_adddup2(&actions, err_pipe[1], 2);
        posix_spawn_file_actions_addclosefrom_np(&actions, 3); // dLidar's sockets stay in dLidar
        posix_spawnattr_t attr;
        posix_spawnattr_init(&attr);
        sigset_t none, all;
        sigemptyset(&none);
        sigfillset(&all);
        posix_spawnattr_setsigmask(&attr, &none);
        posix_spawnattr_setsigdefault(&attr, &all);
        posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);

        std::vector<std::string> args = argv, env = Environment();
        std::vector<char *> arg_p = Pointers(args), env_p = Pointers(env);
        pid_t pid = -1;
        int rc;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancelled_ = false;
            rc = posix_spawnp(&pid, arg_p[0], &actions, &attr, arg_p.data(), env_p.data());
            if (rc == 0)
                pid_ = pid;
        }
        posix_spawn_file_actions_destroy(&actions);
        posix_spawnattr_destroy(&attr);
        close(out_pipe[1]);
        close(err_pipe[1]);
        if (rc != 0)
        {
            close(out_pipe[0]);
            close(err_pipe[0]);
            result.exit_code = 127;
            result.err = "cannot start " + argv[0] + ": " + strerror(rc);
            return result;
        }

        // Read both pipes until they close or the time is up.
        auto deadline = std::chrono::steady_clock::now() + timeout;
        pollfd fds[2] = {{out_pipe[0], POLLIN, 0}, {err_pipe[0], POLLIN, 0}};
        std::string *bufs[2] = {&result.out, &result.err};
        int open_fds = 2;
        while (open_fds > 0)
        {
            auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (left.count() <= 0)
            {
                result.timed_out = true;
                break;
            }
            int n = poll(fds, 2, (int)std::min<long long>(left.count(), 200));
            if (n < 0 && errno != EINTR)
                break;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (cancelled_)
                {
                    result.cancelled = true;
                    break;
                }
            }
            for (int i = 0; i < 2; i++)
            {
                if (fds[i].fd < 0 || !(fds[i].revents & (POLLIN | POLLHUP | POLLERR)))
                    continue;
                char buf[4096];
                ssize_t got = read(fds[i].fd, buf, sizeof buf);
                if (got > 0)
                {
                    size_t room = kMaxOutput - std::min(kMaxOutput, bufs[i]->size());
                    bufs[i]->append(buf, std::min((size_t)got, room));
                }
                else if (got == 0 || errno != EINTR)
                {
                    close(fds[i].fd);
                    fds[i].fd = -1;
                    open_fds--;
                }
            }
        }
        for (pollfd &p : fds)
            if (p.fd >= 0)
                close(p.fd);

        if (result.timed_out || result.cancelled)
            kill(pid, SIGTERM);
        int status = 0;
        for (int waited = 0;; waited++)
        {
            pid_t w = waitpid(pid, &status, (result.timed_out || result.cancelled) ? WNOHANG : 0);
            if (w == pid || (w < 0 && errno != EINTR))
                break;
            if (w == 0)
            {
                if (waited == 50) // 500 ms after SIGTERM
                    kill(pid, SIGKILL);
                usleep(10000);
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pid_ = -1;
        }
        result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        return result;
    }

    void ProcessRunner::Cancel()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        if (pid_ > 0)
            kill(pid_, SIGTERM);
    }
}
