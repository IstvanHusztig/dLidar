#include "adapters/phone/socket_transport.h"

#include "adapters/phone/protocol_codec.h"

#include <glib-unix.h>
#include <glib.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace dlidar::phone
{
    std::string SocketDirectory()
    {
        const char *rd = getenv("RUNTIME_DIRECTORY");
        if (rd && *rd)
        {
            std::string first = rd;
            return first.substr(0, first.find(':'));
        }
        const char *xdg = getenv("XDG_RUNTIME_DIR");
        return xdg && *xdg ? xdg : "";
    }

    namespace
    {
        std::string Errno(const char *what) { return std::string(what) + ": " + strerror(errno); }

        GSource *Watch(GMainContext *context, int fd, GIOCondition condition, GUnixFDSourceFunc fn, void *data)
        {
            GSource *s = g_unix_fd_source_new(fd, condition);
            g_source_set_callback(s, (GSourceFunc)(void *)fn, data, nullptr);
            g_source_attach(s, context);
            return s;
        }

        void Drop(GSource *&s)
        {
            if (s)
            {
                g_source_destroy(s);
                g_source_unref(s);
                s = nullptr;
            }
        }
    }

    // One accepted client. Close() takes effect at once (no more I/O); the
    // handler's OnClosed and the deletion come from an idle callback, so a
    // Send that closes never pulls the object from under its caller.
    class SocketTransport::Client : public Connection
    {
    public:
        Client(SocketTransport &t, int fd) : t_(t), fd_(fd)
        {
            in_ = Watch(t_.context_, fd_, (GIOCondition)(G_IO_IN | G_IO_HUP | G_IO_ERR), OnIn, this);
        }
        ~Client() override
        {
            Shut();
            Drop(closed_source_); // CloseAll came first
        }

        void Send(const std::string &bytes) override
        {
            if (fd_ < 0)
                return;
            if (Pending() + bytes.size() > t_.cap_bytes_)
            {
                Close(); // the client does not read
                return;
            }
            out_.append(bytes);
            Write();
        }

        size_t Pending() const override { return out_.size() - sent_; }

        void Close() override
        {
            if (fd_ < 0)
                return;
            Shut();
            closed_source_ = g_idle_source_new();
            g_source_set_callback(closed_source_, Closed, this, nullptr);
            g_source_attach(closed_source_, t_.context_);
        }

        // No more I/O (also for CloseAll, which calls OnClosed itself).
        void Shut()
        {
            Drop(in_);
            Drop(out_watch_);
            if (fd_ >= 0)
                close(fd_);
            fd_ = -1;
        }

    private:
        SocketTransport &t_;
        int fd_;
        GSource *in_ = nullptr, *out_watch_ = nullptr, *closed_source_ = nullptr;
        std::string out_;
        size_t sent_ = 0;

        // As much as the socket takes now; the rest waits for G_IO_OUT.
        // Returns true if everything went out.
        bool Write()
        {
            while (Pending() > 0)
            {
                ssize_t n = send(fd_, out_.data() + sent_, Pending(), MSG_NOSIGNAL | MSG_DONTWAIT);
                if (n > 0)
                {
                    sent_ += (size_t)n;
                    continue;
                }
                if (n < 0 && errno == EINTR)
                    continue;
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                {
                    if (!out_watch_)
                        out_watch_ = Watch(t_.context_, fd_, G_IO_OUT, OnOut, this);
                    return false;
                }
                Close();
                return false;
            }
            out_.clear();
            sent_ = 0;
            return true;
        }

        static gboolean OnIn(gint, GIOCondition, gpointer data)
        {
            Client *c = static_cast<Client *>(data);
            char buf[4096];
            for (int i = 0; i < 16 && c->fd_ >= 0; i++)
            {
                ssize_t n = recv(c->fd_, buf, sizeof buf, MSG_DONTWAIT);
                if (n > 0)
                {
                    c->t_.handler_.OnBytes(*c, std::string_view(buf, (size_t)n));
                    continue;
                }
                if (n < 0 && errno == EINTR)
                    continue;
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                    return G_SOURCE_CONTINUE;
                c->Close(); // the client closed, or an error
            }
            // Close() destroyed this source; returning CONTINUE then is harmless.
            return G_SOURCE_CONTINUE;
        }

        static gboolean OnOut(gint, GIOCondition, gpointer data)
        {
            Client *c = static_cast<Client *>(data);
            if (c->Write())
            {
                Drop(c->out_watch_);
                c->t_.handler_.OnWritable(*c);
                return G_SOURCE_REMOVE; // already destroyed by Drop
            }
            return G_SOURCE_CONTINUE;
        }

        static gboolean Closed(gpointer data)
        {
            Client *c = static_cast<Client *>(data);
            g_source_unref(c->closed_source_);
            c->closed_source_ = nullptr;
            c->t_.handler_.OnClosed(*c);
            c->t_.Remove(c);
            return G_SOURCE_REMOVE;
        }
    };

    SocketTransport::SocketTransport(GMainContext *context, ConnectionHandler &handler, size_t max_clients,
                                     size_t cap_bytes)
        : context_(context), handler_(handler), max_clients_(max_clients), cap_bytes_(cap_bytes)
    {
    }

    SocketTransport::~SocketTransport() { CloseAll(); }

    bool SocketTransport::Listen(const std::string &dir, std::string &why)
    {
        if (dir.empty())
        {
            why = "neither $RUNTIME_DIRECTORY nor $XDG_RUNTIME_DIR is set";
            return false;
        }
        struct stat st;
        if (stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))
        {
            why = dir + " is not a directory";
            return false;
        }
        std::string path = dir + "/dlidar.sock";
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        if (path.size() >= sizeof addr.sun_path)
        {
            why = path + ": path too long";
            return false;
        }
        memcpy(addr.sun_path, path.c_str(), path.size() + 1);

        // A leftover: replaced if nobody answers on it.
        if (lstat(path.c_str(), &st) == 0)
        {
            if (!S_ISSOCK(st.st_mode))
            {
                why = path + " exists and is not a socket";
                return false;
            }
            int probe = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
            bool answers = probe >= 0 && connect(probe, (sockaddr *)&addr, sizeof addr) == 0;
            if (probe >= 0)
                close(probe);
            if (answers)
            {
                why = "another dLidar is running (it answers on " + path + ")";
                return false;
            }
            unlink(path.c_str());
        }

        int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0)
        {
            why = Errno("socket");
            return false;
        }
        // Mode 0600 before listen(): a connect before listen fails, so there is
        // no moment with the wrong mode.
        if (bind(fd, (sockaddr *)&addr, sizeof addr) != 0 || chmod(path.c_str(), 0600) != 0 ||
            stat(path.c_str(), &st) != 0 || listen(fd, 8) != 0)
        {
            why = Errno(path.c_str());
            close(fd);
            return false;
        }
        listen_fd_ = fd;
        path_ = path;
        dev_ = st.st_dev;
        ino_ = st.st_ino;
        accept_source_ = Watch(context_, fd, G_IO_IN, (GUnixFDSourceFunc)OnAccept, this);
        return true;
    }

    int SocketTransport::OnAccept(int fd, unsigned, void *data)
    {
        SocketTransport *t = static_cast<SocketTransport *>(data);
        for (;;)
        {
            int c = accept4(fd, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (c < 0)
                return G_SOURCE_CONTINUE; // EAGAIN, or a client that left already
            if (t->connections_.size() >= t->max_clients_)
            {
                std::string line = EncodeError(std::nullopt, kTooManyClients, "another client is connected");
                send(c, line.data(), line.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
                close(c);
                continue;
            }
            auto client = std::make_unique<Client>(*t, c);
            Client *raw = client.get();
            t->connections_[raw] = std::move(client);
            t->handler_.OnOpen(*raw);
        }
    }

    void SocketTransport::Remove(Client *c) { connections_.erase(c); }

    void SocketTransport::CloseAll()
    {
        for (auto &kv : connections_)
        {
            kv.second->Shut();
            handler_.OnClosed(*kv.first);
        }
        connections_.clear();
        Drop(accept_source_);
        if (listen_fd_ >= 0)
        {
            close(listen_fd_);
            listen_fd_ = -1;
            // Only our own file: another dLidar may have replaced it.
            struct stat st;
            if (lstat(path_.c_str(), &st) == 0 && st.st_dev == dev_ && st.st_ino == ino_)
                unlink(path_.c_str());
        }
    }
}
