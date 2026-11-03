#pragma once

#include "adapters/phone/connection.h"

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <sys/types.h>

typedef struct _GMainContext GMainContext;
typedef struct _GSource GSource;

namespace dlidar::phone
{
    // The directory for dlidar.sock: $RUNTIME_DIRECTORY (systemd's
    // RuntimeDirectory=, the first entry), else $XDG_RUNTIME_DIR, else empty
    // (no socket; never /tmp, where another user could take the path).
    std::string SocketDirectory();

    // Clients on a Unix stream socket (design D4, D6), on the loop thread
    // (`context`): non-blocking, a write buffer per connection capped at
    // `cap_bytes` (past it the connection is closed), at most `max_clients`;
    // one more gets too_many_clients and is closed.
    class SocketTransport
    {
    public:
        static const size_t kMaxClients = 4;
        static const size_t kCapBytes = 64 * 1024;

        SocketTransport(GMainContext *context, ConnectionHandler &handler, size_t max_clients = kMaxClients,
                        size_t cap_bytes = kCapBytes);
        ~SocketTransport(); // CloseAll(), on the loop thread

        // `dir`/dlidar.sock: a leftover file nobody answers on is replaced;
        // if another dLidar answers, or anything fails, false with `why`.
        bool Listen(const std::string &dir, std::string &why);
        const std::string &Path() const { return path_; }

        // Closes every connection and the listening socket, removes the file.
        void CloseAll();

        size_t Connections() const { return connections_.size(); }
        // Listen failed because another dLidar answers on the socket.
        bool AnotherRunning() const { return another_running_; }

    private:
        class Client;

        GMainContext *context_;
        ConnectionHandler &handler_;
        size_t max_clients_, cap_bytes_;
        std::string path_;
        int listen_fd_ = -1;
        bool another_running_ = false;
        dev_t dev_ = 0;
        ino_t ino_ = 0;
        GSource *accept_source_ = nullptr;
        std::map<Client *, std::unique_ptr<Client>> connections_;

        static int OnAccept(int fd, unsigned condition, void *self);
        void Remove(Client *c);
    };
}
