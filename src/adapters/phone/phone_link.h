#pragma once

#include "application/ports/device_events.h"
#include "application/remote/device_api.h"

#include <chrono>
#include <memory>
#include <string>

namespace dlidar::phone
{
    // The phone link as the composition root sees it (design D6): the loop
    // thread, the protocol server and the local socket. Events() goes to
    // hub.AddAsync. A problem only prints a warning; dLidar records as usual.
    class PhoneLink
    {
    public:
        // Listens in `socket_dir` (SocketDirectory() in production). Null with
        // `why` set if the socket cannot be used.
        static std::unique_ptr<PhoneLink> Start(const DeviceApi &api, const std::string &socket_dir, std::string &why);
        ~PhoneLink(); // Shutdown()

        std::shared_ptr<DeviceEvents> Events() const;
        std::string SocketPath() const;
        size_t Connections();

        // Delivers what is pending, the final idle or error state included,
        // for up to `flush`, then closes the connections, removes the socket
        // and stops the thread. Idempotent.
        void Shutdown(std::chrono::milliseconds flush = std::chrono::milliseconds(200));

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        explicit PhoneLink(std::unique_ptr<Impl> impl);
    };
}
