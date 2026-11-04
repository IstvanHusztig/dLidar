#pragma once

#include <cstddef>
#include <string>
#include <string_view>

namespace dlidar::phone
{
    // One client connection of a transport (the Unix socket, later BLE). All
    // calls, in both directions, happen on the phone loop's thread.
    class Connection
    {
    public:
        virtual ~Connection() = default;

        // Queues bytes for the client. A connection whose unsent bytes pass
        // its cap is closed (a client that does not read).
        virtual void Send(const std::string &bytes) = 0;

        // Bytes queued but not yet taken by the client.
        virtual size_t Pending() const = 0;

        // Closes the connection; OnClosed follows later on the loop.
        virtual void Close() = 0;

        // What kind of client it is, for log lines ("phone", "local client").
        virtual const char *Kind() const { return "client"; }
    };

    // What a transport tells the protocol server, on the loop's thread.
    class ConnectionHandler
    {
    public:
        virtual ~ConnectionHandler() = default;
        virtual void OnOpen(Connection &c) = 0;
        virtual void OnBytes(Connection &c, std::string_view data) = 0;
        // The client took the queued bytes (Pending() dropped to 0).
        virtual void OnWritable(Connection &c) = 0;
        // After this the connection object is gone.
        virtual void OnClosed(Connection &c) = 0;
    };
}
