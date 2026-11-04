#include "adapters/phone/phone_link.h"

#include "adapters/phone/ble_link.h"
#include "adapters/phone/bluez_peripheral.h"
#include "adapters/phone/mailbox.h"
#include "adapters/phone/phone_loop.h"
#include "adapters/phone/phone_server.h"
#include "adapters/phone/socket_transport.h"

#include <iostream>
#include <mutex>
#include <thread>

namespace dlidar::phone
{
    namespace
    {
        // Brings Wi-Fi results from the adapter's thread to the loop until
        // the link shuts down; shared, so a late result never reaches a
        // stopped or destroyed loop.
        struct PostGate
        {
            std::mutex mutex;
            PhoneLoop *loop = nullptr;
            bool open = true;
        };
    }

    struct PhoneLink::Impl
    {
        PhoneLoop loop;
        std::shared_ptr<PostGate> gate = std::make_shared<PostGate>();
        std::shared_ptr<Mailbox> mailbox;
        std::unique_ptr<PhoneServer> server;
        std::unique_ptr<SocketTransport> socket;
        bool socket_ok = false;
        std::unique_ptr<BluezPeripheral> peripheral; // BLE (null when off)
        std::unique_ptr<BleLink> ble;
        std::shared_ptr<PhoneEvents> events;
        bool stopped = false;
    };

    PhoneLink::PhoneLink(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

    PhoneLink::~PhoneLink() { Shutdown(); }

    std::unique_ptr<PhoneLink> PhoneLink::Start(const DeviceApi &api, const std::string &socket_dir,
                                                const BleOptions &ble, std::string &socket_why,
                                                WifiSetup *wifi, LogReader *log)
    {
        auto impl = std::make_unique<Impl>();
        Impl *p = impl.get();
        p->gate->loop = &p->loop;
        PhoneServer::Post post = [gate = p->gate](std::function<void()> fn) {
            std::lock_guard<std::mutex> lock(gate->mutex);
            if (gate->open)
                gate->loop->Invoke(std::move(fn));
        };
        // The wake runs on the hub's thread, under the mailbox lock, so it
        // never runs after Mailbox::Close().
        p->mailbox = std::make_shared<Mailbox>([p] { p->loop.Invoke([p] { p->server->OnMailbox(); }); });
        p->server = std::make_unique<PhoneServer>(api, p->mailbox, p->loop, wifi, std::move(post), log);
        p->loop.on_wake = [p] {
            p->server->OnWake();
            if (p->ble)
                p->ble->OnWake();
        };
        p->socket = std::make_unique<SocketTransport>(p->loop.Context(), *p->server);
        p->loop.Start();
        p->loop.Call([&] { p->socket_ok = p->socket->Listen(socket_dir, socket_why); });
        if ((!p->socket_ok && (p->socket->AnotherRunning() || !ble.enabled)))
        {
            p->gate->open = false;
            p->loop.Stop();
            return nullptr;
        }
        if (ble.enabled)
            p->loop.Call([&] {
                BluezPeripheral::Options o;
                o.name = ble.name;
                o.on_pairing = ble.on_pairing;
                p->peripheral = std::make_unique<BluezPeripheral>(p->loop.Context(), o);
                p->ble = std::make_unique<BleLink>(*p->server, *p->peripheral, p->loop);
                p->peripheral->Attach(*p->ble);
                p->peripheral->Start(); // asynchronous: never waits for bluetoothd
            });
        p->events = std::make_shared<PhoneEvents>(p->mailbox);
        return std::unique_ptr<PhoneLink>(new PhoneLink(std::move(impl)));
    }

    std::shared_ptr<DeviceEvents> PhoneLink::Events() const { return impl_->events; }

    std::string PhoneLink::SocketPath() const { return impl_->socket->Path(); }

    void PhoneLink::OpenPairingWindow(bool forget, std::chrono::seconds seconds)
    {
        Impl *p = impl_.get();
        if (p->stopped)
            return;
        if (!p->peripheral)
        {
            std::cerr << "WARNING: no pairing window" << (forget ? " and no phones forgotten" : "")
                      << ": Bluetooth is off ([phone] ble = false)" << std::endl;
            return;
        }
        p->loop.Invoke([p, forget, seconds] {
            if (!p->peripheral)
                return;
            if (forget)
                p->peripheral->ForgetPhones(seconds);
            else
                p->peripheral->OpenPairingWindow(seconds);
        });
    }

    size_t PhoneLink::Connections()
    {
        size_t n = 0;
        if (!impl_->stopped)
            impl_->loop.Call([&] { n = impl_->socket->Connections() + (impl_->ble && impl_->ble->Open() ? 1 : 0); });
        return n;
    }

    void PhoneLink::Shutdown(std::chrono::milliseconds flush)
    {
        Impl *p = impl_.get();
        if (p->stopped)
            return;
        p->stopped = true;
        auto deadline = std::chrono::steady_clock::now() + flush;
        for (;;)
        {
            bool done = false;
            p->loop.Call([&] {
                DeviceState s = p->server->LastStatus().state;
                done = p->server->Drained() && (s == DeviceState::Idle || s == DeviceState::Error);
            });
            if (done || std::chrono::steady_clock::now() >= deadline)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        p->mailbox->Close();
        {
            std::lock_guard<std::mutex> lock(p->gate->mutex);
            p->gate->open = false;
        }
        if (p->peripheral)
        {
            // Best effort: BlueZ drops our objects at exit anyway.
            p->loop.Call([p] { p->peripheral->Stop(); });
            auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
            for (bool done = false; !done && std::chrono::steady_clock::now() < stop_deadline;)
            {
                p->loop.Call([&] { done = p->peripheral->StopDone(); });
                if (!done)
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        p->loop.Call([p] {
            p->ble.reset(); // OnClosed to the server
            p->peripheral.reset();
            p->socket->CloseAll();
        });
        p->loop.Stop();
    }
}
