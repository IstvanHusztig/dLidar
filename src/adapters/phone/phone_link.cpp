#include "adapters/phone/phone_link.h"

#include "adapters/phone/mailbox.h"
#include "adapters/phone/phone_loop.h"
#include "adapters/phone/phone_server.h"
#include "adapters/phone/socket_transport.h"

#include <thread>

namespace dlidar::phone
{
    struct PhoneLink::Impl
    {
        PhoneLoop loop;
        std::shared_ptr<Mailbox> mailbox;
        std::unique_ptr<PhoneServer> server;
        std::unique_ptr<SocketTransport> socket;
        std::shared_ptr<PhoneEvents> events;
        bool stopped = false;
    };

    PhoneLink::PhoneLink(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

    PhoneLink::~PhoneLink() { Shutdown(); }

    std::unique_ptr<PhoneLink> PhoneLink::Start(const DeviceApi &api, const std::string &socket_dir, std::string &why)
    {
        auto impl = std::make_unique<Impl>();
        Impl *p = impl.get();
        // The wake runs on the hub's thread, under the mailbox lock, so it
        // never runs after Mailbox::Close().
        p->mailbox = std::make_shared<Mailbox>([p] { p->loop.Invoke([p] { p->server->OnMailbox(); }); });
        p->server = std::make_unique<PhoneServer>(api, p->mailbox, p->loop);
        p->loop.on_wake = [p] { p->server->OnWake(); };
        p->socket = std::make_unique<SocketTransport>(p->loop.Context(), *p->server);
        p->loop.Start();
        bool ok = false;
        p->loop.Call([&] { ok = p->socket->Listen(socket_dir, why); });
        if (!ok)
        {
            p->loop.Stop();
            return nullptr;
        }
        p->events = std::make_shared<PhoneEvents>(p->mailbox);
        return std::unique_ptr<PhoneLink>(new PhoneLink(std::move(impl)));
    }

    std::shared_ptr<DeviceEvents> PhoneLink::Events() const { return impl_->events; }

    std::string PhoneLink::SocketPath() const { return impl_->socket->Path(); }

    size_t PhoneLink::Connections()
    {
        size_t n = 0;
        if (!impl_->stopped)
            impl_->loop.Call([&] { n = impl_->socket->Connections(); });
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
        p->loop.Call([p] { p->socket->CloseAll(); });
        p->loop.Stop();
    }
}
