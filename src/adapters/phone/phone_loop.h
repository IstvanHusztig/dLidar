#pragma once

#include "adapters/phone/phone_server.h"

#include <functional>
#include <thread>

typedef struct _GMainContext GMainContext;
typedef struct _GSource GSource;
typedef struct _GMainLoop GMainLoop;

namespace dlidar::phone
{
    // The phone link's own thread with a private GMainContext (design D4):
    // every transport and the server run on it. It is also the server's clock
    // (WakeAt is a GLib timeout on this context).
    class PhoneLoop : public ServerClock
    {
    public:
        PhoneLoop();
        ~PhoneLoop(); // Stop()

        PhoneLoop(const PhoneLoop &) = delete;
        PhoneLoop &operator=(const PhoneLoop &) = delete;

        void Start();
        // Quits and joins; pending invokes are dropped.
        void Stop();

        GMainContext *Context() const { return context_; }

        // Runs `fn` on the loop thread later. Thread-safe.
        void Invoke(std::function<void()> fn);
        // Runs `fn` on the loop thread and waits for it. Not from the loop thread.
        void Call(std::function<void()> fn);

        // ServerClock, on the loop thread. `on_wake` is what WakeAt calls.
        TimePoint Now() const override;
        void WakeAt(TimePoint t) override;
        std::function<void()> on_wake;

    private:
        GMainContext *context_;
        GMainLoop *loop_;
        std::thread thread_;
        GSource *timer_ = nullptr;
        TimePoint timer_at_{};

        static int Fired(void *self);
    };
}
