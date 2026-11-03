#include "adapters/phone/phone_loop.h"

#include <glib.h>

#include <future>

namespace dlidar::phone
{
    namespace
    {
        gboolean RunOnce(gpointer data)
        {
            (*static_cast<std::function<void()> *>(data))();
            return G_SOURCE_REMOVE;
        }

        void Free(gpointer data) { delete static_cast<std::function<void()> *>(data); }
    }

    PhoneLoop::PhoneLoop() : context_(g_main_context_new()), loop_(g_main_loop_new(context_, FALSE)) {}

    PhoneLoop::~PhoneLoop()
    {
        Stop();
        g_main_loop_unref(loop_);
        g_main_context_unref(context_);
    }

    void PhoneLoop::Start()
    {
        thread_ = std::thread([this] {
            g_main_context_push_thread_default(context_);
            g_main_loop_run(loop_);
            if (timer_)
            {
                g_source_destroy(timer_);
                g_source_unref(timer_);
                timer_ = nullptr;
            }
            g_main_context_pop_thread_default(context_);
        });
    }

    void PhoneLoop::Stop()
    {
        if (!thread_.joinable())
            return;
        Invoke([this] { g_main_loop_quit(loop_); });
        thread_.join();
    }

    void PhoneLoop::Invoke(std::function<void()> fn)
    {
        // An idle source on our context: never runs inline on the caller.
        GSource *s = g_idle_source_new();
        g_source_set_priority(s, G_PRIORITY_DEFAULT);
        g_source_set_callback(s, RunOnce, new std::function<void()>(std::move(fn)), Free);
        g_source_attach(s, context_);
        g_source_unref(s);
    }

    void PhoneLoop::Call(std::function<void()> fn)
    {
        std::promise<void> done;
        Invoke([&] {
            fn();
            done.set_value();
        });
        done.get_future().wait();
    }

    ServerClock::TimePoint PhoneLoop::Now() const { return std::chrono::steady_clock::now(); }

    void PhoneLoop::WakeAt(TimePoint t)
    {
        if (timer_ && timer_at_ <= t)
            return; // an earlier wake already comes
        if (timer_)
        {
            g_source_destroy(timer_);
            g_source_unref(timer_);
        }
        auto ms = std::chrono::ceil<std::chrono::milliseconds>(t - Now()).count();
        timer_at_ = t;
        timer_ = g_timeout_source_new(ms > 0 ? (guint)ms : 0);
        g_source_set_callback(timer_, (GSourceFunc)Fired, this, nullptr);
        g_source_attach(timer_, context_);
    }

    int PhoneLoop::Fired(void *data)
    {
        PhoneLoop *self = static_cast<PhoneLoop *>(data);
        g_source_unref(self->timer_);
        self->timer_ = nullptr;
        if (self->on_wake)
            self->on_wake();
        return G_SOURCE_REMOVE;
    }
}
