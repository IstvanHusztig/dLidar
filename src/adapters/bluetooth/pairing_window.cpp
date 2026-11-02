#include "adapters/bluetooth/pairing_window.h"

#include <gio/gio.h>

#include <future>
#include <iostream>
#include <string>
#include <thread>

namespace dlidar::bluetooth
{
    namespace
    {
        const char *const kAgentPath = "/org/dlidar/pairing_agent";
        const char *const kAgentXml = R"XML(<node>
  <interface name='org.bluez.Agent1'>
    <method name='Release'/>
    <method name='RequestPinCode'><arg type='o' direction='in'/><arg type='s' direction='out'/></method>
    <method name='DisplayPinCode'><arg type='o' direction='in'/><arg type='s' direction='in'/></method>
    <method name='RequestPasskey'><arg type='o' direction='in'/><arg type='u' direction='out'/></method>
    <method name='DisplayPasskey'>
      <arg type='o' direction='in'/><arg type='u' direction='in'/><arg type='q' direction='in'/>
    </method>
    <method name='RequestConfirmation'><arg type='o' direction='in'/><arg type='u' direction='in'/></method>
    <method name='RequestAuthorization'><arg type='o' direction='in'/></method>
    <method name='AuthorizeService'><arg type='o' direction='in'/><arg type='s' direction='in'/></method>
    <method name='Cancel'/>
  </interface>
</node>)XML";
    }

    struct PairingWindow::Impl
    {
        std::thread thread;
        GMainContext *context = nullptr;
        GMainLoop *loop = nullptr;
        GDBusConnection *bus = nullptr;
        GDBusNodeInfo *node = nullptr;
        std::string adapter;
        guint agent_id = 0, subscription = 0, timer = 0;

        // BlueZ call on the loop thread; null with a warning on failure.
        GVariant *Call(const char *path, const char *iface, const char *method, GVariant *args, bool warn = true)
        {
            GError *err = nullptr;
            GVariant *r = g_dbus_connection_call_sync(bus, "org.bluez", path, iface, method, args, nullptr,
                                                      G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
            if (!r)
            {
                if (warn)
                    std::cerr << "WARNING: Bluetooth " << method << " failed: " << err->message << std::endl;
                g_error_free(err);
            }
            return r;
        }

        bool SetAdapter(const char *prop, GVariant *value)
        {
            GVariant *r = Call(adapter.c_str(), "org.freedesktop.DBus.Properties", "Set",
                               g_variant_new("(ssv)", "org.bluez.Adapter1", prop, value));
            if (r)
                g_variant_unref(r);
            return r != nullptr;
        }

        std::string FindAdapter()
        {
            GVariant *r = Call("/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr);
            if (!r)
                return "";
            std::string found;
            GVariantIter *objects;
            const gchar *path;
            GVariant *ifaces;
            g_variant_get(r, "(a{oa{sa{sv}}})", &objects);
            while (found.empty() && g_variant_iter_loop(objects, "{&o@a{sa{sv}}}", &path, &ifaces))
                if (GVariant *a = g_variant_lookup_value(ifaces, "org.bluez.Adapter1", nullptr))
                {
                    found = path;
                    g_variant_unref(a);
                }
            g_variant_iter_free(objects);
            g_variant_unref(r);
            return found;
        }

        static void AgentCall(GDBusConnection *, const gchar *, const gchar *, const gchar *, const gchar *method,
                              GVariant *, GDBusMethodInvocation *inv, gpointer)
        {
            // Just Works: accept every request; there is no display or keyboard.
            std::string m = method;
            if (m == "RequestPinCode")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(s)", "000000"));
            else if (m == "RequestPasskey")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(u)", 0u));
            else
                g_dbus_method_invocation_return_value(inv, nullptr);
        }

        // Device1 PropertiesChanged: report a new bond.
        static void DeviceChanged(GDBusConnection *, const gchar *, const gchar *path, const gchar *, const gchar *,
                                  GVariant *params, gpointer data)
        {
            Impl *self = static_cast<Impl *>(data);
            const gchar *iface;
            GVariant *changed, *invalidated;
            g_variant_get(params, "(&s@a{sv}@as)", &iface, &changed, &invalidated);
            gboolean paired = FALSE;
            if (std::string(iface) == "org.bluez.Device1" && g_variant_lookup(changed, "Paired", "b", &paired) &&
                paired)
            {
                std::string name = path;
                GVariant *r = self->Call(path, "org.freedesktop.DBus.Properties", "Get",
                                         g_variant_new("(ss)", "org.bluez.Device1", "Alias"), false);
                if (r)
                {
                    GVariant *v;
                    g_variant_get(r, "(v)", &v);
                    name = g_variant_get_string(v, nullptr);
                    g_variant_unref(v);
                    g_variant_unref(r);
                }
                std::cout << "Bluetooth: paired with " << name << std::endl;
            }
            g_variant_unref(changed);
            g_variant_unref(invalidated);
        }

        static gboolean Expired(gpointer data)
        {
            Impl *self = static_cast<Impl *>(data);
            self->timer = 0;
            self->Close();
            std::cout << "Bluetooth pairing ended: no new phones can pair" << std::endl;
            return G_SOURCE_REMOVE;
        }

        // On the loop thread.
        bool Start(int seconds)
        {
            GError *err = nullptr;
            bus = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &err);
            if (!bus)
            {
                std::cerr << "WARNING: no Bluetooth pairing window (system bus: " << err->message << ")" << std::endl;
                g_error_free(err);
                return false;
            }
            adapter = FindAdapter();
            if (adapter.empty())
            {
                std::cerr << "WARNING: no Bluetooth pairing window (no adapter; is bluetooth.service running?)"
                          << std::endl;
                return false;
            }
            node = g_dbus_node_info_new_for_xml(kAgentXml, nullptr);
            static const GDBusInterfaceVTable vtable = {AgentCall, nullptr, nullptr, {nullptr}};
            agent_id = g_dbus_connection_register_object(bus, kAgentPath, node->interfaces[0], &vtable, this,
                                                         nullptr, &err);
            if (!agent_id)
            {
                std::cerr << "WARNING: no Bluetooth pairing window (agent: " << err->message << ")" << std::endl;
                g_error_free(err);
                return false;
            }
            GVariant *r = Call("/org/bluez", "org.bluez.AgentManager1", "RegisterAgent",
                               g_variant_new("(os)", kAgentPath, "NoInputNoOutput"));
            if (!r)
                return false;
            g_variant_unref(r);
            r = Call("/org/bluez", "org.bluez.AgentManager1", "RequestDefaultAgent", g_variant_new("(o)", kAgentPath));
            if (!r)
                return false;
            g_variant_unref(r);
            subscription = g_dbus_connection_signal_subscribe(bus, "org.bluez", "org.freedesktop.DBus.Properties",
                                                              "PropertiesChanged", nullptr, "org.bluez.Device1",
                                                              G_DBUS_SIGNAL_FLAGS_NONE, DeviceChanged, this, nullptr);
            if (!SetAdapter("Powered", g_variant_new_boolean(TRUE)) ||
                !SetAdapter("PairableTimeout", g_variant_new_uint32(0)) ||
                !SetAdapter("Pairable", g_variant_new_boolean(TRUE)))
                return false;
            GSource *source = g_timeout_source_new_seconds(seconds);
            g_source_set_callback(source, Expired, this, nullptr);
            timer = g_source_attach(source, context);
            g_source_unref(source);
            return true;
        }

        // On the loop thread: not pairable, agent gone, loop ends (when it runs).
        void Close()
        {
            if (timer)
                g_source_destroy(g_main_context_find_source_by_id(context, timer));
            timer = 0;
            if (agent_id)
            {
                GVariant *r = Call("/org/bluez", "org.bluez.AgentManager1", "UnregisterAgent",
                                   g_variant_new("(o)", kAgentPath), false);
                if (r)
                    g_variant_unref(r);
                g_dbus_connection_unregister_object(bus, agent_id);
                agent_id = 0;
            }
            // After the agent is gone: BlueZ may make the adapter pairable
            // again when the default agent changes.
            if (bus && !adapter.empty())
                SetAdapter("Pairable", g_variant_new_boolean(FALSE));
            if (subscription)
                g_dbus_connection_signal_unsubscribe(bus, subscription);
            subscription = 0;
            g_main_loop_quit(loop);
        }

        void Run(int seconds, std::promise<bool> &started)
        {
            g_main_context_push_thread_default(context);
            bool ok = Start(seconds);
            if (ok)
                std::cout << "Bluetooth pairing started: a phone can pair for the next " << seconds
                          << " s (Just Works, no PIN)" << std::endl;
            else
                Close();
            started.set_value(ok);
            if (ok)
                g_main_loop_run(loop);
            if (bus)
                g_object_unref(bus);
            bus = nullptr;
            if (node)
                g_dbus_node_info_unref(node);
            node = nullptr;
            g_main_context_pop_thread_default(context);
        }
    };

    PairingWindow::PairingWindow() : impl_(new Impl) {}

    PairingWindow::~PairingWindow()
    {
        if (impl_->thread.joinable())
        {
            g_main_context_invoke(
                impl_->context,
                [](gpointer data) {
                    static_cast<Impl *>(data)->Close();
                    return G_SOURCE_REMOVE;
                },
                impl_.get());
            impl_->thread.join();
        }
        if (impl_->loop)
            g_main_loop_unref(impl_->loop);
        if (impl_->context)
            g_main_context_unref(impl_->context);
    }

    bool PairingWindow::Open(int seconds)
    {
        // A window still open, or one that ended: start a fresh one.
        if (impl_->thread.joinable())
        {
            g_main_context_invoke(
                impl_->context,
                [](gpointer data) {
                    static_cast<Impl *>(data)->Close();
                    return G_SOURCE_REMOVE;
                },
                impl_.get());
            impl_->thread.join();
        }
        if (!impl_->context)
        {
            impl_->context = g_main_context_new();
            impl_->loop = g_main_loop_new(impl_->context, FALSE);
        }
        std::promise<bool> started;
        std::future<bool> result = started.get_future();
        impl_->thread = std::thread([this, seconds, &started] { impl_->Run(seconds, started); });
        return result.get();
    }
}
