// Throwaway BLE spike for openspec/changes/phone-link-core (design D1, tasks
// 1.1 and 1.2). Not part of dLidar: a GATT "line echo" service on BlueZ's
// D-Bus API, used to measure MTU, notifications to several centrals,
// reconnects, bonding and Wi-Fi switches before the phone protocol is frozen.
//
//   bin/ble_spike [--acquire] [--secure] [--name NAME]
//
//   --acquire  TX notifications through AcquireNotify (a socket per
//              subscription) instead of PropertiesChanged on Value
//   --secure   RX and TX need an encrypted link (encrypt-write, encrypt-notify); registers a
//              NoInputNoOutput agent that accepts every pairing (Just Works)
//
// Write a line ending in "\n" to RX; it comes back as "echo <line>\n" on TX.
// "info\n" answers with the spike's state, "flood N SIZE\n" sends N numbered
// messages of SIZE bytes. Every message starts at a fresh notification and is
// split into MTU-3 byte pieces. See tools/ble_spike/README.md.
#include <gio/gio.h>
#include <gio/gunixfdlist.h>
#include <glib-unix.h>

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <deque>
#include <map>
#include <string>

namespace
{
    const char *kServiceUuid = "b06b0001-1294-4457-85c4-ef6a46d0c127";
    const char *kRxUuid = "b06b0002-1294-4457-85c4-ef6a46d0c127";
    const char *kTxUuid = "b06b0003-1294-4457-85c4-ef6a46d0c127";
    const char *kAppPath = "/org/dlidar/spike";
    const char *kServicePath = "/org/dlidar/spike/service0";
    const char *kRxPath = "/org/dlidar/spike/service0/rx";
    const char *kTxPath = "/org/dlidar/spike/service0/tx";
    const char *kAdvPath = "/org/dlidar/spike/adv0";
    const char *kAgentPath = "/org/dlidar/spike/agent";
    const char *kGattChrc = "org.bluez.GattCharacteristic1";

    const char *kIntrospection = R"XML(
<node>
  <interface name='org.freedesktop.DBus.ObjectManager'>
    <method name='GetManagedObjects'><arg type='a{oa{sa{sv}}}' direction='out'/></method>
  </interface>
  <interface name='org.bluez.GattService1'>
    <property name='UUID' type='s' access='read'/>
    <property name='Primary' type='b' access='read'/>
  </interface>
  <interface name='org.bluez.GattCharacteristic1'>
    <method name='ReadValue'><arg type='a{sv}' direction='in'/><arg type='ay' direction='out'/></method>
    <method name='WriteValue'><arg type='ay' direction='in'/><arg type='a{sv}' direction='in'/></method>
    <method name='StartNotify'/>
    <method name='StopNotify'/>
    <method name='AcquireNotify'>
      <arg type='a{sv}' direction='in'/><arg type='h' direction='out'/><arg type='q' direction='out'/>
    </method>
    <property name='UUID' type='s' access='read'/>
    <property name='Service' type='o' access='read'/>
    <property name='Flags' type='as' access='read'/>
    <property name='Value' type='ay' access='read'/>
    <property name='Notifying' type='b' access='read'/>
    <property name='NotifyAcquired' type='b' access='read'/>
  </interface>
  <interface name='org.bluez.LEAdvertisement1'>
    <method name='Release'/>
    <property name='Type' type='s' access='read'/>
    <property name='ServiceUUIDs' type='as' access='read'/>
    <property name='LocalName' type='s' access='read'/>
  </interface>
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

    struct Spike
    {
        bool acquire = false;
        bool secure = false;
        std::string name = "dLidar";

        GDBusConnection *bus = nullptr;
        GDBusNodeInfo *node = nullptr;
        GMainLoop *loop = nullptr;
        gint64 t0 = 0;

        std::string adapter;     // e.g. /org/bluez/hci0
        uint16_t mtu = 23;       // last MTU BlueZ reported
        bool notifying = false;  // StartNotify seen (PropertiesChanged mode)
        int notify_fd = -1;      // our end of the AcquireNotify socket
        std::string tx_value;    // last notification piece
        std::map<std::string, std::string> lines; // partial line per device

        std::deque<std::string> pieces; // notifications waiting to go out
        guint drain_source = 0;
        int flood_left = 0;
        gint64 flood_start = 0;
        uint64_t flood_bytes = 0;
    };

    Spike g;

    void Log(const char *fmt, ...)
    {
        va_list ap;
        va_start(ap, fmt);
        printf("%9.3f ", (g_get_monotonic_time() - g.t0) / 1e6);
        vprintf(fmt, ap);
        putchar('\n');
        fflush(stdout);
        va_end(ap);
    }

    GVariant *Strv(std::initializer_list<const char *> items)
    {
        GVariantBuilder b;
        g_variant_builder_init(&b, G_VARIANT_TYPE("as"));
        for (const char *s : items)
            g_variant_builder_add(&b, "s", s);
        return g_variant_builder_end(&b);
    }

    GVariant *Bytes(const std::string &s)
    {
        return g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, s.data(), s.size(), 1);
    }

    // The properties of one exported object, a floating a{sv}.
    GVariant *Props(const std::string &path)
    {
        GVariantBuilder b;
        g_variant_builder_init(&b, G_VARIANT_TYPE("a{sv}"));
        if (path == kServicePath)
        {
            g_variant_builder_add(&b, "{sv}", "UUID", g_variant_new_string(kServiceUuid));
            g_variant_builder_add(&b, "{sv}", "Primary", g_variant_new_boolean(TRUE));
        }
        else if (path == kRxPath)
        {
            g_variant_builder_add(&b, "{sv}", "UUID", g_variant_new_string(kRxUuid));
            g_variant_builder_add(&b, "{sv}", "Service", g_variant_new_object_path(kServicePath));
            g_variant_builder_add(&b, "{sv}", "Flags",
                                  g.secure ? Strv({"encrypt-write", "write-without-response"})
                                           : Strv({"write", "write-without-response"}));
        }
        else if (path == kTxPath)
        {
            g_variant_builder_add(&b, "{sv}", "UUID", g_variant_new_string(kTxUuid));
            g_variant_builder_add(&b, "{sv}", "Service", g_variant_new_object_path(kServicePath));
            g_variant_builder_add(&b, "{sv}", "Flags", g.secure ? Strv({"notify", "encrypt-notify"}) : Strv({"notify"}));
            g_variant_builder_add(&b, "{sv}", "Value", Bytes(g.tx_value));
            g_variant_builder_add(&b, "{sv}", "Notifying", g_variant_new_boolean(g.notifying));
            // BlueZ uses AcquireNotify when this property exists.
            if (g.acquire)
                g_variant_builder_add(&b, "{sv}", "NotifyAcquired", g_variant_new_boolean(g.notify_fd >= 0));
        }
        else if (path == kAdvPath)
        {
            g_variant_builder_add(&b, "{sv}", "Type", g_variant_new_string("peripheral"));
            g_variant_builder_add(&b, "{sv}", "ServiceUUIDs", Strv({kServiceUuid}));
            g_variant_builder_add(&b, "{sv}", "LocalName", g_variant_new_string(g.name.c_str()));
        }
        return g_variant_builder_end(&b);
    }

    void EmitTxChanged(const char *prop, GVariant *value)
    {
        GVariantBuilder b;
        g_variant_builder_init(&b, G_VARIANT_TYPE("a{sv}"));
        g_variant_builder_add(&b, "{sv}", prop, value);
        GError *err = nullptr;
        g_dbus_connection_emit_signal(g.bus, nullptr, kTxPath, "org.freedesktop.DBus.Properties", "PropertiesChanged",
                                      g_variant_new("(sa{sv}@as)", kGattChrc, &b, g_variant_new_strv(nullptr, 0)),
                                      &err);
        if (err)
        {
            Log("PropertiesChanged failed: %s", err->message);
            g_error_free(err);
        }
    }

    // Sends up to 32 pieces; false when the queue is empty.
    gboolean Drain(gpointer)
    {
        for (int n = 0; n < 32 && !g.pieces.empty(); n++)
        {
            const std::string &p = g.pieces.front();
            if (g.acquire)
            {
                if (g.notify_fd < 0)
                {
                    Log("no AcquireNotify socket: %zu pieces dropped", g.pieces.size());
                    g.pieces.clear();
                    break;
                }
                ssize_t w = send(g.notify_fd, p.data(), p.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
                if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                    return G_SOURCE_CONTINUE; // BlueZ is behind: retry on the next tick
                if (w < 0)
                {
                    Log("notify socket send failed: %s", strerror(errno));
                    g.pieces.clear();
                    break;
                }
            }
            else
            {
                g.tx_value = p;
                EmitTxChanged("Value", Bytes(p));
            }
            g.flood_bytes += p.size();
            g.pieces.pop_front();
        }
        if (!g.pieces.empty())
            return G_SOURCE_CONTINUE;
        if (g.flood_left > 0)
        {
            double s = (g_get_monotonic_time() - g.flood_start) / 1e6;
            Log("flood: %d messages, %llu bytes handed to BlueZ in %.3f s (%.0f B/s)", g.flood_left,
                (unsigned long long)g.flood_bytes, s, s > 0 ? g.flood_bytes / s : 0.0);
            g.flood_left = 0;
        }
        g.drain_source = 0;
        return G_SOURCE_REMOVE;
    }

    // Queues one message: MTU-3 byte pieces, the first piece fresh.
    void Send(const std::string &msg)
    {
        size_t piece = g.mtu > 3 ? g.mtu - 3 : 20;
        for (size_t i = 0; i < msg.size(); i += piece)
            g.pieces.push_back(msg.substr(i, piece));
        if (!g.drain_source)
            g.drain_source = g_timeout_add(2, Drain, nullptr);
    }

    void HandleLine(const std::string &device, const std::string &line)
    {
        Log("line from %s (%zu bytes): %.60s", device.c_str(), line.size(), line.c_str());
        unsigned n = 0, size = 0;
        if (line == "info")
        {
            char buf[256];
            snprintf(buf, sizeof buf, "info mode=%s secure=%d mtu=%u notifying=%d notify_fd=%d devices=%zu\n",
                     g.acquire ? "acquire" : "properties", g.secure, g.mtu, g.notifying, g.notify_fd, g.lines.size());
            Send(buf);
        }
        else if (sscanf(line.c_str(), "flood %u %u", &n, &size) == 2 && n > 0 && n <= 10000 && size >= 12 &&
                 size <= 4096)
        {
            g.flood_left = n;
            g.flood_start = g_get_monotonic_time();
            g.flood_bytes = 0;
            for (unsigned i = 1; i <= n; i++)
            {
                char head[16];
                snprintf(head, sizeof head, "seq=%05u ", i);
                std::string msg = head;
                msg.resize(size - 1, 'x');
                Send(msg + "\n");
            }
            Log("flood: queued %u messages of %u bytes at MTU %u", n, size, g.mtu);
        }
        else
            Send("echo " + line + "\n");
    }

    void OnWrite(GVariant *params)
    {
        GVariant *value = nullptr, *options = nullptr;
        g_variant_get(params, "(@ay@a{sv})", &value, &options);
        gsize n = 0;
        const char *data = (const char *)g_variant_get_fixed_array(value, &n, 1);
        const char *device = "?", *type = "?";
        guint16 mtu = 0, offset = 0;
        g_variant_lookup(options, "device", "&o", &device);
        g_variant_lookup(options, "type", "&s", &type);
        g_variant_lookup(options, "mtu", "q", &mtu);
        g_variant_lookup(options, "offset", "q", &offset);
        Log("WriteValue %zu bytes from %s type=%s mtu=%u offset=%u", n, device, type, mtu, offset);
        if (mtu)
            g.mtu = mtu;
        std::string &buf = g.lines[device];
        buf.append(data, n);
        for (size_t pos; (pos = buf.find('\n')) != std::string::npos;)
        {
            std::string line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            HandleLine(device, line);
        }
        g_variant_unref(value);
        g_variant_unref(options);
    }

    gboolean OnNotifyHangup(gint fd, GIOCondition, gpointer)
    {
        Log("AcquireNotify socket closed by BlueZ (unsubscribed or disconnected)");
        close(fd);
        g.notify_fd = -1;
        EmitTxChanged("NotifyAcquired", g_variant_new_boolean(FALSE));
        return G_SOURCE_REMOVE;
    }

    void OnAcquireNotify(GVariant *params, GDBusMethodInvocation *inv)
    {
        GVariant *options = nullptr;
        g_variant_get(params, "(@a{sv})", &options);
        const char *device = "?";
        guint16 mtu = 0;
        g_variant_lookup(options, "device", "&o", &device);
        g_variant_lookup(options, "mtu", "q", &mtu);
        Log("AcquireNotify from %s mtu=%u (socket already open: %s)", device, mtu, g.notify_fd >= 0 ? "yes" : "no");
        g_variant_unref(options);
        if (mtu)
            g.mtu = mtu;
        if (g.notify_fd >= 0)
        {
            g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.NotPermitted", "already acquired");
            return;
        }
        int sv[2];
        if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0, sv) < 0)
        {
            g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.Failed", strerror(errno));
            return;
        }
        GUnixFDList *fds = g_unix_fd_list_new();
        g_unix_fd_list_append(fds, sv[1], nullptr);
        close(sv[1]);
        g.notify_fd = sv[0];
        g_unix_fd_add(g.notify_fd, (GIOCondition)(G_IO_HUP | G_IO_ERR), OnNotifyHangup, nullptr);
        g_dbus_method_invocation_return_value_with_unix_fd_list(inv, g_variant_new("(hq)", 0, g.mtu), fds);
        g_object_unref(fds);
        EmitTxChanged("NotifyAcquired", g_variant_new_boolean(TRUE));
    }

    void MethodCall(GDBusConnection *, const gchar *sender, const gchar *path, const gchar *iface, const gchar *method,
                    GVariant *params, GDBusMethodInvocation *inv, gpointer)
    {
        std::string p = path, m = method, i = iface;
        if (m == "GetManagedObjects")
        {
            GVariantBuilder b;
            g_variant_builder_init(&b, G_VARIANT_TYPE("a{oa{sa{sv}}}"));
            const std::pair<const char *, const char *> objects[] = {
                {kServicePath, "org.bluez.GattService1"}, {kRxPath, kGattChrc}, {kTxPath, kGattChrc}};
            for (const auto &o : objects)
            {
                GVariantBuilder ifaces;
                g_variant_builder_init(&ifaces, G_VARIANT_TYPE("a{sa{sv}}"));
                g_variant_builder_add(&ifaces, "{s@a{sv}}", o.second, Props(o.first));
                g_variant_builder_add(&b, "{oa{sa{sv}}}", o.first, &ifaces);
            }
            Log("GetManagedObjects from %s", sender);
            g_dbus_method_invocation_return_value(inv, g_variant_new("(a{oa{sa{sv}}})", &b));
        }
        else if (m == "WriteValue")
        {
            OnWrite(params);
            g_dbus_method_invocation_return_value(inv, nullptr);
        }
        else if (m == "ReadValue")
            g_dbus_method_invocation_return_value(inv, g_variant_new("(@ay)", Bytes(g.tx_value)));
        else if (m == "StartNotify" || m == "StopNotify")
        {
            g.notifying = m == "StartNotify";
            Log("%s on %s", method, path);
            EmitTxChanged("Notifying", g_variant_new_boolean(g.notifying));
            g_dbus_method_invocation_return_value(inv, nullptr);
        }
        else if (m == "AcquireNotify")
            OnAcquireNotify(params, inv);
        else if (i == "org.bluez.Agent1")
        {
            gchar *args = g_variant_print(params, FALSE);
            Log("agent %s %s: accepted", method, args);
            g_free(args);
            if (m == "RequestPinCode")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(s)", "000000"));
            else if (m == "RequestPasskey")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(u)", 0u));
            else
                g_dbus_method_invocation_return_value(inv, nullptr);
        }
        else
        {
            Log("%s.%s on %s", iface, method, path);
            g_dbus_method_invocation_return_value(inv, nullptr);
        }
    }

    GVariant *GetProperty(GDBusConnection *, const gchar *, const gchar *path, const gchar *, const gchar *prop,
                          GError **error, gpointer)
    {
        GVariant *all = g_variant_ref_sink(Props(path));
        GVariant *v = g_variant_lookup_value(all, prop, nullptr);
        g_variant_unref(all);
        if (!v)
            g_set_error(error, G_DBUS_ERROR, G_DBUS_ERROR_UNKNOWN_PROPERTY, "no property %s", prop);
        return v;
    }

    const GDBusInterfaceVTable kVTable = {MethodCall, GetProperty, nullptr, {nullptr}};

    void Export(const char *path, const char *iface)
    {
        GError *err = nullptr;
        if (!g_dbus_connection_register_object(g.bus, path, g_dbus_node_info_lookup_interface(g.node, iface),
                                               &kVTable, nullptr, nullptr, &err))
        {
            Log("cannot export %s %s: %s", path, iface, err->message);
            exit(1);
        }
    }

    GVariant *Call(const char *path, const char *iface, const char *method, GVariant *args)
    {
        GError *err = nullptr;
        GVariant *r = g_dbus_connection_call_sync(g.bus, "org.bluez", path, iface, method, args, nullptr,
                                                  G_DBUS_CALL_FLAGS_NONE, 5000, nullptr, &err);
        if (!r)
        {
            Log("%s.%s failed: %s", iface, method, err->message);
            g_error_free(err);
        }
        return r;
    }

    void SetAdapterBool(const char *prop, bool value)
    {
        GVariant *r = Call(g.adapter.c_str(), "org.freedesktop.DBus.Properties", "Set",
                           g_variant_new("(ssv)", "org.bluez.Adapter1", prop, g_variant_new_boolean(value)));
        if (r)
            g_variant_unref(r);
    }

    std::string FindAdapter()
    {
        GVariant *r = Call("/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr);
        if (!r)
            return "";
        std::string found;
        GVariantIter *objects;
        const char *path;
        GVariant *ifaces;
        g_variant_get(r, "(a{oa{sa{sv}}})", &objects);
        while (g_variant_iter_next(objects, "{&o@a{sa{sv}}}", &path, &ifaces))
        {
            GVariant *gm = g_variant_lookup_value(ifaces, "org.bluez.GattManager1", nullptr);
            if (gm && found.empty())
                found = path;
            if (gm)
                g_variant_unref(gm);
            g_variant_unref(ifaces);
        }
        g_variant_iter_free(objects);
        g_variant_unref(r);
        return found;
    }

    void Registered(GObject *source, GAsyncResult *res, gpointer what)
    {
        GError *err = nullptr;
        GVariant *r = g_dbus_connection_call_finish(G_DBUS_CONNECTION(source), res, &err);
        if (r)
        {
            Log("%s registered", (const char *)what);
            g_variant_unref(r);
        }
        else
        {
            Log("%s registration failed: %s", (const char *)what, err->message);
            g_error_free(err);
        }
    }

    // Async: BlueZ calls GetManagedObjects and reads the advertisement back
    // while it handles these, so the main loop must keep running.
    void RegisterAsync(const char *iface, const char *method, const char *path, const char *what)
    {
        g_dbus_connection_call(g.bus, "org.bluez", g.adapter.c_str(), iface, method,
                               g_variant_new("(oa{sv})", path, nullptr), nullptr, G_DBUS_CALL_FLAGS_NONE, 10000,
                               nullptr, Registered, (gpointer)what);
    }

    void BluezAppeared(GDBusConnection *, const gchar *, const gchar *owner, gpointer)
    {
        g.adapter = FindAdapter();
        if (g.adapter.empty())
        {
            Log("org.bluez (%s) has no adapter with GattManager1; is the controller up?", owner);
            return;
        }
        Log("org.bluez appeared (%s), adapter %s, mode %s%s, name \"%s\"", owner, g.adapter.c_str(),
            g.acquire ? "AcquireNotify" : "PropertiesChanged", g.secure ? ", secure" : "", g.name.c_str());
        SetAdapterBool("Powered", true);
        if (g.secure)
        {
            SetAdapterBool("Pairable", true);
            GVariant *r = Call("/org/bluez", "org.bluez.AgentManager1", "RegisterAgent",
                               g_variant_new("(os)", kAgentPath, "NoInputNoOutput"));
            if (r)
                g_variant_unref(r);
            r = Call("/org/bluez", "org.bluez.AgentManager1", "RequestDefaultAgent", g_variant_new("(o)", kAgentPath));
            if (r)
                g_variant_unref(r);
        }
        RegisterAsync("org.bluez.GattManager1", "RegisterApplication", kAppPath, "GATT application");
        RegisterAsync("org.bluez.LEAdvertisingManager1", "RegisterAdvertisement", kAdvPath, "advertisement");
    }

    void BluezVanished(GDBusConnection *, const gchar *, gpointer)
    {
        Log("org.bluez is gone (bluetoothd stopped); waiting for it to come back");
        g.notifying = false;
        g.lines.clear();
    }

    // Connected/Paired/Bonded/ServicesResolved of every device, as BlueZ reports them.
    void DeviceChanged(GDBusConnection *, const gchar *, const gchar *path, const gchar *, const gchar *,
                       GVariant *params, gpointer)
    {
        GVariant *changed = nullptr;
        g_variant_get(params, "(&s@a{sv}@as)", nullptr, &changed, nullptr);
        gchar *text = g_variant_print(changed, FALSE);
        Log("device %s %s", path, text);
        g_free(text);
        gboolean connected;
        if (g_variant_lookup(changed, "Connected", "b", &connected) && !connected)
            g.lines.erase(path);
        g_variant_unref(changed);
    }

    gboolean Quit(gpointer)
    {
        Log("stopping");
        g_main_loop_quit(g.loop);
        return G_SOURCE_REMOVE;
    }
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        if (a == "--acquire")
            g.acquire = true;
        else if (a == "--secure")
            g.secure = true;
        else if (a == "--name" && i + 1 < argc)
            g.name = argv[++i];
        else
        {
            fprintf(stderr, "usage: %s [--acquire] [--secure] [--name NAME]\n", argv[0]);
            return 2;
        }
    }
    // Flags (3) + 128-bit UUID (18) + name (2 + n) must fit 31 bytes.
    if (g.name.size() > 8)
        fprintf(stderr, "warning: a name over 8 characters does not fit the advertisement next to the UUID\n");

    g.t0 = g_get_monotonic_time();
    GError *err = nullptr;
    g.bus = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &err);
    if (!g.bus)
    {
        fprintf(stderr, "no system bus: %s\n", err->message);
        return 1;
    }
    g.node = g_dbus_node_info_new_for_xml(kIntrospection, &err);
    if (!g.node)
    {
        fprintf(stderr, "introspection: %s\n", err->message);
        return 1;
    }
    Export(kAppPath, "org.freedesktop.DBus.ObjectManager");
    Export(kServicePath, "org.bluez.GattService1");
    Export(kRxPath, kGattChrc);
    Export(kTxPath, kGattChrc);
    Export(kAdvPath, "org.bluez.LEAdvertisement1");
    if (g.secure)
        Export(kAgentPath, "org.bluez.Agent1");

    g_dbus_connection_signal_subscribe(g.bus, "org.bluez", "org.freedesktop.DBus.Properties", "PropertiesChanged",
                                       nullptr, "org.bluez.Device1", G_DBUS_SIGNAL_FLAGS_NONE, DeviceChanged, nullptr,
                                       nullptr);
    g_bus_watch_name_on_connection(g.bus, "org.bluez", G_BUS_NAME_WATCHER_FLAGS_NONE, BluezAppeared, BluezVanished,
                                   nullptr, nullptr);
    g.loop = g_main_loop_new(nullptr, FALSE);
    g_unix_signal_add(SIGINT, Quit, nullptr);
    g_unix_signal_add(SIGTERM, Quit, nullptr);
    Log("ble_spike running; waiting for org.bluez");
    g_main_loop_run(g.loop);
    // BlueZ drops the application and the advertisement when we leave the bus.
    return 0;
}
