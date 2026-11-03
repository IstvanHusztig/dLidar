#include "adapters/phone/bluez_peripheral.h"

#include <gio/gio.h>

#include <functional>
#include <iostream>
#include <set>
#include <vector>

namespace dlidar::phone
{
    namespace
    {
        // The protocol's GATT service (docs/phone-protocol.md, "Transports").
        const char *const kServiceUuid = "b06b0001-1294-4457-85c4-ef6a46d0c127";
        const char *const kRxUuid = "b06b0002-1294-4457-85c4-ef6a46d0c127";
        const char *const kTxUuid = "b06b0003-1294-4457-85c4-ef6a46d0c127";

        const char *const kAppPath = "/org/dlidar/phone";
        const char *const kServicePath = "/org/dlidar/phone/service0";
        const char *const kRxPath = "/org/dlidar/phone/service0/rx";
        const char *const kTxPath = "/org/dlidar/phone/service0/tx";
        const char *const kAdvPath = "/org/dlidar/phone/adv0";
        const char *const kAgentPath = "/org/dlidar/phone/agent";

        const char *const kBluez = "org.bluez";
        const char *const kChrc = "org.bluez.GattCharacteristic1";
        const char *const kProps = "org.freedesktop.DBus.Properties";
        const char *const kObjectManager = "org.freedesktop.DBus.ObjectManager";

        const char *const kIntrospection = R"XML(
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
    <property name='UUID' type='s' access='read'/>
    <property name='Service' type='o' access='read'/>
    <property name='Flags' type='as' access='read'/>
    <property name='Value' type='ay' access='read'/>
    <property name='Notifying' type='b' access='read'/>
  </interface>
  <interface name='org.bluez.LEAdvertisement1'>
    <method name='Release'/>
    <property name='Type' type='s' access='read'/>
    <property name='ServiceUUIDs' type='as' access='read'/>
    <property name='LocalName' type='s' access='read'/>
    <property name='Discoverable' type='b' access='read'/>
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

        bool Under(const std::string &path, const std::string &prefix)
        {
            return !prefix.empty() && path.size() > prefix.size() && path.compare(0, prefix.size(), prefix) == 0 &&
                   path[prefix.size()] == '/';
        }
    }

    struct BluezPeripheral::Impl
    {
        using Done = std::function<void(GVariant *result, GError *error)>;

        BluezPeripheral &self;
        GMainContext *context;
        Options options;

        GDBusConnection *bus = nullptr;
        GCancellable *cancel = g_cancellable_new();
        GDBusNodeInfo *node = nullptr;
        std::vector<guint> objects, signals;
        guint watch = 0;

        bool bluez_seen = false; // org.bluez appeared at least once
        std::string adapter;     // e.g. /org/bluez/hci0
        bool app_registered = false, app_pending = false;
        bool adv_want = false, adv_registered = false, adv_pending = false;
        std::string tx_value;

        // The pairing window.
        std::chrono::steady_clock::time_point window_until{};
        bool window_wanted = false, window_open = false, agent_registered = false;
        GSource *window_timer = nullptr;

        // One warning per cause.
        bool warned_bluez = false, warned_adapter = false, warned_register = false, warned_advertise = false,
             warned_guard = false, warned_off = false;
        bool powered = true;
        // bluetoothd makes a new adapter pairable itself (main.conf): switched
        // off quietly for a moment after the adapter is found.
        std::chrono::steady_clock::time_point adapter_found{};
        GSource *check_timer = nullptr; // the deferred adapter check
        std::set<GSource *> one_shots;  // After() timers still pending

        std::shared_ptr<int> stop_pending = std::make_shared<int>(0);

        Impl(BluezPeripheral &s, GMainContext *c, Options o) : self(s), context(c), options(std::move(o)) {}

        BleLink *Link() const { return self.link_; }

        static void Warn(bool &once, const std::string &text)
        {
            if (!once)
                std::cerr << "WARNING: " << text << std::endl;
            once = true;
        }

        // An asynchronous call to BlueZ. `done` runs on the loop unless the
        // peripheral was destroyed first (cancelled).
        void Call(const std::string &path, const char *iface, const char *method, GVariant *params, Done done)
        {
            struct Ctx
            {
                Impl *impl;
                Done done;
            };
            g_dbus_connection_call(
                bus, kBluez, path.c_str(), iface, method, params, nullptr, G_DBUS_CALL_FLAGS_NONE, 10000, cancel,
                [](GObject *src, GAsyncResult *res, gpointer data) {
                    Ctx *ctx = static_cast<Ctx *>(data);
                    GError *err = nullptr;
                    GVariant *r = g_dbus_connection_call_finish(G_DBUS_CONNECTION(src), res, &err);
                    if (!(err && g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED)) && ctx->done)
                        ctx->done(r, err);
                    if (r)
                        g_variant_unref(r);
                    if (err)
                        g_error_free(err);
                    delete ctx;
                },
                new Ctx{this, std::move(done)});
        }

        // For Stop(): not cancelled by the destructor; counted in stop_pending.
        void CallUncancelled(const std::string &path, const char *iface, const char *method, GVariant *params)
        {
            ++*stop_pending;
            g_dbus_connection_call(
                bus, kBluez, path.c_str(), iface, method, params, nullptr, G_DBUS_CALL_FLAGS_NONE, 300, nullptr,
                [](GObject *src, GAsyncResult *res, gpointer data) {
                    auto *pending = static_cast<std::shared_ptr<int> *>(data);
                    GVariant *r = g_dbus_connection_call_finish(G_DBUS_CONNECTION(src), res, nullptr);
                    if (r)
                        g_variant_unref(r);
                    --**pending;
                    delete pending;
                },
                new std::shared_ptr<int>(stop_pending));
        }

        void SetAdapter(const char *prop, GVariant *value, Done done = nullptr)
        {
            if (adapter.empty())
                return;
            Call(adapter, kProps, "Set", g_variant_new("(ssv)", "org.bluez.Adapter1", prop, value), std::move(done));
        }

        // --- bus, name watch, adapter -------------------------------------

        void OnBus(GDBusConnection *connection)
        {
            bus = connection;
            GError *err = nullptr;
            node = g_dbus_node_info_new_for_xml(kIntrospection, &err);
            if (!node)
            {
                std::cerr << "WARNING: Bluetooth: " << err->message << std::endl;
                g_error_free(err);
                return;
            }
            Export(kAppPath, kObjectManager);
            Export(kServicePath, "org.bluez.GattService1");
            Export(kRxPath, kChrc);
            Export(kTxPath, kChrc);
            Export(kAdvPath, "org.bluez.LEAdvertisement1");
            Export(kAgentPath, "org.bluez.Agent1");
            signals.push_back(g_dbus_connection_signal_subscribe(bus, kBluez, kProps, "PropertiesChanged", nullptr,
                                                                 nullptr, G_DBUS_SIGNAL_FLAGS_NONE, OnPropertiesChanged,
                                                                 this, nullptr));
            signals.push_back(g_dbus_connection_signal_subscribe(bus, kBluez, kObjectManager, "InterfacesAdded",
                                                                 nullptr, nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
                                                                 OnInterfacesAdded, this, nullptr));
            signals.push_back(g_dbus_connection_signal_subscribe(bus, kBluez, kObjectManager, "InterfacesRemoved",
                                                                 nullptr, nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
                                                                 OnInterfacesRemoved, this, nullptr));
            watch = g_bus_watch_name_on_connection(
                bus, kBluez, G_BUS_NAME_WATCHER_FLAGS_NONE,
                [](GDBusConnection *, const gchar *, const gchar *, gpointer d) { static_cast<Impl *>(d)->Appeared(); },
                [](GDBusConnection *, const gchar *, gpointer d) { static_cast<Impl *>(d)->Vanished(); }, this, nullptr);
        }

        void Export(const char *path, const char *iface)
        {
            static const GDBusInterfaceVTable vtable = {OnMethod, OnGetProperty, nullptr, {nullptr}};
            GError *err = nullptr;
            guint id = g_dbus_connection_register_object(bus, path, g_dbus_node_info_lookup_interface(node, iface),
                                                         &vtable, this, nullptr, &err);
            if (id)
                objects.push_back(id);
            else
            {
                std::cerr << "WARNING: Bluetooth: cannot export " << path << ": " << err->message << std::endl;
                g_error_free(err);
            }
        }

        void Appeared()
        {
            if (bluez_seen && warned_bluez)
                std::cout << "Bluetooth: bluetoothd is back" << std::endl;
            bluez_seen = true;
            warned_bluez = false;
            FindAdapter();
        }

        void Vanished()
        {
            Warn(warned_bluez, bluez_seen ? "Bluetooth (bluetoothd) is gone; the phone link waits for it"
                                          : "no Bluetooth (bluetoothd is not running); the phone link waits for it");
            Lost();
        }

        // An adapter that is removed or switched off while bluetoothd stays:
        // warned 1 s later, so a bluetoothd restart (off, removed, gone) gives
        // one warning, the one for bluetoothd.
        void CheckAdapterLater()
        {
            if (check_timer)
                return;
            check_timer = g_timeout_source_new(1000);
            g_source_set_callback(
                check_timer,
                [](gpointer data) -> gboolean {
                    Impl *impl = static_cast<Impl *>(data);
                    g_source_unref(impl->check_timer);
                    impl->check_timer = nullptr;
                    if (impl->warned_bluez)
                        return G_SOURCE_REMOVE; // bluetoothd itself is gone: warned for that
                    if (impl->adapter.empty())
                        Warn(impl->warned_adapter, "no Bluetooth adapter; the phone link waits for one");
                    else if (!impl->powered)
                        Warn(impl->warned_off, "Bluetooth is switched off; the phone link waits");
                    return G_SOURCE_REMOVE;
                },
                this, nullptr);
            g_source_attach(check_timer, context);
        }

        // BlueZ or the adapter went away: everything registered is gone too.
        void Lost()
        {
            adapter.clear();
            app_registered = app_pending = false;
            adv_registered = adv_pending = false;
            agent_registered = false;
            window_open = false; // reopened for the remaining time when BlueZ is back
            if (Link())
                Link()->BluezDown();
        }

        void FindAdapter()
        {
            Call("/", kObjectManager, "GetManagedObjects", nullptr, [this](GVariant *r, GError *err) {
                if (!r)
                {
                    Warn(warned_adapter, std::string("Bluetooth: ") + err->message);
                    return;
                }
                GVariantIter *it;
                const gchar *path;
                GVariant *ifaces;
                std::string found;
                bool powered = true;
                g_variant_get(r, "(a{oa{sa{sv}}})", &it);
                while (g_variant_iter_loop(it, "{&o@a{sa{sv}}}", &path, &ifaces))
                    if (GVariant *a = g_variant_lookup_value(ifaces, "org.bluez.Adapter1", nullptr))
                    {
                        if (found.empty())
                        {
                            found = path;
                            gboolean p = TRUE;
                            g_variant_lookup(a, "Powered", "b", &p);
                            powered = p;
                        }
                        g_variant_unref(a);
                    }
                g_variant_iter_free(it);
                if (found.empty())
                {
                    CheckAdapterLater();
                    return;
                }
                if (warned_adapter)
                    std::cout << "Bluetooth: adapter " << found << " found" << std::endl;
                warned_adapter = false;
                adapter = found;
                adapter_found = std::chrono::steady_clock::now();
                this->powered = powered;
                if (!powered)
                    CheckAdapterLater(); // BlueZ keeps our registrations; advertising resumes when it is on
                else if (warned_off)
                {
                    std::cout << "Bluetooth is on again" << std::endl;
                    warned_off = false;
                }
                // The devices already connected, before BluezUp (BleLink's contract).
                g_variant_get(r, "(a{oa{sa{sv}}})", &it);
                while (g_variant_iter_loop(it, "{&o@a{sa{sv}}}", &path, &ifaces))
                    if (Under(path, adapter))
                        if (GVariant *d = g_variant_lookup_value(ifaces, "org.bluez.Device1", nullptr))
                        {
                            gboolean connected = FALSE;
                            g_variant_lookup(d, "Connected", "b", &connected);
                            if (connected && Link())
                                Link()->DeviceConnected(path);
                            g_variant_unref(d);
                        }
                g_variant_iter_free(it);
                if (!window_wanted)
                    SetAdapter("Pairable", g_variant_new_boolean(FALSE)); // only the pairing window makes it pairable
                // The adapter's name is what a connected or bonded phone shows
                // (the GAP Device Name); without it, the hostname. BlueZ keeps it.
                SetAdapter("Alias", g_variant_new_string(options.name.c_str()), [](GVariant *res, GError *e) {
                    if (!res)
                        std::cerr << "WARNING: Bluetooth: cannot set the adapter's name (" << e->message << ")"
                                  << std::endl;
                });
                RegisterApplication();
                if (window_wanted)
                    StartWindow();
            });
        }

        void RegisterApplication()
        {
            if (app_registered || app_pending || adapter.empty())
                return;
            app_pending = true;
            Call(adapter, "org.bluez.GattManager1", "RegisterApplication",
                 g_variant_new("(oa{sv})", kAppPath, nullptr), [this](GVariant *r, GError *err) {
                     app_pending = false;
                     if (!r)
                     {
                         Warn(warned_register, std::string("Bluetooth: the phone service could not be registered (") +
                                                   err->message + ")");
                         return;
                     }
                     if (warned_register)
                         std::cout << "Bluetooth: the phone service is registered" << std::endl;
                     warned_register = false;
                     app_registered = true;
                     if (Link())
                         Link()->BluezUp(); // asks for the advertisement
                 });
        }

        // --- advertisement ------------------------------------------------

        void Advertise()
        {
            // Not while switched off: it would fail; retried when it is on.
            if (!app_registered || !powered || adv_pending || adv_want == adv_registered)
                return;
            adv_pending = true;
            bool on = adv_want;
            Call(adapter, "org.bluez.LEAdvertisingManager1", on ? "RegisterAdvertisement" : "UnregisterAdvertisement",
                 on ? g_variant_new("(oa{sv})", kAdvPath, nullptr) : g_variant_new("(o)", kAdvPath),
                 [this, on](GVariant *r, GError *err) {
                     adv_pending = false;
                     if (r)
                         adv_registered = on;
                     else if (on)
                         Warn(warned_advertise, std::string("Bluetooth: cannot advertise (") + err->message + ")");
                     else
                         adv_registered = false; // already gone
                     if (r && on && warned_advertise)
                     {
                         std::cout << "Bluetooth: advertising again" << std::endl;
                         warned_advertise = false;
                     }
                     if (r || !on)
                         Advertise(); // the wish may have changed meanwhile
                 });
        }

        // --- pairing window -----------------------------------------------

        void StartWindow()
        {
            if (window_open || adapter.empty())
                return;
            auto left = std::chrono::ceil<std::chrono::seconds>(window_until - std::chrono::steady_clock::now());
            if (left.count() <= 0)
                return;
            window_open = true; // before Pairable turns true: the guard must not fight it
            Call("/org/bluez", "org.bluez.AgentManager1", "RegisterAgent",
                 g_variant_new("(os)", kAgentPath, "NoInputNoOutput"), [this, left](GVariant *r, GError *err) {
                     if (!r)
                     {
                         std::cerr << "WARNING: Bluetooth pairing window: " << err->message << std::endl;
                         window_open = false;
                         return;
                     }
                     agent_registered = true;
                     Call("/org/bluez", "org.bluez.AgentManager1", "RequestDefaultAgent",
                          g_variant_new("(o)", kAgentPath), nullptr);
                     SetAdapter("PairableTimeout", g_variant_new_uint32(0));
                     MakePairable(left, 10);
                 });
        }

        // BlueZ makes the adapter pairable itself when our agent becomes the
        // default; a Set at the same moment gets org.bluez.Error.Busy. Then
        // try again shortly (up to `tries` times).
        void MakePairable(std::chrono::seconds left, int tries)
        {
            SetAdapter("Pairable", g_variant_new_boolean(TRUE), [this, left, tries](GVariant *res, GError *e) {
                if (res)
                {
                    std::cout << "Bluetooth pairing started: a phone can pair for the next " << left.count()
                              << " s (Just Works, no PIN)" << std::endl;
                    return;
                }
                bool busy = g_dbus_error_is_remote_error(e) &&
                            std::string(g_dbus_error_get_remote_error(e) ? g_dbus_error_get_remote_error(e) : "") ==
                                "org.bluez.Error.Busy";
                if (busy && tries > 1 && window_open)
                    After(200, [this, left, tries] {
                        if (window_open)
                            MakePairable(left, tries - 1);
                    });
                else
                    std::cerr << "WARNING: Bluetooth pairing window: " << e->message << std::endl;
            });
        }

        // Runs `fn` on the loop after `ms`, unless the peripheral is gone.
        void After(guint ms, std::function<void()> fn)
        {
            struct Ctx
            {
                Impl *impl;
                GSource *source;
                std::function<void()> fn;
            };
            GSource *s = g_timeout_source_new(ms);
            auto *ctx = new Ctx{this, s, std::move(fn)};
            g_source_set_callback(
                s,
                [](gpointer data) -> gboolean {
                    auto *c = static_cast<Ctx *>(data);
                    c->impl->one_shots.erase(c->source);
                    c->fn();
                    return G_SOURCE_REMOVE;
                },
                ctx, [](gpointer data) { delete static_cast<Ctx *>(data); });
            one_shots.insert(s);
            g_source_attach(s, context);
            g_source_unref(s);
        }

        void WindowExpired()
        {
            window_wanted = false;
            bool was_open = window_open;
            window_open = false;
            // The agent first, then Pairable false: BlueZ may make the adapter
            // pairable again when the default agent changes (the guard would
            // also catch it).
            auto pairable_off = [this](GVariant *, GError *) { SetAdapter("Pairable", g_variant_new_boolean(FALSE)); };
            if (agent_registered)
            {
                agent_registered = false;
                Call("/org/bluez", "org.bluez.AgentManager1", "UnregisterAgent", g_variant_new("(o)", kAgentPath),
                     pairable_off);
            }
            else
                pairable_off(nullptr, nullptr);
            if (was_open)
                std::cout << "Bluetooth pairing ended: no new phones can pair" << std::endl;
        }

        // --- signals --------------------------------------------------------

        static void OnPropertiesChanged(GDBusConnection *, const gchar *, const gchar *path, const gchar *,
                                        const gchar *, GVariant *params, gpointer data)
        {
            Impl *impl = static_cast<Impl *>(data);
            const gchar *iface;
            GVariant *changed, *invalidated;
            g_variant_get(params, "(&s@a{sv}@as)", &iface, &changed, &invalidated);
            std::string i = iface, p = path;
            gboolean b = FALSE;
            if (i == "org.bluez.Device1" && Under(p, impl->adapter))
            {
                if (g_variant_lookup(changed, "Connected", "b", &b) && impl->Link())
                {
                    if (b)
                        impl->Link()->DeviceConnected(p);
                    else
                        impl->Link()->DeviceDisconnected(p);
                }
                if (g_variant_lookup(changed, "Paired", "b", &b) && b)
                    impl->PrintPaired(p);
            }
            else if (i == "org.bluez.Adapter1" && p == impl->adapter)
            {
                if (g_variant_lookup(changed, "Pairable", "b", &b) && b && !impl->window_open)
                {
                    // The Pairable guard (design D8): only the pairing window
                    // may. Quiet right after the adapter appears: bluetoothd
                    // itself makes it pairable then.
                    if (std::chrono::steady_clock::now() - impl->adapter_found > std::chrono::seconds(2))
                        Warn(impl->warned_guard, "another program made Bluetooth pairable; switched off (pairing "
                                                 "only in the pairing window)");
                    impl->SetAdapter("Pairable", g_variant_new_boolean(FALSE));
                }
                if (g_variant_lookup(changed, "Powered", "b", &b))
                {
                    impl->powered = b;
                    if (!b)
                        impl->CheckAdapterLater();
                    else
                    {
                        if (impl->warned_off)
                            std::cout << "Bluetooth is on again" << std::endl;
                        impl->warned_off = false;
                        impl->Advertise(); // if it was wanted while off
                    }
                }
            }
            g_variant_unref(changed);
            g_variant_unref(invalidated);
        }

        static void OnInterfacesAdded(GDBusConnection *, const gchar *, const gchar *, const gchar *, const gchar *,
                                      GVariant *params, gpointer data)
        {
            Impl *impl = static_cast<Impl *>(data);
            const gchar *path;
            GVariant *ifaces;
            g_variant_get(params, "(&o@a{sa{sv}})", &path, &ifaces);
            if (impl->adapter.empty())
            {
                if (GVariant *a = g_variant_lookup_value(ifaces, "org.bluez.Adapter1", nullptr))
                {
                    g_variant_unref(a);
                    impl->FindAdapter();
                }
            }
            else if (Under(path, impl->adapter))
                if (GVariant *d = g_variant_lookup_value(ifaces, "org.bluez.Device1", nullptr))
                {
                    gboolean connected = FALSE;
                    g_variant_lookup(d, "Connected", "b", &connected);
                    if (connected && impl->Link())
                        impl->Link()->DeviceConnected(path);
                    g_variant_unref(d);
                }
            g_variant_unref(ifaces);
        }

        static void OnInterfacesRemoved(GDBusConnection *, const gchar *, const gchar *, const gchar *, const gchar *,
                                        GVariant *params, gpointer data)
        {
            Impl *impl = static_cast<Impl *>(data);
            const gchar *path;
            GVariant *ifaces;
            g_variant_get(params, "(&o@as)", &path, &ifaces);
            std::string p = path;
            if (p == impl->adapter)
            {
                impl->Lost();
                impl->CheckAdapterLater();
            }
            else if (Under(p, impl->adapter) && impl->Link())
                impl->Link()->DeviceDisconnected(p); // no-op unless it was connected
            g_variant_unref(ifaces);
        }

        void PrintPaired(const std::string &device)
        {
            Call(device, kProps, "Get", g_variant_new("(ss)", "org.bluez.Device1", "Alias"),
                 [device](GVariant *r, GError *) {
                     std::string name = device;
                     if (r)
                     {
                         GVariant *v;
                         g_variant_get(r, "(v)", &v);
                         name = g_variant_get_string(v, nullptr);
                         g_variant_unref(v);
                     }
                     std::cout << "Bluetooth: paired with " << name << std::endl;
                 });
        }

        // --- our objects ----------------------------------------------------

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
                g_variant_builder_add(&b, "{sv}", "Flags", Strv({"encrypt-write", "write-without-response"}));
            }
            else if (path == kTxPath)
            {
                g_variant_builder_add(&b, "{sv}", "UUID", g_variant_new_string(kTxUuid));
                g_variant_builder_add(&b, "{sv}", "Service", g_variant_new_object_path(kServicePath));
                g_variant_builder_add(&b, "{sv}", "Flags", Strv({"notify", "encrypt-notify"}));
                g_variant_builder_add(&b, "{sv}", "Value", Bytes(tx_value));
                g_variant_builder_add(&b, "{sv}", "Notifying", g_variant_new_boolean(Link() && Link()->Open()));
            }
            else if (path == kAdvPath)
            {
                g_variant_builder_add(&b, "{sv}", "Type", g_variant_new_string("peripheral"));
                g_variant_builder_add(&b, "{sv}", "ServiceUUIDs", Strv({kServiceUuid}));
                g_variant_builder_add(&b, "{sv}", "LocalName", g_variant_new_string(options.name.c_str()));
                g_variant_builder_add(&b, "{sv}", "Discoverable", g_variant_new_boolean(TRUE));
            }
            return g_variant_builder_end(&b);
        }

        static GVariant *OnGetProperty(GDBusConnection *, const gchar *, const gchar *path, const gchar *,
                                       const gchar *prop, GError **error, gpointer data)
        {
            GVariant *all = g_variant_ref_sink(static_cast<Impl *>(data)->Props(path));
            GVariant *v = g_variant_lookup_value(all, prop, nullptr);
            g_variant_unref(all);
            if (!v)
                g_set_error(error, G_DBUS_ERROR, G_DBUS_ERROR_UNKNOWN_PROPERTY, "no property %s", prop);
            return v;
        }

        static void OnMethod(GDBusConnection *, const gchar *, const gchar *path, const gchar *iface,
                             const gchar *method, GVariant *params, GDBusMethodInvocation *inv, gpointer data)
        {
            Impl *impl = static_cast<Impl *>(data);
            std::string p = path, i = iface, m = method;
            if (m == "GetManagedObjects")
            {
                GVariantBuilder b;
                g_variant_builder_init(&b, G_VARIANT_TYPE("a{oa{sa{sv}}}"));
                const std::pair<const char *, const char *> objects[] = {
                    {kServicePath, "org.bluez.GattService1"}, {kRxPath, kChrc}, {kTxPath, kChrc}};
                for (const auto &o : objects)
                {
                    GVariantBuilder ifaces;
                    g_variant_builder_init(&ifaces, G_VARIANT_TYPE("a{sa{sv}}"));
                    g_variant_builder_add(&ifaces, "{s@a{sv}}", o.second, impl->Props(o.first));
                    g_variant_builder_add(&b, "{oa{sa{sv}}}", o.first, &ifaces);
                }
                g_dbus_method_invocation_return_value(inv, g_variant_new("(a{oa{sa{sv}}})", &b));
            }
            else if (m == "WriteValue" && p == kRxPath)
                impl->OnWrite(params, inv);
            else if (m == "ReadValue")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(@ay)", Bytes(impl->tx_value)));
            else if ((m == "StartNotify" || m == "StopNotify") && p == kTxPath)
            {
                g_dbus_method_invocation_return_value(inv, nullptr);
                if (impl->Link())
                {
                    if (m == "StartNotify")
                        impl->Link()->NotifyStarted();
                    else
                        impl->Link()->NotifyStopped();
                }
            }
            else if (i == "org.bluez.LEAdvertisement1") // Release: BlueZ dropped it
            {
                impl->adv_registered = false;
                g_dbus_method_invocation_return_value(inv, nullptr);
            }
            else if (i == "org.bluez.Agent1")
                impl->OnAgent(m, inv);
            else
                g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.NotSupported", "not supported");
        }

        void OnWrite(GVariant *params, GDBusMethodInvocation *inv)
        {
            GVariant *value = nullptr, *opts = nullptr;
            g_variant_get(params, "(@ay@a{sv})", &value, &opts);
            gsize n = 0;
            const char *bytes = static_cast<const char *>(g_variant_get_fixed_array(value, &n, 1));
            const char *device = "";
            guint16 mtu = 0, offset = 0;
            g_variant_lookup(opts, "device", "&o", &device);
            g_variant_lookup(opts, "mtu", "q", &mtu);
            g_variant_lookup(opts, "offset", "q", &offset);
            WriteResult r = Link() ? Link()->Write(device, std::string_view(bytes, n), mtu, offset)
                                   : WriteResult::NotPermitted;
            g_variant_unref(value);
            g_variant_unref(opts);
            if (r == WriteResult::Ok)
                g_dbus_method_invocation_return_value(inv, nullptr);
            else if (r == WriteResult::InvalidOffset)
                g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.InvalidOffset", "no long writes");
            else
                g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.NotPermitted", "not permitted");
        }

        // Just Works inside the window, nothing outside it.
        void OnAgent(const std::string &m, GDBusMethodInvocation *inv)
        {
            if (m == "Release" || m == "Cancel")
                g_dbus_method_invocation_return_value(inv, nullptr);
            else if (!window_open)
                g_dbus_method_invocation_return_dbus_error(inv, "org.bluez.Error.Rejected", "pairing window closed");
            else if (m == "RequestPinCode")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(s)", "000000"));
            else if (m == "RequestPasskey")
                g_dbus_method_invocation_return_value(inv, g_variant_new("(u)", 0u));
            else
                g_dbus_method_invocation_return_value(inv, nullptr);
        }
    };

    BluezPeripheral::BluezPeripheral(GMainContext *context, Options options)
        : impl_(std::make_unique<Impl>(*this, context, std::move(options)))
    {
    }

    BluezPeripheral::~BluezPeripheral()
    {
        Impl &d = *impl_;
        g_cancellable_cancel(d.cancel);
        for (GSource *t : d.one_shots)
            g_source_destroy(t); // their Ctx is freed by the destroy notify
        for (GSource *t : {d.window_timer, d.check_timer})
            if (t)
            {
                g_source_destroy(t);
                g_source_unref(t);
            }
        if (d.bus)
        {
            if (d.watch)
                g_bus_unwatch_name(d.watch);
            for (guint s : d.signals)
                g_dbus_connection_signal_unsubscribe(d.bus, s);
            for (guint o : d.objects)
                g_dbus_connection_unregister_object(d.bus, o);
            g_object_unref(d.bus);
        }
        if (d.node)
            g_dbus_node_info_unref(d.node);
        g_object_unref(d.cancel);
    }

    void BluezPeripheral::Start()
    {
        g_bus_get(
            G_BUS_TYPE_SYSTEM, impl_->cancel,
            [](GObject *, GAsyncResult *res, gpointer data) {
                GError *err = nullptr;
                GDBusConnection *bus = g_bus_get_finish(res, &err);
                if (!bus)
                {
                    if (!g_error_matches(err, G_IO_ERROR, G_IO_ERROR_CANCELLED))
                        std::cerr << "WARNING: no Bluetooth (system bus: " << err->message << ")" << std::endl;
                    g_error_free(err);
                    return;
                }
                static_cast<Impl *>(data)->OnBus(bus);
            },
            impl_.get());
    }

    void BluezPeripheral::OpenPairingWindow(std::chrono::seconds seconds)
    {
        Impl &d = *impl_;
        d.window_wanted = true;
        d.window_until = std::chrono::steady_clock::now() + seconds;
        if (d.window_timer)
        {
            g_source_destroy(d.window_timer);
            g_source_unref(d.window_timer);
        }
        d.window_timer = g_timeout_source_new_seconds((guint)seconds.count());
        g_source_set_callback(
            d.window_timer,
            [](gpointer data) -> gboolean {
                Impl *impl = static_cast<Impl *>(data);
                g_source_unref(impl->window_timer);
                impl->window_timer = nullptr;
                impl->WindowExpired();
                return G_SOURCE_REMOVE;
            },
            impl_.get(), nullptr);
        g_source_attach(d.window_timer, d.context);
        d.StartWindow(); // or when the adapter is found
    }

    void BluezPeripheral::Stop()
    {
        Impl &d = *impl_;
        if (!d.bus || d.adapter.empty())
            return;
        if (d.adv_registered)
            d.CallUncancelled(d.adapter, "org.bluez.LEAdvertisingManager1", "UnregisterAdvertisement",
                              g_variant_new("(o)", kAdvPath));
        if (d.app_registered)
            d.CallUncancelled(d.adapter, "org.bluez.GattManager1", "UnregisterApplication",
                              g_variant_new("(o)", kAppPath));
        if (d.agent_registered)
            d.CallUncancelled("/org/bluez", "org.bluez.AgentManager1", "UnregisterAgent",
                              g_variant_new("(o)", kAgentPath));
        d.CallUncancelled(d.adapter, kProps, "Set",
                          g_variant_new("(ssv)", "org.bluez.Adapter1", "Pairable", g_variant_new_boolean(FALSE)));
        d.adv_registered = d.app_registered = d.agent_registered = false;
        d.window_open = d.window_wanted = false;
    }

    bool BluezPeripheral::StopDone() const { return *impl_->stop_pending == 0; }

    void BluezPeripheral::Notify(const std::string &piece)
    {
        Impl &d = *impl_;
        if (!d.bus || !d.app_registered)
            return;
        d.tx_value = piece;
        GVariantBuilder b;
        g_variant_builder_init(&b, G_VARIANT_TYPE("a{sv}"));
        g_variant_builder_add(&b, "{sv}", "Value", Bytes(piece));
        g_dbus_connection_emit_signal(d.bus, nullptr, kTxPath, kProps, "PropertiesChanged",
                                      g_variant_new("(sa{sv}@as)", kChrc, &b, g_variant_new_strv(nullptr, 0)),
                                      nullptr);
    }

    void BluezPeripheral::Disconnect(const std::string &device)
    {
        if (impl_->bus)
            impl_->Call(device, "org.bluez.Device1", "Disconnect", nullptr, nullptr);
    }

    void BluezPeripheral::SetAdvertising(bool on)
    {
        impl_->adv_want = on;
        impl_->Advertise();
    }
}
