#include "adapters/livox_mid360s/live_livox_source.h"

#include "adapters/livox_mid360s/livox_conversion.h"

#include <livox_lidar_api.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace dlidar::livox
{
    namespace
    {
        // The SDK is one per process.
        std::atomic<bool> g_open{false};

        HostTime HostNow()
        {
            timespec m, r;
            clock_gettime(CLOCK_MONOTONIC, &m);
            clock_gettime(CLOCK_REALTIME, &r);
            return {(int64_t)m.tv_sec * 1000000000LL + m.tv_nsec, (int64_t)r.tv_sec * 1000000000LL + r.tv_nsec};
        }

        // No sensor found, or no packet at all, for this long: it is not there.
        // After power-on the Mid-360S needs a few seconds before it sends points.
        const std::chrono::seconds kFindTimeout(10);
        const std::chrono::seconds kPacketTimeout(10);
        const std::chrono::seconds kPointsTimeout(60);
        const std::chrono::seconds kCommandTimeout(2);
        const std::chrono::seconds kDrain(2);

        std::string SdkVersion()
        {
            LivoxLidarSdkVer v{};
            GetLivoxLidarSdkVer(&v);
            return std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.patch);
        }

        // The SDK's configuration file (its JSON format, key "Mid360s").
        std::string ConfigJson(const LiveLivoxConfig &c)
        {
            std::string host = "      {\n        \"host_ip\": \"" + c.host_ip + "\",\n";
            if (!c.lidar_ip.empty())
                host += "        \"lidar_ip\": [\"" + c.lidar_ip + "\"],\n";
            host += "        \"cmd_data_port\": " + std::to_string(c.cmd_data_port) + ",\n" +
                    "        \"push_msg_port\": " + std::to_string(c.push_msg_port) + ",\n" +
                    "        \"point_data_port\": " + std::to_string(c.point_data_port) + ",\n" +
                    "        \"imu_data_port\": " + std::to_string(c.imu_data_port) + ",\n" +
                    "        \"log_data_port\": " + std::to_string(c.log_data_port) + "\n      }\n";
            return "{\n  \"Mid360s\": {\n    \"lidar_net_info\": {\n      \"cmd_data_port\": 56100,\n"
                   "      \"push_msg_port\": 56200,\n      \"point_data_port\": 56300,\n      \"imu_data_port\": 56400,\n"
                   "      \"log_data_port\": 56500\n    },\n    \"host_net_info\": [\n" +
                   host + "    ]\n  }\n}\n";
        }

        bool SleepUnlessStopped(std::chrono::milliseconds d, const StopToken &stop)
        {
            auto until = std::chrono::steady_clock::now() + d;
            while (std::chrono::steady_clock::now() < until)
            {
                if (stop.StopRequested())
                    return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            return !stop.StopRequested();
        }
    }

    std::unique_ptr<LiveLivoxSource> LiveLivoxSource::Open(const LiveLivoxConfig &config)
    {
        if (g_open.exchange(true))
        {
            std::cerr << "Livox-SDK2 is already in use in this process." << std::endl;
            return nullptr;
        }
        std::unique_ptr<LiveLivoxSource> source(new LiveLivoxSource(config));
        const char *dir = getenv("XDG_RUNTIME_DIR");
        source->config_path_ = std::string(dir && *dir ? dir : "/tmp") + "/dlidar_livox_" + std::to_string(getpid()) + ".json";
        {
            std::ofstream f(source->config_path_);
            f << ConfigJson(config);
            if (!f)
            {
                std::cerr << "Cannot write the Livox SDK configuration " << source->config_path_ << std::endl;
                g_open = false;
                return nullptr;
            }
        }
        DisableLivoxSdkConsoleLogger();
        // Never a null path: LivoxLidarSdkInit(nullptr, host_ip) crashes on
        // device detection (Livox-SDK2 issue #135).
        if (!LivoxLidarSdkInit(source->config_path_.c_str(), config.host_ip.c_str()))
        {
            std::cerr << "Livox SDK initialization failed (host " << config.host_ip
                      << "); is that address on one of this computer's interfaces?" << std::endl;
            LivoxLidarSdkUninit();
            unlink(source->config_path_.c_str());
            g_open = false;
            return nullptr;
        }
        SetLivoxLidarPointCloudCallBack(&LiveLivoxSource::OnPacket, source.get());
        SetLivoxLidarImuDataCallback(&LiveLivoxSource::OnPacket, source.get());
        SetLivoxLidarInfoChangeCallback(&LiveLivoxSource::OnInfoChange, source.get());
        LivoxLidarSdkStart();
        source->info_.sdk_version = SdkVersion();
        std::cout << "Livox SDK " << source->info_.sdk_version << " started on " << config.host_ip << std::endl;
        return source;
    }

    LiveLivoxSource::~LiveLivoxSource()
    {
        // No callback runs after Uninit, so the pool and the pending answers
        // can go now.
        LivoxLidarSdkUninit();
        unlink(config_path_.c_str());
        g_open = false;
    }

    void LiveLivoxSource::OnPacket(uint32_t, uint8_t, LivoxLidarEthernetPacket *data, void *client_data)
    {
        HostTime now = HostNow(); // first, before anything that takes time
        if (!data)
            return;
        auto *self = static_cast<LiveLivoxSource *>(client_data);
        self->pool_.Push(now, reinterpret_cast<const uint8_t *>(data), data->length);
    }

    void LiveLivoxSource::OnInfoChange(uint32_t handle, const LivoxLidarInfo *info, void *client_data)
    {
        if (!info)
            return;
        auto *self = static_cast<LiveLivoxSource *>(client_data);
        std::lock_guard<std::mutex> lock(self->device_mutex_);
        if (self->found_)
            return;
        self->found_ = true;
        self->handle_ = handle;
        self->dev_type_ = info->dev_type;
        self->sn_.assign(info->sn, strnlen(info->sn, sizeof info->sn));
        self->found_ip_.assign(info->lidar_ip, strnlen(info->lidar_ip, sizeof info->lidar_ip));
    }

    void LiveLivoxSource::OnControl(livox_status status, uint32_t, LivoxLidarAsyncControlResponse *response, void *client_data)
    {
        auto *p = static_cast<Pending *>(client_data);
        std::lock_guard<std::mutex> lock(p->mutex);
        p->status = status;
        p->ret_code = response ? response->ret_code : 0xff;
        p->done = true;
        p->done_cv.notify_all();
    }

    void LiveLivoxSource::OnFirmware(livox_status status, uint32_t, LivoxLidarDiagInternalInfoResponse *response, void *client_data)
    {
        auto *p = static_cast<Pending *>(client_data);
        std::lock_guard<std::mutex> lock(p->mutex);
        p->status = status;
        p->ret_code = response ? response->ret_code : 0xff;
        if (status == kLivoxLidarStatusSuccess && response && response->ret_code == 0 && response->param_num > 0)
        {
            // Key-value list: key (u16), length (u16), value; kKeyVersionApp is 4 bytes.
            LivoxLidarKeyValueParam kv;
            memcpy(&kv, response->data, 4);
            if (kv.key == kKeyVersionApp && kv.length >= 4)
            {
                const uint8_t *v = response->data + 4;
                char text[32];
                snprintf(text, sizeof text, "%u.%u.%u.%u", v[0], v[1], v[2], v[3]);
                p->version = text;
            }
        }
        p->done = true;
        p->done_cv.notify_all();
    }

    LiveLivoxSource::Pending *LiveLivoxSource::NewPending()
    {
        pending_.push_back(std::make_unique<Pending>());
        return pending_.back().get();
    }

    bool LiveLivoxSource::Await(Pending *p, int send_status, const char *what)
    {
        bool ok = false;
        int status = send_status;
        if (send_status == kLivoxLidarStatusSuccess)
        {
            std::unique_lock<std::mutex> lock(p->mutex);
            p->done_cv.wait_for(lock, kCommandTimeout, [p] { return p->done; });
            status = p->done ? p->status : kLivoxLidarStatusTimeout;
            ok = p->done && p->status == kLivoxLidarStatusSuccess && p->ret_code == 0;
        }
        if (!ok)
            std::cerr << "WARNING: the Mid-360S did not confirm " << what << " (status " << status
                      << "); continuing with its current setting." << std::endl;
        return ok;
    }

    int LiveLivoxSource::Next(HostTime &arrival)
    {
        if (pool_.TakeGrowWarning())
            std::cerr << "WARNING: the Livox packet buffer grew past " << pool_.InitialSlots()
                      << " packets: the capture is behind the sensor." << std::endl;
        if (!pool_.Pop(std::chrono::milliseconds(100)))
            return kNothing;
        arrival = pool_.Arrival();
        return kPacket;
    }

    bool LiveLivoxSource::Prepare(const StopToken &stop)
    {
        using Clock = std::chrono::steady_clock;
        // 1. The sensor announces itself.
        std::cout << "searching for a Mid-360S on " << config_.host_ip
                  << (config_.lidar_ip.empty() ? std::string() : " at " + config_.lidar_ip) << " ..." << std::endl;
        Clock::time_point start = Clock::now();
        uint32_t handle = 0;
        uint8_t dev_type = 0;
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (found_)
                {
                    handle = handle_;
                    dev_type = dev_type_;
                    std::cout << "found Livox device type " << (int)dev_type_ << ", SN " << sn_ << ", at " << found_ip_
                              << std::endl;
                    break;
                }
            }
            if (stop.StopRequested())
                return true;
            if (Clock::now() - start >= kFindTimeout)
            {
                std::cerr << "No Mid-360S found within " << kFindTimeout.count()
                          << " s; check its power, the network and that this computer has " << config_.host_ip << "."
                          << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (dev_type == kLivoxLidarTypeMid360)
            std::cerr << "WARNING: this is a Mid-360, not a Mid-360S; it is recorded as livox_mid360s." << std::endl;
        else if (dev_type != kLivoxLidarTypeMid360s)
        {
            std::cerr << "The Livox device (type " << (int)dev_type << ") is not a Mid-360S." << std::endl;
            return false;
        }

        // 2. Best effort (Livox-SDK2 issue #125: commands can time out on a
        // Mid-360S while data flows; after power-on it samples, IMU on).
        Pending *p = NewPending();
        if (Await(p, SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, &OnControl, p), "work mode normal"))
            info_.work_mode = kLivoxLidarNormal;
        p = NewPending();
        Await(p, EnableLivoxLidarImuData(handle, &OnControl, p), "IMU data on");
        p = NewPending();
        Await(p, SetLivoxLidarPclDataType(handle, kLivoxLidarCartesianCoordinateHighData, &OnControl, p),
              "point data type Cartesian high");
        p = NewPending();
        if (Await(p, QueryLivoxLidarFirmwareVer(handle, &OnFirmware, p), "the firmware version query"))
        {
            std::lock_guard<std::mutex> lock(p->mutex);
            if (!p->version.empty())
                info_.lidar_firmware_version = p->version;
        }
        info_.lidar_hardware_version = "Mid-360S SN " + sn_;
        std::cout << "Mid-360S firmware " << info_.lidar_firmware_version << ", SDK " << info_.sdk_version << ", work mode "
                  << info_.work_mode << std::endl;
        if (stop.StopRequested())
            return true;

        // 3. Points arrive.
        start = Clock::now();
        Clock::time_point last_packet = start;
        int other_type = -1;
        while (true)
        {
            if (stop.StopRequested())
                return true;
            Clock::time_point now = Clock::now();
            if (now - last_packet >= kPacketTimeout || now - start >= kPointsTimeout)
            {
                std::cerr << "The Mid-360S sent "
                          << (now - last_packet >= kPacketTimeout ? std::string("no packet for ") + std::to_string(kPacketTimeout.count()) + " s"
                                                                  : std::string("no Cartesian point packet within ") +
                                                                        std::to_string(kPointsTimeout.count()) + " s")
                          << (other_type >= 0 ? " (point packets of data type " + std::to_string(other_type) +
                                                    "; set Cartesian high with Livox Viewer 2)"
                                              : std::string())
                          << "." << std::endl;
                return false;
            }
            if (!pool_.Pop(std::chrono::milliseconds(100)))
                continue;
            last_packet = Clock::now();
            if (CheckPacket(pool_.Data(), pool_.Size()) != PacketStatus::Ok)
                continue;
            uint8_t type = Header(pool_.Data()).data_type;
            if (type == kLivoxLidarCartesianCoordinateHighData)
                break;
            if (type != kLivoxLidarImuData)
                other_type = type;
        }

        // 4. The packets buffered during the start-up are not part of the recording.
        if (!SleepUnlessStopped(kDrain, stop))
            return true;
        size_t dropped = pool_.Clear();
        std::cout << "Mid-360S streaming; dropped " << dropped << " start-up packets." << std::endl;
        return true;
    }

    void LiveLivoxSource::Stop()
    {
        if (stopped_)
            return;
        stopped_ = true;
        uint32_t handle;
        {
            std::lock_guard<std::mutex> lock(device_mutex_);
            if (!found_)
                return;
            handle = handle_;
        }
        Pending *p = NewPending();
        Await(p, SetLivoxLidarWorkMode(handle, kLivoxLidarSleep, &OnControl, p), "work mode sleep");
    }
}
