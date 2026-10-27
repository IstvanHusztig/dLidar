#pragma once

#include "adapters/livox_mid360s/livox_packet_pool.h"
#include "adapters/livox_mid360s/livox_packet_source.h"

#include <livox_lidar_def.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

namespace dlidar::livox
{
    struct LiveLivoxConfig
    {
        std::string lidar_ip; // empty: the first Mid-360S found
        std::string host_ip;
        unsigned short cmd_data_port = 56101;
        unsigned short push_msg_port = 56201;
        unsigned short point_data_port = 56301;
        unsigned short imu_data_port = 56401;
        unsigned short log_data_port = 56501;
        // The IMU setting sent at every start-up (SetLivoxLidarImuRange).
        int imu_rate_hz = 500;            // 50, 100, 200, 500
        int imu_accel_range_g = 8;        // 4, 8, 16, 32
        double imu_gyro_range_dps = 2000; // 2000 ... 15.625
    };

    // The Mid-360S over UDP through Livox-SDK2 (the only user of its library).
    // The SDK delivers packets in callbacks on its own data thread; they go
    // through a LivoxPacketPool to Next() on the capture thread. The SDK is a
    // process-wide singleton, so only one LiveLivoxSource can be open.
    class LiveLivoxSource : public LivoxPacketSource
    {
    public:
        // Writes the SDK configuration, starts the SDK and registers the
        // callbacks; nullptr (after printing why) if that fails.
        static std::unique_ptr<LiveLivoxSource> Open(const LiveLivoxConfig &config);
        ~LiveLivoxSource() override;

        int Next(HostTime &arrival) override;
        const uint8_t *Data() const override { return pool_.Data(); }
        size_t Size() const override { return pool_.Size(); }

        // Waits for the sensor, sets work mode, IMU and point data type and
        // queries the firmware (each best effort: a command without an answer
        // is a warning), waits for points, then drops the backlog for 2 s. A
        // stop ends it early (true); false if no Mid-360S is found within 10 s,
        // no packet arrives for 10 s or no Cartesian point packet within 60 s.
        bool Prepare(const StopToken &stop) override;
        RecordingInfo Versions() override { return info_; }
        void Stop() override; // work mode sleep (best effort)

    private:
        explicit LiveLivoxSource(const LiveLivoxConfig &config) : config_(config) {}

        // One command's answer, kept until the source is destroyed (the SDK
        // may answer after we stopped waiting).
        struct Pending
        {
            std::mutex mutex;
            std::condition_variable done_cv;
            bool done = false;
            int status = 0;
            uint8_t ret_code = 0;
            std::string version;
        };

        LiveLivoxConfig config_;
        std::string config_path_;
        LivoxPacketPool pool_;
        RecordingInfo info_;
        std::mutex device_mutex_;
        bool found_ = false;
        uint32_t handle_ = 0;
        uint8_t dev_type_ = 0;
        std::string sn_, found_ip_;
        std::vector<std::unique_ptr<Pending>> pending_;
        bool stopped_ = false;

        static void OnPacket(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data);
        static void OnInfoChange(uint32_t handle, const LivoxLidarInfo *info, void *client_data);
        static void OnControl(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
        static void OnFirmware(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse *response, void *client_data);

        Pending *NewPending();
        // Waits up to 2 s for `p`; prints a warning naming `what` if it got no
        // successful answer. True if it did.
        bool Await(Pending *p, int send_status, const char *what);
    };
}
