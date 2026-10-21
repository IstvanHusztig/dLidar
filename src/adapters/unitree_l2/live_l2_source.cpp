#include "adapters/unitree_l2/live_l2_source.h"

#include "adapters/unitree_l2/vendor/unitree_lidar_sdk.h"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <time.h>
#include <unistd.h>

using unitree_lidar_sdk::UnitreeLidarReader;

namespace dlidar
{
    namespace
    {
        HostTime HostNow()
        {
            timespec m, r;
            clock_gettime(CLOCK_MONOTONIC, &m);
            clock_gettime(CLOCK_REALTIME, &r);
            return {(int64_t)m.tv_sec * 1000000000LL + m.tv_nsec, (int64_t)r.tv_sec * 1000000000LL + r.tv_nsec};
        }

        void PrintDirtyPercentage(UnitreeLidarReader *lreader)
        {
            float dirtyPercentage;
            while (!lreader->getDirtyPercentage(dirtyPercentage))
            {
                lreader->runParse();
            }
            printf("dirty percentage = %f %%\n", dirtyPercentage);
            sleep(1);
        }

        void PrintTimeDelay(UnitreeLidarReader *lreader)
        {
            double timeDelay;
            while (!lreader->getTimeDelay(timeDelay))
            {
                lreader->runParse();
            }
            printf("time delay (second) = %f\n", timeDelay);
            sleep(1);
        }

        void RestartLidar(UnitreeLidarReader *lreader)
        {
            std::cout << "stop lidar rotation ..." << std::endl;
            lreader->stopLidarRotation();
            sleep(3);

            std::cout << "start lidar rotation ..." << std::endl;
            lreader->startLidarRotation();
            sleep(3);
        }

        // The SDK buffers packets while nothing reads them (lidar restart, the
        // status queries above). Reading those out now instead of sleeping keeps a
        // burst of stale packets (late arrivals) out of the recording.
        void DrainPendingPackets(UnitreeLidarReader *lreader, std::chrono::milliseconds duration, const StopToken &stop)
        {
            auto until = std::chrono::steady_clock::now() + duration;
            while (std::chrono::steady_clock::now() < until && !stop.StopRequested())
            {
                lreader->runParse();
            }
        }

        // Calls `get` (a getVersionOf... of the reader) while parsing packets, for at
        // most `timeout`. The SDK fills the version from the L2's version packet.
        template <typename Get>
        std::string QueryVersion(UnitreeLidarReader *lreader, Get get, std::chrono::milliseconds timeout,
                                 const StopToken &stop)
        {
            std::string version;
            auto until = std::chrono::steady_clock::now() + timeout;
            while (!get(version))
            {
                if (std::chrono::steady_clock::now() >= until || stop.StopRequested())
                    return "unknown";
                lreader->runParse();
            }
            return version;
        }

        RecordingInfo QueryRecordingInfo(UnitreeLidarReader *lreader, uint32_t work_mode, const StopToken &stop)
        {
            RecordingInfo info;
            info.work_mode = work_mode;
            info.lidar_firmware_version = QueryVersion(
                lreader, [&](std::string &v) { return lreader->getVersionOfLidarFirmware(v); }, std::chrono::seconds(3), stop);
            info.lidar_hardware_version = QueryVersion(
                lreader, [&](std::string &v) { return lreader->getVersionOfLidarHardware(v); }, std::chrono::seconds(1), stop);
            if (!lreader->getVersionOfSDK(info.sdk_version))
                info.sdk_version = unitree_lidar_sdk_VERSION;
            std::cout << "L2 firmware " << info.lidar_firmware_version << ", hardware " << info.lidar_hardware_version
                      << ", SDK " << info.sdk_version << ", work mode " << work_mode << std::endl;
            return info;
        }
    }

    std::unique_ptr<LiveL2Source> LiveL2Source::Open(const LiveL2Config &config)
    {
        UnitreeLidarReader *lreader = unitree_lidar_sdk::createUnitreeLidarReader();
        if (lreader->initializeUDP(config.lidar_port, config.lidar_ip, config.local_port, config.local_ip,
                                   config.cloud_scan_num, config.use_system_timestamp))
        {
            printf("Unilidar initialization failed! Exit here!\n");
            return nullptr;
        }
        printf("Unilidar initialization succeed!\n");

        std::cout << "set Lidar work mode to: " << config.work_mode << std::endl;
        lreader->setLidarWorkMode(config.work_mode);
        sleep(1);

        return std::unique_ptr<LiveL2Source>(new LiveL2Source(lreader, config.work_mode));
    }

    LiveL2Source::LiveL2Source(UnitreeLidarReader *reader, uint32_t work_mode) : reader_(reader), work_mode_(work_mode)
    {
        info_.work_mode = work_mode;
    }

    int LiveL2Source::Next(HostTime &arrival)
    {
        int result = reader_->runParse();
        // Both packet types keep the L2's own stamp (use_system_timestamp is
        // off); the host arrival time is taken here for diagnostics and the
        // fallback clock map.
        if (result == LIDAR_IMU_DATA_PACKET_TYPE || result == LIDAR_POINT_DATA_PACKET_TYPE)
        {
            arrival = HostNow();
            return result;
        }
        return 0;
    }

    const unitree_lidar_sdk::LidarPointDataPacket &LiveL2Source::PointPacket() const
    {
        return reader_->getLidarPointDataPacket();
    }

    const unitree_lidar_sdk::LidarImuData &LiveL2Source::ImuPacket() const
    {
        return reader_->getLidarImuDataPacket().data;
    }

    bool LiveL2Source::Prepare(const StopToken &stop)
    {
        RestartLidar(reader_);
        PrintDirtyPercentage(reader_);
        PrintTimeDelay(reader_);
        info_ = QueryRecordingInfo(reader_, work_mode_, stop);
        DrainPendingPackets(reader_, std::chrono::seconds(2), stop);
        return true;
    }

    std::optional<float> LiveL2Source::LensDirtiness()
    {
        // Non-blocking: the dirty_index cached from the most recently parsed packet.
        float dirtyPercentage;
        if (reader_->getDirtyPercentage(dirtyPercentage))
            return dirtyPercentage;
        return std::nullopt;
    }

    void LiveL2Source::Stop()
    {
        reader_->stopLidarRotation();
    }
}
