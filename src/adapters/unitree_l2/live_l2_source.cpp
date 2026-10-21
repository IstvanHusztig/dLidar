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

        // The L2 answers the start-up queries within a few seconds once it
        // rotates; longer means it is not there (UDP does not fail without it).
        const std::chrono::seconds kStartupAnswerTimeout(10);

        // Sleeps `seconds`, returning early (false) on a stop.
        bool SleepUnlessStopped(int seconds, const StopToken &stop)
        {
            auto until = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
            while (std::chrono::steady_clock::now() < until)
            {
                if (stop.StopRequested())
                    return false;
                usleep(50000);
            }
            return !stop.StopRequested();
        }

        // Parses packets until `get` succeeds; false on a stop or after the timeout.
        template <typename Get>
        bool ParseUntil(UnitreeLidarReader *lreader, Get get, const StopToken &stop)
        {
            auto until = std::chrono::steady_clock::now() + kStartupAnswerTimeout;
            while (!get())
            {
                if (stop.StopRequested() || std::chrono::steady_clock::now() >= until)
                    return false;
                lreader->runParse();
            }
            return true;
        }

        enum class Step
        {
            Done,
            Stopped,
            NoAnswer
        };

        Step Outcome(bool done, const StopToken &stop)
        {
            if (done)
                return Step::Done;
            return stop.StopRequested() ? Step::Stopped : Step::NoAnswer;
        }

        Step PrintDirtyPercentage(UnitreeLidarReader *lreader, const StopToken &stop)
        {
            float dirtyPercentage;
            if (!ParseUntil(lreader, [&] { return lreader->getDirtyPercentage(dirtyPercentage); }, stop))
                return Outcome(false, stop);
            printf("dirty percentage = %f %%\n", dirtyPercentage);
            return Outcome(SleepUnlessStopped(1, stop), stop);
        }

        Step PrintTimeDelay(UnitreeLidarReader *lreader, const StopToken &stop)
        {
            double timeDelay;
            if (!ParseUntil(lreader, [&] { return lreader->getTimeDelay(timeDelay); }, stop))
                return Outcome(false, stop);
            printf("time delay (second) = %f\n", timeDelay);
            return Outcome(SleepUnlessStopped(1, stop), stop);
        }

        Step RestartLidar(UnitreeLidarReader *lreader, const StopToken &stop)
        {
            std::cout << "stop lidar rotation ..." << std::endl;
            lreader->stopLidarRotation();
            if (!SleepUnlessStopped(3, stop))
                return Step::Stopped;

            std::cout << "start lidar rotation ..." << std::endl;
            lreader->startLidarRotation();
            return Outcome(SleepUnlessStopped(3, stop), stop);
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
        // A stop ends the start-up early and successfully: the capture loop
        // then sees the stop and the (empty) recording is finished normally.
        for (auto step : {&RestartLidar, &PrintDirtyPercentage, &PrintTimeDelay})
        {
            Step result = step(reader_, stop);
            if (result == Step::Stopped)
                return true;
            if (result == Step::NoAnswer)
            {
                std::cerr << "The L2 did not answer within " << kStartupAnswerTimeout.count() << " s; check its power and network ("
                          << "is it at the configured IP?)." << std::endl;
                return false;
            }
        }
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
