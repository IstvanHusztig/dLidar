#pragma once

#include "application/ports/device_events.h"
#include "application/ports/lidar_sensor.h"

#include <sstream>
#include <stdint.h>

namespace dlidar
{
    // Checks the sensor's lens dirtiness at most every kIntervalNs of host time
    // (the first check right away) and warns above kWarnPercent. A dirty lens
    // attenuates/scatters returns and quietly degrades the whole run (fogging,
    // dust - relevant for handheld cave/outdoor use). Non-blocking: the sensor
    // reports what it cached from its latest packet.
    class LensMonitor
    {
    public:
        static constexpr float kWarnPercent = 5.0f;
        static constexpr int64_t kIntervalNs = 10000000000LL;

        void Check(LidarSensor &sensor, int64_t host_mono_ns, DeviceEvents &events)
        {
            if (checked_ && host_mono_ns - last_ns_ < kIntervalNs)
                return;
            checked_ = true;
            last_ns_ = host_mono_ns;

            std::optional<float> dirty = sensor.LensDirtiness();
            if (dirty && *dirty > kWarnPercent)
            {
                std::ostringstream text;
                text << "WARNING: lens dirty percentage = " << *dirty << " % (threshold " << kWarnPercent
                     << " %) - clean the sensor housing, points may be degraded.";
                events.OnWarning({WarningKind::LensDirty, text.str()});
            }
        }

    private:
        bool checked_ = false;
        int64_t last_ns_ = 0;
    };
}
