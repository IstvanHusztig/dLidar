#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>

namespace dlidar
{
    // What a recording's files say about the sensor that made it
    // (docs/recording-format.md). Each sensor adapter defines one; the
    // storage writes it, and recovery picks it by the id stored in the points
    // temp file, so a leftover is finished as the sensor that recorded it.
    struct RecordingProfile
    {
        uint32_t sensor_id = 0;              // stored in points_tempNNNN.bin
        const char *sensor_model = "";       // timingNNNN.json "sensor_model"
        const char *las_system_identifier = ""; // LAZ header, at most 31 characters
        // Descriptions of the LAZ extra bytes in record order: range, raw_range,
        // sample_index, packet_seq, alpha, theta. At most 31 characters each.
        const char *extra_byte_descriptions[6] = {"", "", "", "", "", ""};
        const char *packet_state_header = ""; // packetstateNNNN.tsv first line with '\n'; "": no file
    };

    // Points temp files before recording format 3 carry no sensor id: they are
    // all Unitree L2 recordings.
    const uint32_t kPreFormat3SensorId = 1;

    constexpr bool FitsLasText(const char *s)
    {
        size_t n = 0;
        while (s[n])
            n++;
        return n <= 31;
    }

    // The LAS header and the Extra Bytes VLR hold at most 31 characters per text.
    constexpr bool FitsLas(const RecordingProfile &p)
    {
        bool ok = FitsLasText(p.las_system_identifier);
        for (const char *d : p.extra_byte_descriptions)
            ok = ok && FitsLasText(d);
        return ok;
    }

    inline const RecordingProfile *FindProfile(const std::vector<RecordingProfile> &profiles, uint32_t sensor_id)
    {
        for (const RecordingProfile &p : profiles)
            if (p.sensor_id == sensor_id)
                return &p;
        return nullptr;
    }
}
