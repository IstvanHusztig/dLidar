#pragma once

#include <stdint.h>
#include <string>

namespace dlidar
{
    // Version of what a recording contains (docs/l2-recording-format.md). Written
    // to timingNNNN.json and the LAZ generating_software. 1: everything before
    // the LAZ extra bytes; 2: LAZ extra bytes, packetstateNNNN.tsv, sensor
    // versions in timingNNNN.json.
    const int kRecordingFormatVersion = 2;

    // Sensor and software versions of a recording, added to timingNNNN.json.
    struct RecordingInfo
    {
        std::string lidar_firmware_version = "unknown";
        std::string lidar_hardware_version = "unknown";
        std::string sdk_version = "unknown";
        int64_t work_mode = -1;
    };

    inline std::string JsonEscape(const std::string &v)
    {
        std::string e;
        for (char ch : v)
        {
            if (ch == '"' || ch == '\\')
                e += '\\';
            if ((unsigned char)ch >= 0x20)
                e += ch;
        }
        return e;
    }

    // Adds the recording keys at the top of a timing report ("{\n...}").
    inline std::string AddRecordingInfo(const std::string &report, const RecordingInfo &info)
    {
        if (report.compare(0, 2, "{\n") != 0)
            return report;
        std::string keys = "  \"recording_format_version\": " + std::to_string(kRecordingFormatVersion) + ",\n" +
                           "  \"lidar_firmware_version\": \"" + JsonEscape(info.lidar_firmware_version) + "\",\n" +
                           "  \"lidar_hardware_version\": \"" + JsonEscape(info.lidar_hardware_version) + "\",\n" +
                           "  \"sdk_version\": \"" + JsonEscape(info.sdk_version) + "\",\n" +
                           "  \"work_mode\": " + std::to_string(info.work_mode) + ",\n";
        return "{\n" + keys + report.substr(2);
    }
}
