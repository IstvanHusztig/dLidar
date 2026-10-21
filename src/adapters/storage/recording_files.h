#pragma once

// File names, session numbering, IMU CSV rows, text-file trimming and the
// points temp layout of a recording (docs/l2-recording-format.md).
#include "domain/recording/imu_sample.h"
#include "domain/recording/point_record.h"

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace dlidar
{
    // Files of a recording inside the output directory.
    // Every recording takes the next free session number N, so its files group:
    // imuNNNN.csv, lidarNNNN.laz, timingNNNN.json (how the times were made),
    // packetsNNNN.bin (arrival and L2 stamp of every packet, for replay),
    // packetstateNNNN.tsv (state and calibration of every point packet) and,
    // while recording, points_tempNNNN.bin. dMapper reads every .csv/.las/.laz
    // in the directory, so no other file may use those extensions.
    const char *const kImuCsvPrefix = "imu";
    const char *const kImuCsvSuffix = ".csv";
    const char *const kLazPrefix = "lidar";
    const char *const kLazSuffix = ".laz";
    const char *const kPointsTempPrefix = "points_temp"; // raw PointRecord records while recording
    const char *const kPointsTempSuffix = ".bin";
    const char *const kTimingPrefix = "timing";
    const char *const kTimingSuffix = ".json";
    const char *const kPacketLogPrefix = "packets"; // kPacketLogMagic, then PacketTiming records
    const char *const kPacketLogSuffix = ".bin";
    const char *const kPacketStatePrefix = "packetstate"; // the sensor's diagnostics header, then one row per point packet
    const char *const kPacketStateSuffix = ".tsv";
    const char *const kImuCsvHeader = "gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix\n";

    inline std::string JoinPath(const std::string &dir, const std::string &name)
    {
        return (std::filesystem::path(dir) / name).string();
    }

    inline std::string SessionFileName(const char *prefix, int index, const char *suffix)
    {
        char name[48];
        snprintf(name, sizeof name, "%s%04d%s", prefix, index, suffix);
        return name;
    }
    inline std::string ImuCsvName(int index) { return SessionFileName(kImuCsvPrefix, index, kImuCsvSuffix); }
    inline std::string LazName(int index) { return SessionFileName(kLazPrefix, index, kLazSuffix); }
    inline std::string PointsTempName(int index) { return SessionFileName(kPointsTempPrefix, index, kPointsTempSuffix); }
    inline std::string TimingName(int index) { return SessionFileName(kTimingPrefix, index, kTimingSuffix); }
    inline std::string PacketLogName(int index) { return SessionFileName(kPacketLogPrefix, index, kPacketLogSuffix); }
    inline std::string PacketStateName(int index) { return SessionFileName(kPacketStatePrefix, index, kPacketStateSuffix); }

    // The session number in `name` if it is exactly prefix + 4 digits + suffix, else 0.
    inline int ParseSessionIndex(const std::string &name, const char *prefix, const char *suffix)
    {
        size_t p = strlen(prefix), q = strlen(suffix);
        if (name.size() != p + 4 + q || name.compare(0, p, prefix) != 0 || name.compare(p + 4, q, suffix) != 0)
            return 0;
        int index = 0;
        for (size_t i = p; i < p + 4; i++)
        {
            if (name[i] < '0' || name[i] > '9')
                return 0;
            index = index * 10 + (name[i] - '0');
        }
        return index;
    }

    // The session number for a new recording: one above the highest used by any
    // recording file in `dir`, so nothing from an earlier recording is overwritten.
    inline int NextSessionIndex(const std::string &dir)
    {
        int highest = 0;
        std::error_code ec;
        for (const auto &entry : std::filesystem::directory_iterator(dir, ec))
        {
            std::string name = entry.path().filename().string();
            highest = std::max({highest, ParseSessionIndex(name, kImuCsvPrefix, kImuCsvSuffix),
                                ParseSessionIndex(name, kLazPrefix, kLazSuffix),
                                ParseSessionIndex(name, kPointsTempPrefix, kPointsTempSuffix),
                                ParseSessionIndex(name, kTimingPrefix, kTimingSuffix),
                                ParseSessionIndex(name, kPacketLogPrefix, kPacketLogSuffix),
                                ParseSessionIndex(name, kPacketStatePrefix, kPacketStateSuffix)});
        }
        return highest + 1;
    }

    // Session numbers of unfinished recordings (points temp files) in `dir`, ascending.
    inline std::vector<int> LeftoverSessions(const std::string &dir)
    {
        std::vector<int> found;
        std::error_code ec;
        for (const auto &entry : std::filesystem::directory_iterator(dir, ec))
        {
            int index = ParseSessionIndex(entry.path().filename().string(), kPointsTempPrefix, kPointsTempSuffix);
            if (index > 0)
                found.push_back(index);
        }
        std::sort(found.begin(), found.end());
        return found;
    }

    // One CSV row, formatted as before (std::fixed with 17 decimals for floats).
    inline void AppendImuRow(std::string &out, const ImuSample &imu)
    {
        char row[512];
        int n = snprintf(row, sizeof row, "%.17f %.17f %.17f %.17f %.17f %.17f %d %" PRId64 " %" PRId64 "\n",
                         (double)imu.GyroX, (double)imu.GyroY, (double)imu.GyroZ,
                         (double)imu.AccelerationX, (double)imu.AccelerationY, (double)imu.AccelerationZ,
                         imu.ImuId, imu.LidarTimestamp, imu.EpochTimestamp);
        if (n > 0 && n < (int)sizeof row)
            out.append(row, (size_t)n);
    }

    inline bool WriteAll(int fd, const char *data, size_t size)
    {
        while (size > 0)
        {
            ssize_t n = write(fd, data, size);
            if (n < 0)
            {
                if (errno == EINTR)
                    continue;
                return false;
            }
            data += n;
            size -= (size_t)n;
        }
        return true;
    }

    // Cuts a text file back to its last complete line (a crash or power loss can
    // leave a partial one); an empty result gets `header`. Returns the number of
    // bytes removed, or -1 on error.
    inline int64_t TrimPartialLine(const std::string &csv_path, const char *header)
    {
        int fd = open(csv_path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0)
            return -1;
        struct stat st;
        if (fstat(fd, &st) != 0)
        {
            close(fd);
            return -1;
        }

        off_t end = st.st_size;
        off_t keep = 0;
        char block[65536];
        while (end > 0)
        {
            off_t start = std::max<off_t>(0, end - (off_t)sizeof block);
            ssize_t n = pread(fd, block, (size_t)(end - start), start);
            if (n != end - start)
            {
                close(fd);
                return -1;
            }
            const void *nl = memrchr(block, '\n', (size_t)n);
            if (nl)
            {
                keep = start + ((const char *)nl - block) + 1;
                break;
            }
            end = start;
        }

        int64_t removed = st.st_size - keep;
        bool ok = removed == 0 || ftruncate(fd, keep) == 0;
        if (ok && keep == 0)
            ok = WriteAll(fd, header, strlen(header));
        close(fd);
        return ok ? removed : -1;
    }

    inline int64_t TrimPartialCsvLine(const std::string &csv_path) { return TrimPartialLine(csv_path, kImuCsvHeader); }

    // points_tempNNNN.bin since recording format 2: kPointsTempMagic, the record
    // size (uint32) and 4 reserved bytes, then PointRecord records. A temp file
    // without the magic is from an older build: bare 32-byte PointDLidarV1 records.
    const char kPointsTempMagic[8] = {'d', 'L', 'p', 't', 's', 'v', '2', '\n'};
    const uint64_t kPointsTempHeaderSize = 16;

    // The point record of format 1 temp files.
    struct PointDLidarV1
    {
        float x;
        float y;
        float z;
        float intensity;
        double time;
        uint32_t ring;
    };
    static_assert(sizeof(PointDLidarV1) == 32, "points temp record layout (format 1)");

    struct PointsTempLayout
    {
        bool extra = true;        // format 2 records, with the extra-bytes fields
        uint64_t data_offset = 0; // first record
        size_t record_size = sizeof(PointRecord);
    };

    inline void WritePointsTempHeader(std::ostream &out)
    {
        char header[kPointsTempHeaderSize] = {};
        memcpy(header, kPointsTempMagic, sizeof kPointsTempMagic);
        uint32_t record_size = sizeof(PointRecord);
        memcpy(header + 8, &record_size, sizeof record_size);
        out.write(header, sizeof header);
    }

    // Layout of a points temp file of `bytes` bytes. A file shorter than the
    // header that starts like it was cut while writing the header: no points.
    inline bool DetectPointsTempLayout(std::ifstream &in, uint64_t bytes, PointsTempLayout &layout)
    {
        char header[kPointsTempHeaderSize] = {};
        size_t n = (size_t)std::min<uint64_t>(bytes, kPointsTempHeaderSize);
        in.clear();
        in.seekg(0);
        if (n > 0 && !in.read(header, (std::streamsize)n))
            return false;
        size_t m = std::min(n, sizeof kPointsTempMagic);
        if (memcmp(header, kPointsTempMagic, m) == 0 && (n == kPointsTempHeaderSize || n == bytes))
        {
            layout.extra = true;
            layout.data_offset = std::min<uint64_t>(bytes, kPointsTempHeaderSize);
            uint32_t record_size = sizeof(PointRecord);
            if (n == kPointsTempHeaderSize)
                memcpy(&record_size, header + 8, sizeof record_size);
            if (record_size != sizeof(PointRecord))
            {
                std::cerr << "Error: points temp file has " << record_size << "-byte records, this build reads "
                          << sizeof(PointRecord) << "." << std::endl;
                return false;
            }
            layout.record_size = record_size;
            return true;
        }
        layout.extra = false;
        layout.data_offset = 0;
        layout.record_size = sizeof(PointDLidarV1);
        return true;
    }

    // Reads `n` records of `layout` into `out` (format 1 records get zero extra fields).
    inline bool ReadPointsTempRecords(std::ifstream &in, const PointsTempLayout &layout, size_t n, PointRecord *out,
                                      std::vector<PointDLidarV1> &v1)
    {
        if (layout.extra)
            return (bool)in.read(reinterpret_cast<char *>(out), (std::streamsize)(n * sizeof(PointRecord)));
        v1.resize(n);
        if (!in.read(reinterpret_cast<char *>(v1.data()), (std::streamsize)(n * sizeof(PointDLidarV1))))
            return false;
        for (size_t i = 0; i < n; i++)
        {
            PointRecord p{};
            p.x = v1[i].x;
            p.y = v1[i].y;
            p.z = v1[i].z;
            p.intensity = v1[i].intensity;
            p.time = v1[i].time;
            p.ring = v1[i].ring;
            out[i] = p;
        }
        return true;
    }
}
