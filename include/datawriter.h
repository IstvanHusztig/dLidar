#pragma once

#include "unitree_lidar_sdk.h"
#include "domain/recording/recording_info.h"
#include "domain/recording/sensor_chunk.h"
#include "time_alignment.h"
#include "packet_state.h"
#include "shutdown.h"
#include <dll/laszip_api.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace unitree_lidar_sdk;


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
const char *const kPointsTempPrefix = "points_temp"; // raw PointDLidar records while recording
const char *const kPointsTempSuffix = ".bin";
const char *const kTimingPrefix = "timing";
const char *const kTimingSuffix = ".json";
const char *const kPacketLogPrefix = "packets"; // kPacketLogMagic, then PacketTiming records
const char *const kPacketLogSuffix = ".bin";
const char *const kPacketStatePrefix = "packetstate"; // kPacketStateHeader, then one row per point packet
const char *const kPacketStateSuffix = ".tsv";
const char *const kImuCsvHeader = "gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix\n";

using dlidar::AddRecordingInfo;
using dlidar::JsonEscape;
using dlidar::kRecordingFormatVersion;
using dlidar::RecordingInfo;
using dlidar::SensorChunk;

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
inline void AppendImuRow(std::string &out, const OutputImuData &imu)
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
// size (uint32) and 4 reserved bytes, then PointDLidar records. A temp file
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
    size_t record_size = sizeof(PointDLidar);
};

inline void WritePointsTempHeader(std::ostream &out)
{
    char header[kPointsTempHeaderSize] = {};
    memcpy(header, kPointsTempMagic, sizeof kPointsTempMagic);
    uint32_t record_size = sizeof(PointDLidar);
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
        uint32_t record_size = sizeof(PointDLidar);
        if (n == kPointsTempHeaderSize)
            memcpy(&record_size, header + 8, sizeof record_size);
        if (record_size != sizeof(PointDLidar))
        {
            std::cerr << "Error: points temp file has " << record_size << "-byte records, this build reads "
                      << sizeof(PointDLidar) << "." << std::endl;
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
inline bool ReadPointsTempRecords(std::ifstream &in, const PointsTempLayout &layout, size_t n, PointDLidar *out,
                                  std::vector<PointDLidarV1> &v1)
{
    if (layout.extra)
        return (bool)in.read(reinterpret_cast<char *>(out), (std::streamsize)(n * sizeof(PointDLidar)));
    v1.resize(n);
    if (!in.read(reinterpret_cast<char *>(v1.data()), (std::streamsize)(n * sizeof(PointDLidarV1))))
        return false;
    for (size_t i = 0; i < n; i++)
    {
        PointDLidar p{};
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

// The LAZ extra bytes of recording format 2, in record order. LASzip writes
// the standard Extra Bytes VLR; readers that don't know them see the plain
// LAS 1.2 / point format 1 fields unchanged. Names and descriptions are at
// most 31 characters.
struct LazExtraAttribute
{
    laszip_U32 type; // LASzip attribute type: 2 = uint16, 4 = uint32, 8 = float32
    const char *name;
    const char *description;
};
const LazExtraAttribute kLazExtraAttributes[] = {
    {8, "range", "range_scale*(raw+bias) [m]"},
    {2, "raw_range", "range as sent by the L2 [mm]"},
    {2, "sample_index", "index of point in packet"},
    {4, "packet_seq", "point packet info.seq"},
    {8, "alpha", "alpha scan angle incl bias, rad"},
    {8, "theta", "theta scan angle incl bias, rad"},
};
const laszip_U16 kLazExtraBytes = 20;

inline void PackLazExtraBytes(const PointDLidar &pt, laszip_U8 *out)
{
    memcpy(out + 0, &pt.range, 4);
    memcpy(out + 4, &pt.raw_range, 2);
    memcpy(out + 6, &pt.sample_index, 2);
    memcpy(out + 8, &pt.packet_seq, 4);
    memcpy(out + 12, &pt.alpha, 4);
    memcpy(out + 16, &pt.theta, 4);
}

// Encodes `count` points starting at record `first` of the temp file into a
// LAS 1.2 / point format 1 LAZ file, with the format 2 extra bytes if the
// temp file has them. The header needs the bounds before any point is
// written, so the range is read twice.
inline bool WriteLazChunk(std::ifstream &in, const PointsTempLayout &layout, uint64_t first, uint64_t count,
                          const std::string &laz_path)
{
    const size_t kBlock = 65536;
    std::vector<PointDLidar> block(kBlock);
    std::vector<PointDLidarV1> v1;
    const std::streamoff start = (std::streamoff)(layout.data_offset + first * layout.record_size);

    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x, min_z = min_x, max_z = max_x;
    in.clear();
    in.seekg(start);
    for (uint64_t done = 0; done < count;)
    {
        size_t n = (size_t)std::min<uint64_t>(kBlock, count - done);
        if (!ReadPointsTempRecords(in, layout, n, block.data(), v1))
            return false;
        for (size_t i = 0; i < n; i++)
        {
            min_x = std::min(min_x, (double)block[i].x);
            max_x = std::max(max_x, (double)block[i].x);
            min_y = std::min(min_y, (double)block[i].y);
            max_y = std::max(max_y, (double)block[i].y);
            min_z = std::min(min_z, (double)block[i].z);
            max_z = std::max(max_z, (double)block[i].z);
        }
        done += n;
    }

    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
        std::cerr << "DLL ERROR: creating laszip writer\n";
        return false;
    }

    laszip_header *header;
    if (laszip_get_header_pointer(laszip_writer, &header))
    {
        std::cerr << "DLL ERROR: getting header pointer\n";
        laszip_destroy(laszip_writer);
        return false;
    }

    header->file_source_ID = 4711;
    header->global_encoding = (1 << 0);
    header->version_major = 1;
    header->version_minor = 2;
    header->point_data_format = 1;
    header->point_data_record_length = 28;
    header->x_scale_factor = 0.0001;
    header->y_scale_factor = 0.0001;
    header->z_scale_factor = 0.0001;
    header->number_of_point_records = (laszip_U32)count;
    header->number_of_points_by_return[0] = (laszip_U32)count;
    header->max_x = max_x;
    header->min_x = min_x;
    header->max_y = max_y;
    header->min_y = min_y;
    header->max_z = max_z;
    header->min_z = min_z;

    if (layout.extra)
    {
        for (const LazExtraAttribute &a : kLazExtraAttributes)
        {
            if (laszip_add_attribute(laszip_writer, a.type, a.name, a.description, 1.0, 0.0))
            {
                std::cerr << "DLL ERROR: adding extra bytes attribute " << a.name << "\n";
                laszip_destroy(laszip_writer);
                return false;
            }
        }
        header->point_data_record_length = 28 + kLazExtraBytes;
        memset(header->generating_software, 0, sizeof header->generating_software);
        snprintf(header->generating_software, sizeof header->generating_software, "dLidar recording format %d",
                 kRecordingFormatVersion);
        laszip_preserve_generating_software(laszip_writer, 1);
    }

    if (laszip_open_writer(laszip_writer, laz_path.c_str(), 1))
    {
        std::cerr << "DLL ERROR: opening laszip writer for " << laz_path << "\n";
        laszip_destroy(laszip_writer);
        return false;
    }

    laszip_point *laszipPoint;
    bool ok = !laszip_get_point_pointer(laszip_writer, &laszipPoint);
    if (ok && layout.extra && laszipPoint->num_extra_bytes != kLazExtraBytes)
    {
        std::cerr << "DLL ERROR: LAZ point has " << laszipPoint->num_extra_bytes << " extra bytes, expected "
                  << kLazExtraBytes << "\n";
        ok = false;
    }

    in.clear();
    in.seekg(start);
    laszip_F64 coordinates[3];
    for (uint64_t done = 0; ok && done < count;)
    {
        size_t n = (size_t)std::min<uint64_t>(kBlock, count - done);
        if (!ReadPointsTempRecords(in, layout, n, block.data(), v1))
        {
            ok = false;
            break;
        }
        for (size_t i = 0; i < n; i++)
        {
            const PointDLidar &pt = block[i];
            laszipPoint->intensity = pt.intensity;
            laszipPoint->gps_time = pt.time;
            laszipPoint->user_data = pt.ring;
            if (layout.extra)
                PackLazExtraBytes(pt, laszipPoint->extra_bytes);

            coordinates[0] = pt.x;
            coordinates[1] = pt.y;
            coordinates[2] = pt.z;

            if (laszip_set_coordinates(laszip_writer, coordinates) || laszip_write_point(laszip_writer))
            {
                std::cerr << "DLL ERROR: Failed writing point during packaging.\n";
                ok = false;
                break;
            }
        }
        done += n;
    }

    if (laszip_close_writer(laszip_writer))
    {
        std::cerr << "DLL ERROR: closing laszip writer\n";
        ok = false;
    }
    laszip_destroy(laszip_writer);
    return ok;
}

// Converts a points temp file into lidarNNNN.laz (session `index`) in out_dir. A trailing partial
// record (from an interrupted write) is ignored. The LAZ is written to a .tmp
// file and renamed when complete; the temp file is deleted only after that, so
// an interrupted finalization is simply redone next time.
inline bool FinalizePointsTemp(const std::string &temp_path, const std::string &out_dir, int index)
{
    std::error_code ec;
    uint64_t bytes = std::filesystem::file_size(temp_path, ec);
    if (ec)
    {
        std::cerr << "Error: cannot read " << temp_path << ": " << ec.message() << std::endl;
        return false;
    }

    std::ifstream in(temp_path, std::ios::in | std::ios::binary);
    if (!in.is_open())
    {
        std::cerr << "Error: cannot open " << temp_path << std::endl;
        return false;
    }
    PointsTempLayout layout;
    if (!DetectPointsTempLayout(in, bytes, layout))
    {
        std::cerr << "Error: cannot read the header of " << temp_path << std::endl;
        return false;
    }
    if (!layout.extra)
        std::cout << temp_path << " is from an older dLidar (recording format 1): no extra bytes in the LAZ." << std::endl;

    uint64_t total = (bytes - layout.data_offset) / layout.record_size;
    if ((bytes - layout.data_offset) % layout.record_size)
    {
        std::cerr << "WARNING: ignoring " << (bytes - layout.data_offset) % layout.record_size
                  << " trailing bytes (partial point) in " << temp_path << std::endl;
    }

    std::string laz_path = JoinPath(out_dir, LazName(index));
    std::string part_path = laz_path + ".tmp";
    std::cout << "Packaging " << total << " points into " << laz_path << " ..." << std::endl;
    if (!WriteLazChunk(in, layout, 0, total, part_path))
    {
        std::cerr << "Error: packaging " << laz_path << " failed; " << temp_path
                  << " is kept and will be finalized on the next start." << std::endl;
        std::filesystem::remove(part_path, ec);
        return false;
    }
    in.close();
    std::filesystem::rename(part_path, laz_path, ec);
    if (ec)
    {
        std::cerr << "Error: cannot rename " << part_path << ": " << ec.message() << std::endl;
        return false;
    }

    std::filesystem::remove(temp_path, ec);
    return true;
}

// Finalizes one unfinished recording (session `index`): trims the IMU CSV and
// the packet state TSV to their last complete rows and converts the points
// temp file into lidarNNNN.laz.
// With move_aside both go into a recovered_<time> subdirectory.
inline bool RecoverSession(const std::string &dir, int index, bool move_aside)
{
    std::string temp_path = JoinPath(dir, PointsTempName(index));
    std::string csv_path = JoinPath(dir, ImuCsvName(index));
    struct stat st;
    if (stat(temp_path.c_str(), &st) != 0)
        return true;

    std::cout << "Found an unfinished recording " << index << " in " << dir << ", finalizing it." << std::endl;

    if (access(csv_path.c_str(), F_OK) == 0)
    {
        int64_t removed = TrimPartialCsvLine(csv_path);
        if (removed < 0)
            std::cerr << "WARNING: could not check " << csv_path << " for a partial last row." << std::endl;
        else if (removed > 0)
            std::cout << "Removed a partial last row (" << removed << " bytes) from " << csv_path << std::endl;
    }
    std::string state_path = JoinPath(dir, PacketStateName(index));
    if (access(state_path.c_str(), F_OK) == 0 && TrimPartialLine(state_path, kPacketStateHeader) < 0)
        std::cerr << "WARNING: could not check " << state_path << " for a partial last row." << std::endl;

    std::string target = dir;
    if (move_aside)
    {
        char stamp[32];
        struct tm tm_local;
        localtime_r(&st.st_mtime, &tm_local);
        strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &tm_local);
        target = JoinPath(dir, std::string("recovered_") + stamp);
        for (int i = 2; std::filesystem::exists(target); i++)
            target = JoinPath(dir, std::string("recovered_") + stamp + "_" + std::to_string(i));

        std::error_code ec;
        std::filesystem::create_directory(target, ec);
        if (ec)
        {
            std::cerr << "Error: cannot create " << target << ": " << ec.message() << std::endl;
            return false;
        }
        for (const std::string &name : {ImuCsvName(index), TimingName(index), PacketLogName(index), PacketStateName(index)})
        {
            std::string from = JoinPath(dir, name);
            if (access(from.c_str(), F_OK) != 0)
                continue;
            std::filesystem::rename(from, JoinPath(target, name), ec);
            if (ec)
            {
                std::cerr << "Error: cannot move " << from << ": " << ec.message() << std::endl;
                return false;
            }
        }
    }

    if (!FinalizePointsTemp(temp_path, target, index))
        return false;
    std::cout << "Recovered recording is in " << target << std::endl;
    return true;
}

// Finalizes every recording in `dir` that was not finished (crash, kill, power
// loss). With move_aside each recovered recording is put in a recovered_<time>
// subdirectory, so a new recording in `dir` does not mix with it. Returns
// false if something was found and failed.
inline bool RecoverInterruptedRecording(const std::string &dir, bool move_aside)
{
    bool ok = true;
    for (int index : LeftoverSessions(dir))
        ok = RecoverSession(dir, index, move_aside) && ok;
    return ok;
}

class ContinuousDataWriter
{
private:
    std::string out_dir;
    std::string csv_filename;
    std::string bin_filename;
    int session = 0;
    int csv_fd = -1;
    int64_t csv_committed = 0; // size after the last complete row
    std::ofstream bin_file;
    std::ofstream packet_file;
    std::ofstream state_file;

    uint64_t total_points = 0;

    // Timestamp order guards: SLAM expects IMU and point times to increase
    // in file order, so any regression is reported rather than silently written.
    bool has_last_imu = false;
    int64_t last_imu_timestamp = 0;
    int64_t last_imu_epoch = 0;
    uint64_t non_increasing_imu = 0;
    bool has_last_point = false;
    double last_point_time = 0;
    uint64_t decreasing_points = 0;

    void CloseCsv()
    {
        if (csv_fd < 0)
            return;
        CrashTruncateFd() = -1;
        fsync(csv_fd);
        close(csv_fd);
        csv_fd = -1;
    }

public:
    ~ContinuousDataWriter() { CloseCsv(); }

    int SessionIndex() const { return session; }

    // Starts a recording in `dir`: imuNNNN.csv and the points temp file, with
    // NNNN the next free session number.
    bool Open(const std::string &dir)
    {
        out_dir = dir;
        session = NextSessionIndex(dir);
        csv_filename = JoinPath(dir, ImuCsvName(session));
        bin_filename = JoinPath(dir, PointsTempName(session));

        // 1. IMU CSV, written with plain write() calls: every write is a
        // whole number of rows, and a failed one is cut back.
        csv_fd = open(csv_filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        if (csv_fd < 0 || !WriteAll(csv_fd, kImuCsvHeader, strlen(kImuCsvHeader)))
        {
            std::cerr << "Error: Could not open CSV for continuous writing.\n";
            return false;
        }
        csv_committed = (int64_t)strlen(kImuCsvHeader);
        CrashTruncateSize() = csv_committed;
        CrashTruncateFd() = csv_fd;

        // 2. Raw binary file for the point cloud
        bin_file.open(bin_filename, std::ios::out | std::ios::binary | std::ios::trunc);
        if (!bin_file.is_open())
        {
            std::cerr << "Error: Could not open temporary binary file.\n";
            return false;
        }
        WritePointsTempHeader(bin_file);
        bin_file.flush();

        // 3. Packet timing log (diagnostics only; a failure is not fatal)
        packet_file.open(JoinPath(dir, PacketLogName(session)), std::ios::out | std::ios::binary | std::ios::trunc);
        if (packet_file.is_open())
            packet_file.write(kPacketLogMagic, sizeof kPacketLogMagic);
        else
            std::cerr << "WARNING: could not open " << PacketLogName(session) << ", packet timing not logged." << std::endl;

        // 4. Point packet state and calibration (not fatal either)
        state_file.open(JoinPath(dir, PacketStateName(session)), std::ios::out | std::ios::binary | std::ios::trunc);
        if (state_file.is_open())
        {
            state_file << kPacketStateHeader;
            state_file.flush();
        }
        else
            std::cerr << "WARNING: could not open " << PacketStateName(session) << ", packet state not logged." << std::endl;

        return true;
    }

    // Writes timingNNNN.json (via a temp file, so it is never half written).
    void WriteTimingReport(const std::string &json)
    {
        std::string path = JoinPath(out_dir, TimingName(session));
        std::string part = path + ".tmp";
        {
            std::ofstream f(part, std::ios::out | std::ios::trunc);
            f << json;
            if (!f.good())
            {
                std::cerr << "WARNING: could not write " << part << std::endl;
                return;
            }
        }
        std::error_code ec;
        std::filesystem::rename(part, path, ec);
        if (ec)
            std::cerr << "WARNING: could not rename " << part << ": " << ec.message() << std::endl;
    }

    void AppendChunk(const SensorChunk &chunk)
    {
        if (packet_file.is_open() && !chunk.packets.empty())
        {
            packet_file.write(reinterpret_cast<const char *>(chunk.packets.data()), chunk.packets.size() * sizeof(PacketTiming));
            packet_file.flush();
        }
        if (!chunk.timing_report.empty())
            WriteTimingReport(chunk.timing_report);

        if (state_file.is_open() && !chunk.packet_state_rows.empty())
        {
            const std::string &rows = chunk.packet_state_rows;
            state_file.write(rows.data(), (std::streamsize)rows.size());
            state_file.flush();
        }

        // Stream IMU Data
        if (csv_fd >= 0 && !chunk.imu_data.empty())
        {
            uint64_t chunk_non_increasing = 0;
            std::string rows;
            rows.reserve(chunk.imu_data.size() * 160);
            for (const auto &imu : chunk.imu_data)
            {
                if (has_last_imu && (imu.LidarTimestamp <= last_imu_timestamp || imu.EpochTimestamp <= last_imu_epoch))
                {
                    chunk_non_increasing++;
                }
                has_last_imu = true;
                last_imu_timestamp = imu.LidarTimestamp;
                last_imu_epoch = imu.EpochTimestamp;
                AppendImuRow(rows, imu);
            }

            if (WriteAll(csv_fd, rows.data(), rows.size()))
            {
                csv_committed += (int64_t)rows.size();
                CrashTruncateSize() = csv_committed;
            }
            else
            {
                std::cerr << "ERROR: writing IMU rows failed (" << strerror(errno) << "), "
                          << chunk.imu_data.size() << " rows of chunk " << chunk.chunk_index << " lost." << std::endl;
                if (ftruncate(csv_fd, (off_t)csv_committed) != 0 || lseek(csv_fd, 0, SEEK_END) < 0)
                    std::cerr << "ERROR: could not cut the CSV back to its last complete row." << std::endl;
            }

            if (chunk_non_increasing > 0)
            {
                non_increasing_imu += chunk_non_increasing;
                std::cerr << "WARNING: chunk " << chunk.chunk_index << " wrote " << chunk_non_increasing
                          << " non-increasing IMU timestamps (" << non_increasing_imu << " total)." << std::endl;
            }
        }

        // Stream points as raw bytes
        if (bin_file.is_open() && !chunk.points.empty())
        {
            bin_file.write(reinterpret_cast<const char *>(chunk.points.data()), chunk.points.size() * sizeof(PointDLidar));
            bin_file.flush();

            uint64_t chunk_decreasing = 0;
            for (const auto &point : chunk.points)
            {
                if (has_last_point && point.time < last_point_time)
                {
                    chunk_decreasing++;
                }
                has_last_point = true;
                last_point_time = point.time;
            }
            total_points += chunk.points.size();

            if (chunk_decreasing > 0)
            {
                decreasing_points += chunk_decreasing;
                std::cerr << "WARNING: chunk " << chunk.chunk_index << " has " << chunk_decreasing
                          << " point times going backwards (" << decreasing_points << " total)." << std::endl;
            }
        }
    }

    // Closes both streams and converts the points into lidarNNNN.laz. If that
    // fails, the temp file stays and is finalized on the next start.
    bool CloseAndPackageLAZ()
    {
        CloseCsv();
        if (bin_file.is_open())
            bin_file.close();
        if (packet_file.is_open())
            packet_file.close();
        if (state_file.is_open())
            state_file.close();

        std::cout << "\nTimestamp check: " << non_increasing_imu << " non-increasing IMU timestamps, "
                  << decreasing_points << " backward point times." << std::endl;

        std::cout << "\nCapture finished. " << total_points << " points recorded." << std::endl;
        if (!FinalizePointsTemp(bin_filename, out_dir, session))
            return false;

        std::cout << "Packaging complete." << std::endl;
        return true;
    }
};
