#pragma once

#include "unitree_lidar_sdk.h"
#include "imu_timestamper.h"
#include "shutdown.h"
#include <dll/laszip_api.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <limits>
#include <algorithm>
#include <cerrno>
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
const char *const kImuCsvName = "imu.csv";
const char *const kPointsTempName = "points_temp.bin"; // raw PointDLidar records while recording
const char *const kLazPrefix = "lidar";                // lidar0001.laz, lidar0002.laz, ...
const uint64_t kMaxPointsPerLaz = 20000000;            // ~5 min of L2 points per LAZ chunk
const char *const kImuCsvHeader = "gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix\n";

struct SensorChunk
{
    int chunk_index;
    std::vector<PointDLidar> points;
    std::vector<OutputImuData> imu_data;
};

inline std::string JoinPath(const std::string &dir, const std::string &name)
{
    return (std::filesystem::path(dir) / name).string();
}

inline std::string LazChunkName(int index)
{
    char name[32];
    snprintf(name, sizeof name, "%s%04d.laz", kLazPrefix, index);
    return name;
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

// Cuts a CSV back to its last complete line (a crash or power loss can leave
// a partial one). Returns the number of bytes removed, or -1 on error.
inline int64_t TrimPartialCsvLine(const std::string &csv_path)
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
        ok = WriteAll(fd, kImuCsvHeader, strlen(kImuCsvHeader));
    close(fd);
    return ok ? removed : -1;
}

// Removes lidarNNNN.laz chunks numbered above `keep`: leftovers of an earlier
// recording in the same directory, which the new one replaces.
inline void RemoveStaleLazChunks(const std::string &dir, int keep)
{
    std::error_code ec;
    for (const auto &entry : std::filesystem::directory_iterator(dir, ec))
    {
        std::string name = entry.path().filename().string();
        int index = 0;
        char tail = 0;
        size_t prefix = strlen(kLazPrefix);
        if (name.size() == prefix + 8 && name.compare(0, prefix, kLazPrefix) == 0 &&
            sscanf(name.c_str() + prefix, "%4d.la%c", &index, &tail) == 2 && tail == 'z' &&
            name == LazChunkName(index) && index > keep)
        {
            std::cout << "Removing " << name << " left over from an earlier recording." << std::endl;
            std::filesystem::remove(entry.path(), ec);
        }
    }
}

// Encodes `count` points starting at record `first` of the temp file into a
// LAS 1.2 / point format 1 LAZ file. The header needs the bounds before any
// point is written, so the range is read twice.
inline bool WriteLazChunk(std::ifstream &in, uint64_t first, uint64_t count, const std::string &laz_path)
{
    const size_t kBlock = 65536;
    std::vector<PointDLidar> block(kBlock);

    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x, min_z = min_x, max_z = max_x;
    in.clear();
    in.seekg((std::streamoff)(first * sizeof(PointDLidar)));
    for (uint64_t done = 0; done < count;)
    {
        size_t n = (size_t)std::min<uint64_t>(kBlock, count - done);
        if (!in.read(reinterpret_cast<char *>(block.data()), n * sizeof(PointDLidar)))
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

    if (laszip_open_writer(laszip_writer, laz_path.c_str(), 1))
    {
        std::cerr << "DLL ERROR: opening laszip writer for " << laz_path << "\n";
        laszip_destroy(laszip_writer);
        return false;
    }

    laszip_point *laszipPoint;
    bool ok = !laszip_get_point_pointer(laszip_writer, &laszipPoint);

    in.clear();
    in.seekg((std::streamoff)(first * sizeof(PointDLidar)));
    laszip_F64 coordinates[3];
    for (uint64_t done = 0; ok && done < count;)
    {
        size_t n = (size_t)std::min<uint64_t>(kBlock, count - done);
        if (!in.read(reinterpret_cast<char *>(block.data()), n * sizeof(PointDLidar)))
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

// Converts a points temp file into lidar0001.laz, lidar0002.laz, ... in
// out_dir (max_points_per_file points each). A trailing partial record (from
// an interrupted write) is ignored. Each chunk is written to a .tmp file and
// renamed when complete; the temp file is deleted only after all chunks are
// in place, so an interrupted finalization is simply redone next time.
inline bool FinalizePointsTemp(const std::string &temp_path, const std::string &out_dir,
                               uint64_t max_points_per_file = kMaxPointsPerLaz)
{
    std::error_code ec;
    uint64_t bytes = std::filesystem::file_size(temp_path, ec);
    if (ec)
    {
        std::cerr << "Error: cannot read " << temp_path << ": " << ec.message() << std::endl;
        return false;
    }
    uint64_t total = bytes / sizeof(PointDLidar);
    if (bytes % sizeof(PointDLidar))
    {
        std::cerr << "WARNING: ignoring " << bytes % sizeof(PointDLidar)
                  << " trailing bytes (partial point) in " << temp_path << std::endl;
    }

    std::ifstream in(temp_path, std::ios::in | std::ios::binary);
    if (!in.is_open())
    {
        std::cerr << "Error: cannot open " << temp_path << std::endl;
        return false;
    }

    int files = (int)((total + max_points_per_file - 1) / max_points_per_file);
    std::cout << "Packaging " << total << " points into " << files << " LAZ file(s) in " << out_dir << " ..." << std::endl;
    for (int f = 0; f < files; f++)
    {
        uint64_t first = (uint64_t)f * max_points_per_file;
        uint64_t count = std::min<uint64_t>(max_points_per_file, total - first);
        std::string laz_path = JoinPath(out_dir, LazChunkName(f + 1));
        std::string part_path = laz_path + ".tmp";
        if (!WriteLazChunk(in, first, count, part_path))
        {
            std::cerr << "Error: packaging " << laz_path << " failed; " << temp_path
                      << " is kept and will be finalized on the next start." << std::endl;
            std::filesystem::remove(part_path, ec);
            return false;
        }
        std::filesystem::rename(part_path, laz_path, ec);
        if (ec)
        {
            std::cerr << "Error: cannot rename " << part_path << ": " << ec.message() << std::endl;
            return false;
        }
        std::cout << "  " << laz_path << ": " << count << " points" << std::endl;
    }
    in.close();

    RemoveStaleLazChunks(out_dir, files);
    std::filesystem::remove(temp_path, ec);
    return true;
}

// If the last recording in `dir` was not finalized (crash, kill, power loss),
// trims the IMU CSV to its last complete row and converts the points temp
// file into lidar*.laz. With move_aside the recovered recording is put in a
// recovered_<time> subdirectory, so a new recording in `dir` does not
// overwrite it. Returns false only if something was found and failed.
inline bool RecoverInterruptedRecording(const std::string &dir, bool move_aside,
                                        uint64_t max_points_per_file = kMaxPointsPerLaz)
{
    std::string temp_path = JoinPath(dir, kPointsTempName);
    std::string csv_path = JoinPath(dir, kImuCsvName);
    struct stat st;
    if (stat(temp_path.c_str(), &st) != 0)
        return true;

    std::cout << "Found an unfinished recording in " << dir << ", finalizing it." << std::endl;

    if (access(csv_path.c_str(), F_OK) == 0)
    {
        int64_t removed = TrimPartialCsvLine(csv_path);
        if (removed < 0)
            std::cerr << "WARNING: could not check " << csv_path << " for a partial last row." << std::endl;
        else if (removed > 0)
            std::cout << "Removed a partial last row (" << removed << " bytes) from " << csv_path << std::endl;
    }

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
        if (access(csv_path.c_str(), F_OK) == 0)
        {
            std::filesystem::rename(csv_path, JoinPath(target, kImuCsvName), ec);
            if (ec)
            {
                std::cerr << "Error: cannot move " << csv_path << ": " << ec.message() << std::endl;
                return false;
            }
        }
    }

    if (!FinalizePointsTemp(temp_path, target, max_points_per_file))
        return false;
    std::cout << "Recovered recording is in " << target << std::endl;
    return true;
}

class ContinuousDataWriter
{
private:
    std::string out_dir;
    std::string csv_filename;
    std::string bin_filename;
    int csv_fd = -1;
    int64_t csv_committed = 0; // size after the last complete row
    std::ofstream bin_file;

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

    // Starts a recording in `dir`: imu.csv and the points temp file.
    bool Open(const std::string &dir)
    {
        out_dir = dir;
        csv_filename = JoinPath(dir, kImuCsvName);
        bin_filename = JoinPath(dir, kPointsTempName);

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

        return true;
    }

    void AppendChunk(const SensorChunk &chunk)
    {
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

    // Closes both streams and converts the points into lidar*.laz. If that
    // fails, the temp file stays and is finalized on the next start.
    bool CloseAndPackageLAZ()
    {
        CloseCsv();
        if (bin_file.is_open())
            bin_file.close();

        std::cout << "\nTimestamp check: " << non_increasing_imu << " non-increasing IMU timestamps, "
                  << decreasing_points << " backward point times." << std::endl;

        std::cout << "\nCapture finished. " << total_points << " points recorded." << std::endl;
        if (!FinalizePointsTemp(bin_filename, out_dir))
            return false;

        std::cout << "Packaging complete." << std::endl;
        return true;
    }
};
