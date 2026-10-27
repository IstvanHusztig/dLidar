#pragma once

// Writes one recording: imuNNNN.csv, points_tempNNNN.bin -> lidarNNNN.laz,
// timingNNNN.json, packetsNNNN.bin and (if the sensor has diagnostics)
// packetstateNNNN.tsv (docs/l2-recording-format.md).
#include "adapters/storage/crash_guard.h"
#include "adapters/storage/laz_encoder.h"
#include "adapters/storage/recording_files.h"
#include "domain/recording/packet_timing.h"
#include "domain/recording/recording_profile.h"
#include "domain/recording/sensor_chunk.h"

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

namespace dlidar
{
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
        RecordingProfile profile;                // the sensor that records
        std::vector<RecordingProfile> profiles; // every known sensor, to finalize a temp file

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
        // `profiles` lists every sensor whose temp files can be finalized; it
        // must contain `profile` (default: only `profile`).
        explicit ContinuousDataWriter(const RecordingProfile &profile, std::vector<RecordingProfile> profiles = {})
            : profile(profile), profiles(profiles.empty() ? std::vector<RecordingProfile>{profile} : std::move(profiles)) {}
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
            if (*profile.packet_state_header)
            {
                state_file.open(JoinPath(dir, PacketStateName(session)), std::ios::out | std::ios::binary | std::ios::trunc);
                if (state_file.is_open())
                {
                    state_file << profile.packet_state_header;
                    state_file.flush();
                }
                else
                    std::cerr << "WARNING: could not open " << PacketStateName(session) << ", packet state not logged." << std::endl;
            }

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
                bin_file.write(reinterpret_cast<const char *>(chunk.points.data()), chunk.points.size() * sizeof(PointRecord));
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
            if (!FinalizePointsTemp(bin_filename, out_dir, session, profiles))
                return false;

            std::cout << "Packaging complete." << std::endl;
            return true;
        }
    };
}
