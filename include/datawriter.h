#pragma once

#include "unitree_lidar_sdk.h"
#include "d_lidar_util.h"
#include <dll/laszip_api.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <algorithm>
#include <cstdio> // For std::remove

using namespace unitree_lidar_sdk;

struct OutputImuData
{
    uint64_t LidarTimestamp;
    uint64_t EpochTimestamp;
    int ImuId;

    float GyroX;
    float GyroY;
    float GyroZ;

    float AccelerationX;
    float AccelerationY;
    float AccelerationZ;
};

struct SensorChunk
{
    int chunk_index;
    std::vector<PointDLidar> points;
    std::vector<OutputImuData> imu_data;
};

class ContinuousDataWriter
{
private:
    std::ofstream csv_file;
    std::ofstream bin_file;
    std::string bin_filename;
    std::string final_laz_filename;

    uint32_t total_points = 0;
    double max_x{std::numeric_limits<double>::lowest()};
    double max_y{std::numeric_limits<double>::lowest()};
    double max_z{std::numeric_limits<double>::lowest()};
    double min_x{std::numeric_limits<double>::max()};
    double min_y{std::numeric_limits<double>::max()};
    double min_z{std::numeric_limits<double>::max()};

public:
    bool Open(const std::string &laz_filename, const std::string &csv_filename)
    {
        final_laz_filename = laz_filename;
        // Create a temporary binary file path next to the LAZ file
        bin_filename = laz_filename.substr(0, laz_filename.find_last_of('.')) + "_temp.bin";

        // 1. Open CSV for continuous IMU
        csv_file.open(csv_filename, std::ios::out);
        if (csv_file.is_open())
        {
            csv_file << "gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix\n";
        }
        else
        {
            std::cerr << "Error: Could not open CSV for continuous writing.\n";
            return false;
        }

        // 2. Open raw binary file for Point Cloud
        bin_file.open(bin_filename, std::ios::out | std::ios::binary);
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
        if (csv_file.is_open())
        {
            for (const auto &imu : chunk.imu_data)
            {
                csv_file << std::fixed << std::setprecision(17)
                         << imu.GyroX << " " << imu.GyroY << " " << imu.GyroZ << " "
                         << imu.AccelerationX << " " << imu.AccelerationY << " " << imu.AccelerationZ << " "
                         << imu.ImuId << " " << imu.LidarTimestamp << " " << imu.EpochTimestamp << "\n";
            }
        }

        // Stream Points and Calculate Bounds dynamically
        if (bin_file.is_open() && !chunk.points.empty())
        {
            // Write vector directly to disk as raw bytes (highly efficient)
            bin_file.write(reinterpret_cast<const char *>(chunk.points.data()), chunk.points.size() * sizeof(PointDLidar));

            for (const auto &point : chunk.points)
            {
                max_x = std::max(max_x, (double)point.x);
                min_x = std::min(min_x, (double)point.x);
                max_y = std::max(max_y, (double)point.y);
                min_y = std::min(min_y, (double)point.y);
                max_z = std::max(max_z, (double)point.z);
                min_z = std::min(min_z, (double)point.z);
            }
            total_points += chunk.points.size();
        }
    }

    void CloseAndPackageLAZ()
    {
        if (csv_file.is_open())
            csv_file.close();
        if (bin_file.is_open())
            bin_file.close();

        std::cout << "\nCapture finished. Packaging " << total_points << " points into LAZ format..." << std::endl;

        // ==========================================
        // STAGE 2: Execute your original working LAZ logic
        // ==========================================
        laszip_POINTER laszip_writer;
        if (laszip_create(&laszip_writer))
        {
            std::cerr << "DLL ERROR: creating laszip writer\n";
            return;
        }

        laszip_header *header;
        if (laszip_get_header_pointer(laszip_writer, &header))
        {
            std::cerr << "DLL ERROR: getting header pointer\n";
            return;
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

        // Inject the perfectly calculated totals before opening!
        header->number_of_point_records = total_points;
        header->number_of_points_by_return[0] = total_points;
        header->max_x = max_x;
        header->min_x = min_x;
        header->max_y = max_y;
        header->min_y = min_y;
        header->max_z = max_z;
        header->min_z = min_z;

        laszip_BOOL compress = (strstr(final_laz_filename.c_str(), ".laz") != 0);
        if (laszip_open_writer(laszip_writer, final_laz_filename.c_str(), compress))
        {
            std::cerr << "DLL ERROR: opening laszip writer\n";
            return;
        }

        laszip_point *laszipPoint;
        if (laszip_get_point_pointer(laszip_writer, &laszipPoint))
        {
            std::cerr << "DLL ERROR: getting point pointer\n";
            return;
        }

        // Stream from raw binary into the LAS encoder
        std::ifstream bin_in(bin_filename, std::ios::in | std::ios::binary);
        if (bin_in.is_open())
        {
            PointDLidar pt;
            laszip_F64 coordinates[3];

            while (bin_in.read(reinterpret_cast<char *>(&pt), sizeof(PointDLidar)))
            {
                laszipPoint->intensity = pt.intensity;
                laszipPoint->gps_time = pt.time;
                laszipPoint->user_data = pt.ring;

                coordinates[0] = pt.x;
                coordinates[1] = pt.y;
                coordinates[2] = pt.z;

                if (laszip_set_coordinates(laszip_writer, coordinates) || laszip_write_point(laszip_writer))
                {
                    std::cerr << "DLL ERROR: Failed writing point during packaging.\n";
                    break;
                }
            }
            bin_in.close();
        }

        if (laszip_close_writer(laszip_writer))
        {
            std::cerr << "DLL ERROR: closing laszip writer\n";
        }
        laszip_destroy(laszip_writer);

        // Safely delete the temporary binary file
        std::remove(bin_filename.c_str());

        std::cout << "Packaging complete. Continuous LAZ file ready." << std::endl;
    }
};