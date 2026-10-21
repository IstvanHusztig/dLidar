// LAZ encoding of the points temp file: the only code that sees LASzip.
#include "adapters/storage/laz_encoder.h"

#include "adapters/storage/recording_files.h"
#include "domain/recording/recording_info.h"

#include <dll/laszip_api.h>

#include <limits>

namespace dlidar
{
    namespace
    {
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


        // Encodes `count` points starting at record `first` of the temp file into a
        // LAS 1.2 / point format 1 LAZ file, with the format 2 extra bytes if the
        // temp file has them. The header needs the bounds before any point is
        // written, so the range is read twice.
        inline bool WriteLazChunk(std::ifstream &in, const PointsTempLayout &layout, uint64_t first, uint64_t count,
                                  const std::string &laz_path)
        {
            const size_t kBlock = 65536;
            std::vector<PointRecord> block(kBlock);
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
                    const PointRecord &pt = block[i];
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
    }

    void PackLazExtraBytes(const PointRecord &pt, unsigned char *out)
    {
        memcpy(out + 0, &pt.range, 4);
        memcpy(out + 4, &pt.raw_range, 2);
        memcpy(out + 6, &pt.sample_index, 2);
        memcpy(out + 8, &pt.packet_seq, 4);
        memcpy(out + 12, &pt.alpha, 4);
        memcpy(out + 16, &pt.theta, 4);
    }

    bool FinalizePointsTemp(const std::string &temp_path, const std::string &out_dir, int index)
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
}
