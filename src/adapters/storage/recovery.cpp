#include "adapters/storage/recovery.h"

#include "adapters/storage/laz_encoder.h"
#include "adapters/storage/recording_files.h"

#include <ctime>

namespace dlidar
{
    // Finalizes one unfinished recording (session `index`): trims the IMU CSV and
    // the packet state TSV to their last complete rows and converts the points
    // temp file into lidarNNNN.laz.
    // With move_aside both go into a recovered_<time> subdirectory.
    bool RecoverSession(const std::string &dir, int index, bool move_aside, const std::vector<RecordingProfile> &profiles)
    {
        std::string temp_path = JoinPath(dir, PointsTempName(index));
        std::string csv_path = JoinPath(dir, ImuCsvName(index));
        struct stat st;
        if (stat(temp_path.c_str(), &st) != 0)
            return true;

        std::cout << "Found an unfinished recording " << index << " in " << dir << ", finalizing it." << std::endl;

        // The sensor that recorded it, from the temp file's header.
        PointsTempLayout layout;
        {
            std::ifstream in(temp_path, std::ios::in | std::ios::binary);
            if (!in.is_open() || !DetectPointsTempLayout(in, (uint64_t)st.st_size, layout))
            {
                std::cerr << "Error: cannot read the header of " << temp_path << std::endl;
                return false;
            }
        }
        const RecordingProfile *profile = FindProfile(profiles, layout.sensor_id);
        if (!profile)
        {
            std::cerr << "Error: " << temp_path << " is from a sensor this build does not know (sensor id "
                      << layout.sensor_id << "); it is kept." << std::endl;
            return false;
        }

        if (access(csv_path.c_str(), F_OK) == 0)
        {
            int64_t removed = TrimPartialCsvLine(csv_path);
            if (removed < 0)
                std::cerr << "WARNING: could not check " << csv_path << " for a partial last row." << std::endl;
            else if (removed > 0)
                std::cout << "Removed a partial last row (" << removed << " bytes) from " << csv_path << std::endl;
        }
        std::string state_path = JoinPath(dir, PacketStateName(index));
        if (access(state_path.c_str(), F_OK) == 0 && TrimPartialLine(state_path, profile->packet_state_header) < 0)
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

        if (!FinalizePointsTemp(temp_path, target, index, profiles))
            return false;
        std::cout << "Recovered recording is in " << target << std::endl;
        return true;
    }

    bool RecoverInterruptedRecording(const std::string &dir, bool move_aside, const std::vector<RecordingProfile> &profiles)
    {
        bool ok = true;
        for (int index : LeftoverSessions(dir))
            ok = RecoverSession(dir, index, move_aside, profiles) && ok;
        return ok;
    }
}
