#pragma once

#include "domain/recording/recording_profile.h"

#include <string>
#include <vector>

namespace dlidar
{
    // Finalizes every recording in `dir` that was not finished (crash, kill, power
    // loss): trims the IMU CSV and the packet state TSV to their last complete rows
    // and converts the points temp file into lidarNNNN.laz. With move_aside each
    // recovered recording is put in a recovered_<time> subdirectory, so a new
    // recording in `dir` does not mix with it. Each is finished as the sensor
    // that recorded it: the profile in `profiles` whose id its points temp file
    // names (its packetstate header is written if trimming empties that file).
    // Returns false if something was found and failed.
    bool RecoverInterruptedRecording(const std::string &dir, bool move_aside, const std::vector<RecordingProfile> &profiles);
}
