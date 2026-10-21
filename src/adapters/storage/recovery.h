#pragma once

#include <string>

namespace dlidar
{
    // Finalizes every recording in `dir` that was not finished (crash, kill, power
    // loss): trims the IMU CSV and the packet state TSV to their last complete rows
    // and converts the points temp file into lidarNNNN.laz. With move_aside each
    // recovered recording is put in a recovered_<time> subdirectory, so a new
    // recording in `dir` does not mix with it. `packet_state_header` is the
    // sensor's diagnostics header (written if trimming empties that file).
    // Returns false if something was found and failed.
    bool RecoverInterruptedRecording(const std::string &dir, bool move_aside, const std::string &packet_state_header);
}
