#pragma once

#include "domain/recording/point_record.h"
#include "domain/recording/recording_profile.h"

#include <string>
#include <vector>

namespace dlidar
{
    // The 20 LAZ extra bytes of a point (recording format 2): range, raw_range,
    // sample_index, packet_seq, alpha, theta.
    void PackLazExtraBytes(const PointRecord &pt, unsigned char *out);

    // Converts a points temp file into lidarNNNN.laz (session `index`) in out_dir,
    // described by the profile of the sensor id the temp file names (one of
    // `profiles`; an unknown id keeps the temp file). A trailing partial
    // record (from an interrupted write) is ignored. The LAZ is written to a .tmp
    // file and renamed when complete; the temp file is deleted only after that, so
    // an interrupted finalization is simply redone next time.
    bool FinalizePointsTemp(const std::string &temp_path, const std::string &out_dir, int index,
                            const std::vector<RecordingProfile> &profiles);
}
