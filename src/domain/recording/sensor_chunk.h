#pragma once

#include "domain/recording/imu_sample.h"
#include "domain/recording/packet_timing.h"
#include "domain/recording/point_record.h"

#include <string>
#include <vector>

namespace dlidar
{
    // One unit of recording work, handed from the capture to the storage.
    struct SensorChunk
    {
        int chunk_index = 0;
        std::vector<PointRecord> points;
        std::vector<ImuSample> imu_data;
        std::vector<PacketTiming> packets; // timing of the packets of this chunk
        std::string packet_state_rows;     // sensor diagnostics rows (packetstateNNNN.tsv), already formatted
        std::string timing_report;         // if set, (re)written as timingNNNN.json
    };
}
