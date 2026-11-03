#pragma once

namespace dlidar
{
    // Which measured points are recorded. Policy for point-cloud clarity, the
    // same for every sensor; the values come from the sensor's defaults (its
    // blind zone, its intensity scale) unless the device configures others.
    // A point is kept if min_range_m <= range <= max_range_m and
    // intensity >= min_intensity. `range` is the measured range, not |xyz|
    // (the L2's axis offsets make those differ). Comparisons are on floats,
    // as the L2 conversion always did; NaN is never rejected.
    struct PointFilter
    {
        float min_range_m = 0.0f;
        float max_range_m = 100.0f;
        float min_intensity = 0.0f;

        bool Accepts(float range_m, float intensity) const
        {
            if (range_m < min_range_m || range_m > max_range_m)
                return false;
            return !(intensity < min_intensity);
        }
    };
}
