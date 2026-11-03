// PointFilter keeps exactly the points the L2 conversion's filter kept:
// `range < min || range > max` and `intensity(uint8) < min_intensity(uint8)`
// rejected, everything else (including NaN ranges) kept.
#include "domain/geometry/point_filter.h"
#include "../check.h"

#include <cmath>
#include <limits>
#include <stdint.h>

using namespace dlidar;

namespace
{
bool Legacy(float range, uint8_t intensity, float range_min, float range_max, uint8_t min_intensity)
{
    if (range < range_min || range > range_max)
        return false;
    if (intensity < min_intensity)
        return false;
    return true;
}

void TestMatchesLegacy()
{
    const float kMin = 0.1f, kMax = 100;
    const uint8_t kMinIntensity = 5;
    PointFilter f{kMin, kMax, (float)kMinIntensity};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float ranges[] = {0.0f,  -0.0f, 0.0999999f, kMin, std::nextafter(kMin, 1.0f), 50.0f, kMax,
                            std::nextafter(kMax, 1e9f), 1e9f, nan, -1.0f, std::numeric_limits<float>::infinity()};
    for (float r : ranges)
        for (int i = 0; i < 256; i++)
            CHECK(f.Accepts(r, (float)(uint8_t)i) == Legacy(r, (uint8_t)i, kMin, kMax, kMinIntensity),
                  "range %.9g intensity %d", r, i);
}

void TestDefaultsKeepEverythingInRange()
{
    PointFilter f;
    CHECK(f.Accepts(0.0f, 0.0f) && f.Accepts(100.0f, 0.0f) && !f.Accepts(100.5f, 255.0f), "defaults");
}
} // namespace

int main()
{
    TestMatchesLegacy();
    TestDefaultsKeepEverythingInRange();
    return FinishTests("PointFilter");
}
