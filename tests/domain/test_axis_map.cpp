// AxisMap: every signed axis permutation is classified (24 proper rotations,
// 24 mirrors), Apply() is select-and-negate (bits, -0.0 and NaN signs kept),
// and the L2 vertical mount maps exactly as the old VERTICAL code did.
#include "domain/geometry/axis_map.h"
#include "../check.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

using namespace dlidar;

namespace
{
uint32_t Bits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    return u;
}

std::string Text(float f)
{
    char s[64];
    snprintf(s, sizeof s, "%.17f", (double)f); // as the IMU CSV prints it
    return s;
}

void TestClassification()
{
    int proper = 0, mirror = 0, invalid = 0;
    for (int a = 0; a < 6; a++)
        for (int b = 0; b < 6; b++)
            for (int c = 0; c < 6; c++)
            {
                AxisMap m{(Axis)a, (Axis)b, (Axis)c};
                bool distinct = a / 2 != b / 2 && a / 2 != c / 2 && b / 2 != c / 2;
                if (!distinct)
                {
                    invalid++;
                    CHECK(!m.IsProperRotation(), "%s repeats an axis", m.Describe().c_str());
                    continue;
                }
                // Determinant of the matrix whose rows are the picked unit vectors.
                double r[3][3] = {};
                int axes[3] = {a, b, c};
                for (int i = 0; i < 3; i++)
                    r[i][axes[i] / 2] = axes[i] % 2 ? -1 : 1;
                double det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                             r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                             r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
                (det > 0 ? proper : mirror)++;
                CHECK(m.IsProperRotation() == (det > 0), "%s det %g", m.Describe().c_str(), det);
            }
    CHECK(proper == 24 && mirror == 24 && invalid == 216 - 48, "%d proper, %d mirror", proper, mirror);
}

void TestSelectAndNegate()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float specials[] = {0.0f, -0.0f, 1.5f, -2.25f, nan, -nan, 1e-40f, -std::numeric_limits<float>::infinity()};
    for (int a = 0; a < 6; a++)
        for (int b = 0; b < 6; b++)
            for (int c = 0; c < 6; c++)
            {
                AxisMap m{(Axis)a, (Axis)b, (Axis)c};
                if (!m.IsProperRotation())
                    continue;
                for (float s0 : specials)
                    for (float s1 : specials)
                    {
                        float in[3] = {s0, s1, -s0}, out[3];
                        m.Apply(in, out);
                        int axes[3] = {a, b, c};
                        for (int i = 0; i < 3; i++)
                        {
                            float want = axes[i] % 2 ? -in[axes[i] / 2] : in[axes[i] / 2];
                            CHECK(Bits(out[i]) == Bits(want), "%s component %d", m.Describe().c_str(), i);
                        }
                    }
            }
}

// The L2 vertical mount: physical +Z front, +Y left, +X down. The old code
// wrote x = z, y = y, z = -x for points and the same for the IMU (with the
// division by g before the minus).
void TestVerticalL2()
{
    AxisMap m{Axis::PosZ, Axis::PosY, Axis::NegX};
    CHECK(m.IsProperRotation(), "vertical mount must be a rotation");
    CHECK(m.Describe() == "front=+Z left=+Y up=-X", "%s", m.Describe().c_str());

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float values[] = {0.0f, -0.0f, 9.80665f, -9.80665f, 0.0123f, nan, -nan};
    for (float x : values)
        for (float z : values)
        {
            float y = 0.5f;
            // points
            float px = x, py = y, pz = z;
            m.ApplyInPlace(px, py, pz);
            CHECK(Bits(px) == Bits(z) && Bits(py) == Bits(y) && Bits(pz) == Bits(-x), "points x=%g z=%g", x, z);
            // IMU: old = -lin[0] / g; new = mount applied to (lin / g)
            float lin[3] = {x, y, z};
            float old_z = -lin[0] / 9.80665f;
            float g[3] = {lin[0] / 9.80665f, lin[1] / 9.80665f, lin[2] / 9.80665f}, out[3];
            m.Apply(g, out);
            CHECK(Bits(out[2]) == Bits(old_z) && Text(out[2]) == Text(old_z), "imu x=%g: %s vs %s", x,
                  Text(out[2]).c_str(), Text(old_z).c_str());
        }
    CHECK(Text(-0.0f) == "-0.00000000000000000", "-0.0 prints with its sign: %s", Text(-0.0f).c_str());
}
} // namespace

int main()
{
    TestClassification();
    TestSelectAndNegate();
    TestVerticalL2();
    return FinishTests("AxisMap");
}
