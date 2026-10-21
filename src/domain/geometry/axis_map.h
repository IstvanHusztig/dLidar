#pragma once

#include <stdint.h>
#include <string>

namespace dlidar
{
    // One sensor axis with a sign.
    enum class Axis : uint8_t
    {
        PosX,
        NegX,
        PosY,
        NegY,
        PosZ,
        NegZ
    };

    // How the sensor is mounted on the device: which sensor axis points to the
    // device's front (output +X), left (+Y) and up (+Z). Only 90-degree mounts
    // (signed axis permutations) are possible. Apply() only selects and negates
    // components - never multiplies - so a value keeps its exact bits, including
    // the sign of zero and of NaN (the IMU CSV prints -0.0 as "-0.000...").
    // Points and IMU go through the same AxisMap, so they always agree.
    struct AxisMap
    {
        Axis front = Axis::PosX;
        Axis left = Axis::PosY;
        Axis up = Axis::PosZ;

        // A proper rotation: three different axes, determinant +1 (a mirrored
        // mapping would flip the handedness of the point cloud and the gyro).
        bool IsProperRotation() const
        {
            int a = Index(front), b = Index(left), c = Index(up);
            if (a == b || a == c || b == c)
                return false;
            // Sign of the permutation (a, b, c) of (0, 1, 2) times the axis signs.
            int inversions = (a > b) + (a > c) + (b > c);
            int sign = (inversions % 2 ? -1 : 1) * Sign(front) * Sign(left) * Sign(up);
            return sign == 1;
        }

        // out = (front, left, up) components of the sensor-frame vector `in`.
        template <typename T>
        void Apply(const T in[3], T out[3]) const
        {
            T x = Pick(in, front), y = Pick(in, left), z = Pick(in, up);
            out[0] = x;
            out[1] = y;
            out[2] = z;
        }

        template <typename T>
        void ApplyInPlace(T &x, T &y, T &z) const
        {
            const T in[3] = {x, y, z};
            T out[3];
            Apply(in, out);
            x = out[0];
            y = out[1];
            z = out[2];
        }

        // "front=+Z left=+Y up=-X"
        std::string Describe() const
        {
            return "front=" + Name(front) + " left=" + Name(left) + " up=" + Name(up);
        }

        static int Index(Axis a) { return (int)a / 2; }
        static int Sign(Axis a) { return (int)a % 2 ? -1 : 1; }

    private:
        template <typename T>
        static T Pick(const T in[3], Axis a)
        {
            T v = in[Index(a)];
            return Sign(a) < 0 ? -v : v;
        }

        static std::string Name(Axis a)
        {
            return std::string(Sign(a) < 0 ? "-" : "+") + "XYZ"[Index(a)];
        }
    };
}
