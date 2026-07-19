#pragma once

#include <limits>

#include "Geometry/Triangle.h"

namespace bvh
{

struct ClosestPointResult
{
    Vec3 point;
    double distanceSquared;
    double u;
    double v;
    double w;
};

class ClosestPointTriangle
{
public:

    /**
     * Ericson's Closest Point on Triangle
     * (Real-Time Collision Detection)
     */
    static inline ClosestPointResult Compute(
        const Vec3& p,
        const Triangle& tri)
    {
        const Vec3& a = tri.v0;
        const Vec3& b = tri.v1;
        const Vec3& c = tri.v2;

        const Vec3 ab = b - a;
        const Vec3 ac = c - a;
        const Vec3 ap = p - a;

        double d1 = Vec3::Dot(ab, ap);
        double d2 = Vec3::Dot(ac, ap);

        //--------------------------------------------------------------------------
        // Vertex Region A
        //--------------------------------------------------------------------------

        if (d1 <= 0.0 && d2 <= 0.0)
        {
            return {
                a,
                (p - a).LengthSquared(),
                1.0, 0.0, 0.0
            };
        }

        const Vec3 bp = p - b;

        double d3 = Vec3::Dot(ab, bp);
        double d4 = Vec3::Dot(ac, bp);

        //--------------------------------------------------------------------------
        // Vertex Region B
        //--------------------------------------------------------------------------

        if (d3 >= 0.0 && d4 <= d3)
        {
            return {
                b,
                (p - b).LengthSquared(),
                0.0, 1.0, 0.0
            };
        }

        //--------------------------------------------------------------------------
        // Edge AB
        //--------------------------------------------------------------------------

        double vc = d1 * d4 - d3 * d2;

        if (vc <= 0.0 &&
            d1 >= 0.0 &&
            d3 <= 0.0)
        {
            double v = d1 / (d1 - d3);

            Vec3 point = a + ab * v;

            return {
                point,
                (p - point).LengthSquared(),
                1.0 - v,
                v,
                0.0
            };
        }

        const Vec3 cp = p - c;

        double d5 = Vec3::Dot(ab, cp);
        double d6 = Vec3::Dot(ac, cp);

        //--------------------------------------------------------------------------
        // Vertex Region C
        //--------------------------------------------------------------------------

        if (d6 >= 0.0 &&
            d5 <= d6)
        {
            return {
                c,
                (p - c).LengthSquared(),
                0.0, 0.0, 1.0
            };
        }

        //--------------------------------------------------------------------------
        // Edge AC
        //--------------------------------------------------------------------------

        double vb = d5 * d2 - d1 * d6;

        if (vb <= 0.0 &&
            d2 >= 0.0 &&
            d6 <= 0.0)
        {
            double w = d2 / (d2 - d6);

            Vec3 point = a + ac * w;

            return {
                point,
                (p - point).LengthSquared(),
                1.0 - w,
                0.0,
                w
            };
        }

        //--------------------------------------------------------------------------
        // Edge BC
        //--------------------------------------------------------------------------

        double va = d3 * d6 - d5 * d4;

        if (va <= 0.0 &&
            (d4 - d3) >= 0.0 &&
            (d5 - d6) >= 0.0)
        {
            Vec3 bc = c - b;

            double w =
                (d4 - d3) /
                ((d4 - d3) + (d5 - d6));

            Vec3 point = b + bc * w;

            return {
                point,
                (p - point).LengthSquared(),
                0.0,
                1.0 - w,
                w
            };
        }

        //--------------------------------------------------------------------------
        // Face Region
        //--------------------------------------------------------------------------

        double denom = 1.0 / (va + vb + vc);

        double v = vb * denom;
        double w = vc * denom;
        double u = 1.0 - v - w;

        Vec3 point =
            a * u +
            b * v +
            c * w;

        return
        {
            point,
            (p - point).LengthSquared(),
            u,
            v,
            w
        };
    }
};

} // namespace bvh
