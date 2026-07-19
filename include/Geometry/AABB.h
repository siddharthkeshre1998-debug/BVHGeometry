#pragma once

#include <algorithm>
#include <limits>

#include "Math/Vec3.h"
#include "Math/Ray.h"

namespace bvh
{

class AABB
{
public:

    Vec3 min;
    Vec3 max;

public:

    AABB()
    {
        Reset();
    }

    AABB(const Vec3& minimum,
         const Vec3& maximum)
        : min(minimum),
          max(maximum)
    {
    }

    //---------------------------------------------------------------------
    // Reset
    //---------------------------------------------------------------------

    void Reset()
    {
        const double inf = std::numeric_limits<double>::infinity();

        min = Vec3( inf,  inf,  inf);
        max = Vec3(-inf, -inf, -inf);
    }

    //---------------------------------------------------------------------
    // Expand
    //---------------------------------------------------------------------

    void Expand(const Vec3& p)
    {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);

        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }

    void Expand(const AABB& box)
    {
        Expand(box.min);
        Expand(box.max);
    }

    //---------------------------------------------------------------------
    // Queries
    //---------------------------------------------------------------------

    Vec3 Center() const
    {
        return (min + max) * 0.5;
    }

    Vec3 Extents() const
    {
        return max - min;
    }

    double SurfaceArea() const
    {
        Vec3 e = Extents();

        return 2.0 *
        (
            e.x * e.y +
            e.y * e.z +
            e.z * e.x
        );
    }

    int LongestAxis() const
    {
        Vec3 e = Extents();

        if (e.x >= e.y && e.x >= e.z)
            return 0;

        if (e.y >= e.z)
            return 1;

        return 2;
    }

    double DistanceSquared(const Vec3& p) const
    {
        double dx = 0.0;
        if (p.x < min.x)
            dx = min.x - p.x;
        else if (p.x > max.x)
            dx = p.x - max.x;

        double dy = 0.0;
        if (p.y < min.y)
            dy = min.y - p.y;
        else if (p.y > max.y)
            dy = p.y - max.y;

        double dz = 0.0;
        if (p.z < min.z)
            dz = min.z - p.z;
        else if (p.z > max.z)
            dz = p.z - max.z;

        return dx * dx + dy * dy + dz * dz;
    }

    //---------------------------------------------------------------------
    // Ray Intersection
    //---------------------------------------------------------------------

    bool Intersect(
        const Ray& ray,
        double& tNear,
        double& tFar,
        double tMin = 0.0,
        double tMax = std::numeric_limits<double>::max()) const
    {
        tNear = tMin;
        tFar  = tMax;

        for (int axis = 0; axis < 3; ++axis)
        {
            double t0 =
                (min[axis] - ray.origin[axis]) *
                ray.invDirection[axis];

            double t1 =
                (max[axis] - ray.origin[axis]) *
                ray.invDirection[axis];

            if (t0 > t1)
                std::swap(t0, t1);

            tNear = std::max(tNear, t0);
            tFar  = std::min(tFar, t1);

            if (tNear > tFar)
                return false;
        }

        return true;
    }

    bool Intersect(
        const Ray& ray,
        double tMin = 0.0,
        double tMax = std::numeric_limits<double>::max()) const
    {
        double tNear, tFar;

        return Intersect(
            ray,
            tNear,
            tFar,
            tMin,
            tMax);
    }

    //---------------------------------------------------------------------
    // Merge
    //---------------------------------------------------------------------

    static AABB Merge(
        const AABB& a,
        const AABB& b)
    {
        AABB box;

        box.min.x = std::min(a.min.x, b.min.x);
        box.min.y = std::min(a.min.y, b.min.y);
        box.min.z = std::min(a.min.z, b.min.z);

        box.max.x = std::max(a.max.x, b.max.x);
        box.max.y = std::max(a.max.y, b.max.y);
        box.max.z = std::max(a.max.z, b.max.z);

        return box;
    }
};

} // namespace bvh
