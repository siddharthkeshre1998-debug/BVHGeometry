#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "Math/Ray.h"
#include "Math/Vec3.h"

namespace bvh
{

class AABB
{
public:

    Vec3 min;
    Vec3 max;

public:

    inline double SurfaceArea() const noexcept
    {
        Vec3 e = Extents();

        return 2.0 *
        (
            e.x * e.y +
            e.y * e.z +
            e.z * e.x
        );
    }

    inline int LongestAxis() const noexcept
    {
        Vec3 e = Extents();

        if (e.x >= e.y && e.x >= e.z)
            return 0;

        if (e.y >= e.z)
            return 1;

        return 2;
    }

    inline double DistanceSquared(
        const Vec3& p) const noexcept
    {
        double dx = std::max({min.x - p.x, 0.0, p.x - max.x});
        double dy = std::max({min.y - p.y, 0.0, p.y - max.y});
        double dz = std::max({min.z - p.z, 0.0, p.z - max.z});

        return dx * dx + dy * dy + dz * dz;
    }

    //----------------------------------------------------------------------
    // Ray intersection (returns tNear/tFar)
    //----------------------------------------------------------------------

    inline bool Intersect(
        const Ray& ray,
        double& tNear,
        double& tFar,
        double tMin = 0.0,
        double tMax = std::numeric_limits<double>::max()) const noexcept
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
            {
                std::swap(t0, t1);
            }

            tNear = std::max(tNear, t0);
            tFar  = std::min(tFar,  t1);

            if (tNear > tFar)
            {
                return false;
            }
        }

        return true;
    }

    //----------------------------------------------------------------------
    // Compatibility overload
    //----------------------------------------------------------------------

    inline bool Intersect(
        const Ray& ray,
        double tMin = 0.0,
        double tMax = std::numeric_limits<double>::max()) const noexcept
    {
        double tNear, tFar;

        return Intersect(
            ray,
            tNear,
            tFar,
            tMin,
            tMax);
    }

    //----------------------------------------------------------------------
    // Merge
    //----------------------------------------------------------------------

    static inline AABB Merge(
        const AABB& a,
        const AABB& b) noexcept
    {
        return
        {
            Vec3::Min(a.min, b.min),
            Vec3::Max(a.max, b.max)
        };
    }
};

} // namespace bvh
