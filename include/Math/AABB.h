#pragma once

#include <algorithm>
#include <limits>
#include <cmath>

#include "Math/Vec3.h"
#include "Math/Ray.h"

namespace bvh
{

/**
 * @brief Axis Aligned Bounding Box
 */
class AABB
{
public:

    Vec3 min;
    Vec3 max;

public:

    //----------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------

    AABB() noexcept
    {
        Reset();
    }

    AABB(const Vec3& minimum,
         const Vec3& maximum) noexcept
        : min(minimum),
          max(maximum)
    {
    }

    //----------------------------------------------------------------------
    // Reset
    //----------------------------------------------------------------------

    inline void Reset() noexcept
    {
        constexpr double INF =
            std::numeric_limits<double>::infinity();

        min = { INF, INF, INF };
        max = { -INF, -INF, -INF };
    }

    //----------------------------------------------------------------------
    // Expand
    //----------------------------------------------------------------------

    inline void Expand(const Vec3& p) noexcept
    {
        min = Vec3::Min(min, p);
        max = Vec3::Max(max, p);
    }

    inline void Expand(const AABB& box) noexcept
    {
        Expand(box.min);
        Expand(box.max);
    }

    //----------------------------------------------------------------------
    // Queries
    //----------------------------------------------------------------------

    inline Vec3 Center() const noexcept
    {
        return (min + max) * 0.5;
    }

    inline Vec3 Extents() const noexcept
    {
        return max - min;
    }

    inline double Volume() const noexcept
    {
        Vec3 e = Extents();

        return e.x * e.y * e.z;
    }

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

    inline bool Contains(const Vec3& p) const noexcept
    {
        return
            p.x >= min.x &&
            p.x <= max.x &&
            p.y >= min.y &&
            p.y <= max.y &&
            p.z >= min.z &&
            p.z <= max.z;
    }

    //----------------------------------------------------------------------
    // Squared Distance
    //
    // Used during nearest triangle traversal.
    //----------------------------------------------------------------------

    inline double DistanceSquared(
        const Vec3& p) const noexcept
    {
        double dx = std::max(
            {min.x - p.x, 0.0, p.x - max.x});

        double dy = std::max(
            {min.y - p.y, 0.0, p.y - max.y});

        double dz = std::max(
            {min.z - p.z, 0.0, p.z - max.z});

        return dx * dx +
               dy * dy +
               dz * dz;
    }

    //----------------------------------------------------------------------
    // Ray Intersection
    //
    // Slab algorithm
    //----------------------------------------------------------------------

    inline bool Intersect(
        const Ray& ray,
        double tMin = 0.0,
        double tMax = std::numeric_limits<double>::max()) const noexcept
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            double origin =
                ray.origin[axis];

            double invDir =
                ray.invDirection[axis];

            double t0 =
                (min[axis] - origin) * invDir;

            double t1 =
                (max[axis] - origin) * invDir;

            if (invDir < 0.0)
            {
                std::swap(t0, t1);
            }

            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);

            if (tMax < tMin)
            {
                return false;
            }
        }

        return true;
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
