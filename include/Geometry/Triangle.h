#pragma once

#include "Math/Vec3.h"
#include "Geometry/AABB.h"

namespace bvh
{

/**
 * @brief Triangle primitive.
 *
 * The triangle caches its centroid and AABB since they are heavily used during
 * BVH construction. The actual geometric queries (ray intersection and closest
 * point) are implemented in dedicated algorithm headers to keep this class
 * lightweight.
 */
class Triangle
{
public:

    //----------------------------------------------------------------------
    // Vertex Data
    //----------------------------------------------------------------------

    Vec3 v0;
    Vec3 v1;
    Vec3 v2;

    //----------------------------------------------------------------------
    // Cached Data
    //----------------------------------------------------------------------

    AABB bounds;
    Vec3 centroid;

    //----------------------------------------------------------------------
    // Optional application data
    //----------------------------------------------------------------------

    int id;

public:

    //----------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------

    Triangle() noexcept
        : id(-1)
    {
    }

    Triangle(const Vec3& p0,
             const Vec3& p1,
             const Vec3& p2,
             int triangleId = -1) noexcept
        : v0(p0),
          v1(p1),
          v2(p2),
          id(triangleId)
    {
        UpdateCache();
    }

    //----------------------------------------------------------------------
    // Cache Update
    //----------------------------------------------------------------------

    inline void UpdateCache() noexcept
    {
        bounds.Reset();

        bounds.Expand(v0);
        bounds.Expand(v1);
        bounds.Expand(v2);

        centroid =
        (
            v0 +
            v1 +
            v2
        ) / 3.0;
    }

    //----------------------------------------------------------------------
    // Accessors
    //----------------------------------------------------------------------

    inline const AABB& GetBounds() const noexcept
    {
        return bounds;
    }

    inline const Vec3& GetCentroid() const noexcept
    {
        return centroid;
    }

    inline Vec3 Edge0() const noexcept
    {
        return v1 - v0;
    }

    inline Vec3 Edge1() const noexcept
    {
        return v2 - v0;
    }

    inline Vec3 Normal() const noexcept
    {
        Vec3 n =
            Vec3::Cross(
                Edge0(),
                Edge1());

        n.Normalize();

        return n;
    }

    inline double Area() const noexcept
    {
        return 0.5 *
            Vec3::Cross(
                Edge0(),
                Edge1()).Length();
    }

    inline bool Degenerate() const noexcept
    {
        constexpr double EPS = 1e-12;

        return Area() < EPS;
    }
};

} // namespace bvh
