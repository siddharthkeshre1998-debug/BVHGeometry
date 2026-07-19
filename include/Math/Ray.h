#pragma once

#include "Vec3.h"

#include <limits>

namespace bvh
{

/**
 * @brief Represents a 3D ray.
 *
 * The ray stores the inverse direction and sign bits to accelerate
 * ray-AABB intersection during BVH traversal.
 */
class Ray
{
public:

    Vec3 origin;
    Vec3 direction;

    // Cached inverse direction
    Vec3 invDirection;

    // Sign of each inverse direction component
    bool sign[3];

public:

    Ray() noexcept
        : origin(),
          direction(0.0, 0.0, 1.0)
    {
        ComputeCache();
    }

    Ray(const Vec3& o,
        const Vec3& d) noexcept
        : origin(o),
          direction(d)
    {
        direction.Normalize();
        ComputeCache();
    }

    /**
     * @brief Returns a point on the ray.
     */
    inline Vec3 At(double t) const noexcept
    {
        return origin + direction * t;
    }

private:

    inline void ComputeCache() noexcept
    {
        constexpr double INF =
            std::numeric_limits<double>::infinity();

        invDirection.x =
            (std::abs(direction.x) > 1e-15)
                ? 1.0 / direction.x
                : INF;

        invDirection.y =
            (std::abs(direction.y) > 1e-15)
                ? 1.0 / direction.y
                : INF;

        invDirection.z =
            (std::abs(direction.z) > 1e-15)
                ? 1.0 / direction.z
                : INF;

        sign[0] = invDirection.x < 0.0;
        sign[1] = invDirection.y < 0.0;
        sign[2] = invDirection.z < 0.0;
    }
};

} // namespace bvh
