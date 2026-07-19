#pragma once

#include <cmath>
#include <limits>

#include "Geometry/Triangle.h"
#include "Math/Ray.h"

namespace bvh
{

/**
 * @brief Result of a ray-triangle intersection.
 */
struct RayHit
{
    /**
     * Was there an intersection?
     */
    bool hit = false;

    /**
     * Distance along the ray.
     *
     * Initialized to infinity so that
     * closer intersections automatically replace it.
     */
    double t = std::numeric_limits<double>::max();

    /**
     * Barycentric coordinates.
     *
     * w = 1 - u - v
     */
    double u = 0.0;
    double v = 0.0;

    /**
     * World-space hit position.
     */
    Vec3 position;

    /**
     * Face normal.
     */
    Vec3 normal;

    /**
     * Pointer to the intersected triangle.
     */
    const Triangle* triangle = nullptr;

    /**
     * Index of the triangle inside the mesh.
     */
    int triangleIndex = -1;

    /**
     * Reset the structure for another query.
     */
    void Reset()
    {
        hit = false;
        t = std::numeric_limits<double>::max();

        u = 0.0;
        v = 0.0;

        position = Vec3();

        normal = Vec3();

        triangle = nullptr;
        // triangleIndex is assigned by BVH traversal.

        return true;
    }
};

} // namespace bvh
