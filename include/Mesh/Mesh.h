#pragma once

#include <vector>

#include "Geometry/Triangle.h"

namespace bvh
{

/**
 * @brief Triangle mesh used for BVH construction.
 *
 * Owns all triangles.
 * BVH nodes only reference triangles by index.
 */
class Mesh
{
public:

    Mesh() = default;

    //----------------------------------------------------------------------
    // Triangle Storage
    //----------------------------------------------------------------------

    std::vector<Triangle> triangles;

    //----------------------------------------------------------------------
    // Modifiers
    //----------------------------------------------------------------------

    inline void AddTriangle(const Triangle& triangle)
    {
        triangles.push_back(triangle);
    }

    inline void AddTriangle(
        const Vec3& v0,
        const Vec3& v1,
        const Vec3& v2)
    {
        triangles.emplace_back(
            v0,
            v1,
            v2,
            static_cast<int>(triangles.size()));
    }

    inline void Clear()
    {
        triangles.clear();
    }

    //----------------------------------------------------------------------
    // Queries
    //----------------------------------------------------------------------

    inline size_t TriangleCount() const
    {
        return triangles.size();
    }

    inline bool Empty() const
    {
        return triangles.empty();
    }

    inline Triangle& operator[](size_t index)
    {
        return triangles[index];
    }

    inline const Triangle&
    operator[](size_t index) const
    {
        return triangles[index];
    }

    //----------------------------------------------------------------------
    // Mesh Bounds
    //----------------------------------------------------------------------

    AABB ComputeBounds() const
    {
        AABB bounds;

        for (const auto& tri : triangles)
        {
            bounds.Expand(tri.bounds);
        }

        return bounds;
    }
};

} // namespace bvh
