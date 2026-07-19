#pragma once

#include <memory>
#include <vector>
#include <limits>

#include "Algorithms/RayTriangle.h"
#include "BVH/BVHNode.h"
#include "Geometry/Triangle.h"
#include "Math/Ray.h"
#include "Math/Vec3.h"
#include "Mesh/Mesh.h"

namespace bvh
{

/**
 * @brief Result returned by the nearest triangle query.
 */
struct ClosestTriangleResult
{
    const Triangle* triangle = nullptr;

    int triangleIndex = -1;

    Vec3 closestPoint;

    double distanceSquared =
        std::numeric_limits<double>::max();

    // Barycentric coordinates
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
};

class BVH
{
public:

    static constexpr int MaxLeafTriangles = 4;

public:

    explicit BVH(Mesh& mesh);

    ~BVH() = default;

    //----------------------------------------------------------------------
    // Build
    //----------------------------------------------------------------------

    void Build();

    //----------------------------------------------------------------------
    // Queries
    //----------------------------------------------------------------------

    /**
     * Returns true if the ray intersects the mesh.
     * The closest hit is returned in 'hit'.
     */
    bool Intersect(
        const Ray& ray,
        RayHit& hit) const;

    /**
     * Finds the closest triangle to a point.
     *
     * Returns false if the mesh is empty.
     */
    bool FindClosestTriangle(
        const Vec3& point,
        ClosestTriangleResult& result) const;

private:

    //----------------------------------------------------------------------
    // Build
    //----------------------------------------------------------------------

    std::unique_ptr<BVHNode> BuildRecursive(
        int start,
        int end);

    AABB ComputeBounds(
        int start,
        int end) const;

    AABB ComputeCentroidBounds(
        int start,
        int end) const;

    int LongestAxis(
        const AABB& bounds) const;

    /**
     * Uses std::nth_element to partition the range.
     * Returns the median index.
     */
    int MedianSplit(
        int start,
        int end,
        int axis);

    //----------------------------------------------------------------------
    // Traversal
    //----------------------------------------------------------------------

    bool IntersectRecursive(
        const BVHNode* node,
        const Ray& ray,
        RayHit& closestHit) const;

    void FindClosestRecursive(
        const BVHNode* node,
        const Vec3& point,
        ClosestTriangleResult& result) const;

private:

    //----------------------------------------------------------------------
    // Data
    //----------------------------------------------------------------------

    Mesh& m_mesh;

    std::unique_ptr<BVHNode> m_root;

    /**
     * Triangle permutation used during BVH construction.
     * Triangles in Mesh are never reordered.
     */
    std::vector<int> m_triangleIndices;
};

} // namespace bvh
