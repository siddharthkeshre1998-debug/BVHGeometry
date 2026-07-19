#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "Algorithms/RayTriangle.h"
#include "BVH/BVHNode.h"
#include "Geometry/Triangle.h"
#include "Math/Ray.h"
#include "Math/Vec3.h"
#include "Mesh/Mesh.h"

namespace bvh
{

struct ClosestTriangleResult
{
    const Triangle* triangle = nullptr;

    int triangleIndex = -1;

    Vec3 closestPoint;

    double distanceSquared =
        std::numeric_limits<double>::max();

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

    bool Intersect(
        const Ray& ray,
        RayHit& hit) const;

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

    int MedianSplit(
        int start,
        int end,
        int axis);

    //----------------------------------------------------------------------
    // Traversal
    //----------------------------------------------------------------------

    /**
     * Traverses the BVH and updates the closest hit.
     *
     * tMax always stores the closest hit distance found so far.
     * Nodes farther than tMax are skipped.
     */
    void IntersectRecursive(
        const BVHNode* node,
        const Ray& ray,
        double& tMax,
        RayHit& closestHit) const;

    /**
     * Traverses the BVH looking for the nearest triangle.
     */
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
     * Permutation array used during BVH construction.
     * The mesh itself is never reordered.
     */
    std::vector<int> m_triangleIndices;
};

} // namespace bvh
