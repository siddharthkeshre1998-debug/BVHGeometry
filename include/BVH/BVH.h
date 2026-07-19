#pragma once

#include <memory>
#include <vector>

#include "BVH/BVHNode.h"
#include "Geometry/Triangle.h"
#include "Math/Ray.h"
#include "Math/Vec3.h"
#include "Mesh/Mesh.h"

namespace bvh
{

class BVH
{
public:

    /**
     * Maximum number of triangles allowed inside a leaf node.
     */
    static constexpr int MaxLeafTriangles = 4;

public:

    explicit BVH(Mesh& mesh);

    /**
     * Builds the BVH.
     */
    void Build();

    /**
     * Returns true if the ray intersects the mesh.
     */
    bool Intersect(const Ray& ray) const;

    /**
     * Returns the closest triangle to the query point.
     */
    const Triangle* FindClosestTriangle(
        const Vec3& point,
        double& distanceSquared) const;

private:

    //----------------------------------------------------------------------
    // Mesh
    //----------------------------------------------------------------------

    Mesh& m_mesh;

    //----------------------------------------------------------------------
    // Root Node
    //----------------------------------------------------------------------

    std::unique_ptr<BVHNode> m_root;

    //----------------------------------------------------------------------
    // Triangle Indices
    //----------------------------------------------------------------------

    std::vector<int> m_triangleIndices;

private:

    //----------------------------------------------------------------------
    // Build
    //----------------------------------------------------------------------

    std::unique_ptr<BVHNode> BuildRecursive(
        int start,
        int end);

    //----------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------

    AABB ComputeBounds(
        int start,
        int end) const;

    int LongestAxis(
        const AABB& bounds) const;

    void MedianSplit(
        int start,
        int end,
        int axis);

    //----------------------------------------------------------------------
    // Traversal
    //----------------------------------------------------------------------

    bool IntersectRecursive(
        const BVHNode* node,
        const Ray& ray) const;

    void FindClosestRecursive(
        const BVHNode* node,
        const Vec3& point,
        const Triangle*& closestTriangle,
        double& bestDistanceSquared) const;
};

}
