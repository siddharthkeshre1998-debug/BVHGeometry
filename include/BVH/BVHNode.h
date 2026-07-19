#pragma once

#include <memory>

#include "Geometry/AABB.h"

namespace bvh
{

/**
 * @brief Node of a Bounding Volume Hierarchy.
 *
 * Internal nodes contain two children.
 * Leaf nodes store a contiguous range of triangle indices.
 */
class BVHNode
{
public:

    //----------------------------------------------------------------------
    // Bounding Box
    //----------------------------------------------------------------------

    AABB bounds;

    //----------------------------------------------------------------------
    // Children
    //----------------------------------------------------------------------

    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;

    //----------------------------------------------------------------------
    // Triangle Range
    //
    // Leaf nodes reference:
    // triangleIndices[firstTriangle ... firstTriangle + triangleCount - 1]
    //----------------------------------------------------------------------

    int firstTriangle = 0;

    int triangleCount = 0;

public:

    //----------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------

    BVHNode() = default;

    //----------------------------------------------------------------------
    // Queries
    //----------------------------------------------------------------------

    inline bool IsLeaf() const noexcept
    {
        return left == nullptr &&
               right == nullptr;
    }

    inline bool IsInternal() const noexcept
    {
        return !IsLeaf();
    }

    //----------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------

    inline void MakeLeaf(
        int first,
        int count)
    {
        firstTriangle = first;
        triangleCount = count;

        left.reset();
        right.reset();
    }

    inline void MakeInternal(
        std::unique_ptr<BVHNode> leftNode,
        std::unique_ptr<BVHNode> rightNode)
    {
        left = std::move(leftNode);
        right = std::move(rightNode);

        triangleCount = 0;
    }
};

} // namespace bvh
