#include "BVH/BVH.h"

#include <algorithm>
#include <numeric>

namespace bvh
{

namespace
{
struct BuildInfo
{
    AABB primitiveBounds;
    AABB centroidBounds;
};
}

BVH::BVH(Mesh& mesh)
    : m_mesh(mesh)
{
}

void BVH::Build()
{
    m_triangleIndices.resize(m_mesh.TriangleCount());
    std::iota(m_triangleIndices.begin(), m_triangleIndices.end(), 0);

    if (m_triangleIndices.empty())
    {
        m_root.reset();
        return;
    }

    m_root = BuildRecursive(0, static_cast<int>(m_triangleIndices.size()));
}

static BuildInfo ComputeBuildInfoImpl(const Mesh& mesh,
                                      const std::vector<int>& indices,
                                      int start,
                                      int end)
{
    BuildInfo info;

    for (int i = start; i < end; ++i)
    {
        const Triangle& t = mesh[indices[i]];
        info.primitiveBounds.Expand(t.bounds);
        info.centroidBounds.Expand(t.centroid);
    }

    return info;
}

AABB BVH::ComputeBounds(int start, int end) const
{
    return ComputeBuildInfoImpl(m_mesh, m_triangleIndices, start, end).primitiveBounds;
}

AABB BVH::ComputeCentroidBounds(int start, int end) const
{
    return ComputeBuildInfoImpl(m_mesh, m_triangleIndices, start, end).centroidBounds;
}

int BVH::LongestAxis(const AABB& bounds) const
{
    return bounds.LongestAxis();
}

int BVH::MedianSplit(int start, int end, int axis)
{
    int mid = start + (end - start) / 2;

    std::nth_element(
        m_triangleIndices.begin() + start,
        m_triangleIndices.begin() + mid,
        m_triangleIndices.begin() + end,
        [&](int a, int b)
        {
            return m_mesh[a].centroid[axis] < m_mesh[b].centroid[axis];
        });

    return mid;
}

std::unique_ptr<BVHNode> BVH::BuildRecursive(int start, int end)
{
    auto node = std::make_unique<BVHNode>();

    BuildInfo info = ComputeBuildInfoImpl(
        m_mesh,
        m_triangleIndices,
        start,
        end);

    node->bounds = info.primitiveBounds;

    int count = end - start;

    if (count <= MaxLeafTriangles)
    {
        node->MakeLeaf(start, count);
        return node;
    }

    int axis = LongestAxis(info.centroidBounds);
    int mid = MedianSplit(start, end, axis);

    node->left = BuildRecursive(start, mid);
    node->right = BuildRecursive(mid, end);

    return node;
}

} // namespace bvh
