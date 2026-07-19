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

bool BVH::Intersect(const Ray& ray, RayHit& hit) const
{
    hit.Reset();

    if (!m_root)
        return false;

    double tMax = std::numeric_limits<double>::max();
    IntersectRecursive(m_root.get(), ray, tMax, hit);

    return hit.hit;
}

void BVH::IntersectRecursive(
    const BVHNode* node,
    const Ray& ray,
    double& tMax,
    RayHit& closestHit) const
{
    if (!node)
        return;

    double tNear, tFar;
    if (!node->bounds.Intersect(ray, tNear, tFar, 0.0, tMax))
        return;

    if (node->IsLeaf())
    {
        for (int i = 0; i < node->triangleCount; ++i)
        {
            int triIndex = m_triangleIndices[node->firstTriangle + i];
            const Triangle& tri = m_mesh[triIndex];

            RayHit localHit;
            if (RayTriangle::Intersect(ray, tri, localHit) && localHit.t < tMax)
            {
                tMax = localHit.t;
                localHit.triangleIndex = triIndex;
                closestHit = localHit;
            }
        }
        return;
    }

    double lNear=0,lFar=0,rNear=0,rFar=0;
    bool hitLeft=node->left && node->left->bounds.Intersect(ray,lNear,lFar,0.0,tMax);
    bool hitRight=node->right && node->right->bounds.Intersect(ray,rNear,rFar,0.0,tMax);

    if(hitLeft && hitRight)
    {
        if(lNear < rNear)
        {
            IntersectRecursive(node->left.get(),ray,tMax,closestHit);
            IntersectRecursive(node->right.get(),ray,tMax,closestHit);
        }
        else
        {
            IntersectRecursive(node->right.get(),ray,tMax,closestHit);
            IntersectRecursive(node->left.get(),ray,tMax,closestHit);
        }
    }
    else if(hitLeft)
        IntersectRecursive(node->left.get(),ray,tMax,closestHit);
    else if(hitRight)
        IntersectRecursive(node->right.get(),ray,tMax,closestHit);
}

bool BVH::FindClosestTriangle(const Vec3& point,
                              ClosestTriangleResult& result) const
{
    result = ClosestTriangleResult{};
    if(!m_root) return false;
    FindClosestRecursive(m_root.get(), point, result);
    return result.triangle != nullptr;
}

void BVH::FindClosestRecursive(
    const BVHNode* node,
    const Vec3& point,
    ClosestTriangleResult& result) const
{
    if(!node) return;

    if(node->bounds.DistanceSquared(point) > result.distanceSquared)
        return;

    if(node->IsLeaf())
    {
        for(int i=0;i<node->triangleCount;++i)
        {
            int triIndex=m_triangleIndices[node->firstTriangle+i];
            const Triangle& tri=m_mesh[triIndex];

            auto cp=ClosestPointTriangle::Compute(point,tri);

            if(cp.distanceSquared < result.distanceSquared)
            {
                result.triangle=&tri;
                result.triangleIndex=triIndex;
                result.closestPoint=cp.point;
                result.distanceSquared=cp.distanceSquared;
                result.u=cp.u;
                result.v=cp.v;
                result.w=cp.w;
            }
        }
        return;
    }

    double leftDist = node->left ? node->left->bounds.DistanceSquared(point)
                                 : std::numeric_limits<double>::max();
    double rightDist = node->right ? node->right->bounds.DistanceSquared(point)
                                   : std::numeric_limits<double>::max();

    if(leftDist < rightDist)
    {
        FindClosestRecursive(node->left.get(),point,result);
        FindClosestRecursive(node->right.get(),point,result);
    }
    else
    {
        FindClosestRecursive(node->right.get(),point,result);
        FindClosestRecursive(node->left.get(),point,result);
    }
}

} // namespace bvh
