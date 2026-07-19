#pragma once

#include <cmath>

#include "Geometry/Triangle.h"
#include "Math/Ray.h"

namespace bvh
{

struct RayHit
{
    bool hit = false;

    double t = 0.0;

    double u = 0.0;
    double v = 0.0;

    Vec3 position;
    Vec3 normal;

    const Triangle* triangle = nullptr;
};

class RayTriangle
{
public:

    static constexpr double EPSILON = 1e-8;

    static inline bool Intersect(
        const Ray& ray,
        const Triangle& triangle,
        RayHit& hit)
    {
        const Vec3 edge1 = triangle.v1 - triangle.v0;
        const Vec3 edge2 = triangle.v2 - triangle.v0;

        const Vec3 pVec =
            Vec3::Cross(ray.direction, edge2);

        const double det =
            Vec3::Dot(edge1, pVec);

        if (std::abs(det) < EPSILON)
        {
            return false;
        }

        const double invDet = 1.0 / det;

        const Vec3 tVec =
            ray.origin - triangle.v0;

        const double u =
            Vec3::Dot(tVec, pVec) * invDet;

        if (u < 0.0 || u > 1.0)
        {
            return false;
        }

        const Vec3 qVec =
            Vec3::Cross(tVec, edge1);

        const double v =
            Vec3::Dot(ray.direction, qVec) * invDet;

        if (v < 0.0 || (u + v) > 1.0)
        {
            return false;
        }

        const double t =
            Vec3::Dot(edge2, qVec) * invDet;

        if (t < EPSILON)
        {
            return false;
        }

        hit.hit = true;
        hit.t = t;
        hit.u = u;
        hit.v = v;

        hit.position = ray.At(t);

        hit.normal =
            Vec3::Cross(edge1, edge2).Normalized();

        hit.triangle = &triangle;

        return true;
    }
};

} // namespace bvh
