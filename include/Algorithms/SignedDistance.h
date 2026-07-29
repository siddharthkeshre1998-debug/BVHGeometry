#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "BVH/BVH.h" 

namespace bvh
{
bool IsInside(const Vec3& point,
              const BVH& bvh)
{
  //Random direction for now
  Vec3 direction(1.0, 0.3723, 0.6180).Normalized();
  Ray ray(point, direction);

  return (bvh.CountRayIntersections(ray) % 2) == 1;
}
} //namespace bvh
