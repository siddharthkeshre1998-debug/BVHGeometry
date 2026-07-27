#include "SDF/MeshToSDF.h"

#include <algorithm>
#include <cmath>

namespace bvh
{

//---------------------------------------------------------------------
// Compute padded grid bounds
//---------------------------------------------------------------------

AABB MeshToSDF::ComputeGridBounds(
    const Mesh& mesh,
    double padding)
{
    AABB bounds = mesh.GetBounds();

    Vec3 extent = bounds.Extents();

    Vec3 pad(
        extent.x * padding,
        extent.y * padding,
        extent.z * padding);

    bounds.min = bounds.min - pad;
    bounds.max = bounds.max + pad;

    return bounds;
}

//---------------------------------------------------------------------
// Compute grid dimensions
//---------------------------------------------------------------------

void MeshToSDF::ComputeGridDimensions(
    const AABB& bounds,
    int resolution,
    int& nx,
    int& ny,
    int& nz,
    double& voxelSize)
{
    Vec3 extent = bounds.Extents();

    double longest =
        std::max({extent.x, extent.y, extent.z});

    voxelSize = longest / static_cast<double>(resolution);

    nx = static_cast<int>(std::ceil(extent.x / voxelSize));
    ny = static_cast<int>(std::ceil(extent.y / voxelSize));
    nz = static_cast<int>(std::ceil(extent.z / voxelSize));
}

//---------------------------------------------------------------------
// Generate unsigned distance field
//---------------------------------------------------------------------

SDFGrid MeshToSDF::Generate(
    const Mesh& mesh,
    const BVH& bvh,
    int resolution,
    double padding)
{
    AABB bounds =
        ComputeGridBounds(mesh, padding);

    int nx;
    int ny;
    int nz;

    double voxelSize;

    ComputeGridDimensions(
        bounds,
        resolution,
        nx,
        ny,
        nz,
        voxelSize);

    SDFGrid grid(
        nx,
        ny,
        nz,
        voxelSize,
        bounds.min);

    ClosestTriangleResult result;

    for (int k = 0; k < nz; ++k)
    {
        for (int j = 0; j < ny; ++j)
        {
            for (int i = 0; i < nx; ++i)
            {
                Vec3 point =
                    grid.GridToWorld(i, j, k);

                if (bvh.FindClosestTriangle(
                        point,
                        result))
                {
                    grid.At(i, j, k) =
                        static_cast<float>(
                            std::sqrt(
                                result.distanceSquared));
                }
            }
        }
    }

    return grid;
}

} // namespace bvh
