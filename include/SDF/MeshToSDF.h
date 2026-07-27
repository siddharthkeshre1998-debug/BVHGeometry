#pragma once

#include "BVH/BVH.h"
#include "Mesh/Mesh.h"
#include "SDF/SDFGrid.h"

namespace bvh
{

class MeshToSDF
{
public:

    /**
     * @brief Generate an unsigned distance field from a triangle mesh.
     *
     * The mesh is voxelized into a dense grid. For every voxel centre,
     * the nearest triangle is found using the BVH and the Euclidean
     * distance is stored in the grid.
     *
     * The generated field is currently unsigned.
     * Signed distance generation will be added later.
     *
     * @param mesh Input triangle mesh.
     * @param bvh BVH built over the mesh.
     * @param resolution Number of voxels along the longest axis.
     * @param padding Fractional padding added around the mesh bounding box.
     *
     * @return Dense SDF grid.
     */
    static SDFGrid Generate(
        const Mesh& mesh,
        const BVH& bvh,
        int resolution,
        double padding = 0.05);

private:

    /**
     * @brief Compute the voxel grid dimensions.
     */
    static void ComputeGridDimensions(
        const AABB& bounds,
        int resolution,
        int& nx,
        int& ny,
        int& nz,
        double& voxelSize);

    /**
     * @brief Compute the padded bounding box used for the SDF grid.
     */
    static AABB ComputeGridBounds(
        const Mesh& mesh,
        double padding);
};

} // namespace bvh
