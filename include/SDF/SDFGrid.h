#pragma once

#include <cassert>
#include <vector>

#include "Math/Vec3.h"

namespace bvh
{

class SDFGrid
{
public:

    SDFGrid() = default;

    SDFGrid(
        int nx,
        int ny,
        int nz,
        double voxelSize,
        const Vec3& origin)
        : m_nx(nx),
          m_ny(ny),
          m_nz(nz),
          m_voxelSize(voxelSize),
          m_origin(origin),
          m_values(static_cast<size_t>(nx) * ny * nz, 0.0f)
    {
    }

    //---------------------------------------------------------------------
    // Grid Dimensions
    //---------------------------------------------------------------------

    int GetNx() const
    {
        return m_nx;
    }

    int GetNy() const
    {
        return m_ny;
    }

    int GetNz() const
    {
        return m_nz;
    }

    double GetVoxelSize() const
    {
        return m_voxelSize;
    }

    const Vec3& GetOrigin() const
    {
        return m_origin;
    }

    //---------------------------------------------------------------------
    // Element Access
    //---------------------------------------------------------------------

    float& At(
        int i,
        int j,
        int k)
    {
        assert(i >= 0 && i < m_nx);
        assert(j >= 0 && j < m_ny);
        assert(k >= 0 && k < m_nz);

        return m_values[Index(i, j, k)];
    }

    const float& At(
        int i,
        int j,
        int k) const
    {
        assert(i >= 0 && i < m_nx);
        assert(j >= 0 && j < m_ny);
        assert(k >= 0 && k < m_nz);

        return m_values[Index(i, j, k)];
    }

    //---------------------------------------------------------------------
    // Coordinate Conversion
    //---------------------------------------------------------------------

    Vec3 GridToWorld(
        int i,
        int j,
        int k) const
    {
        return Vec3(
            m_origin.x + (i + 0.5) * m_voxelSize,
            m_origin.y + (j + 0.5) * m_voxelSize,
            m_origin.z + (k + 0.5) * m_voxelSize);
    }
    
    Vec3 WorldToGrid(
        const Vec3& point) const
    {
        return Vec3(
            (point.x - m_origin.x) / m_voxelSize,
            (point.y - m_origin.y) / m_voxelSize,
            (point.z - m_origin.z) / m_voxelSize);
    }

    bool IsValidIndex(
        int i,
        int j,
        int k) const
    {
        return
            i >= 0 && i < m_nx &&
            j >= 0 && j < m_ny &&
            k >= 0 && k < m_nz;
    }

    //---------------------------------------------------------------------
    // Raw Storage
    //---------------------------------------------------------------------

    std::vector<float>& Data()
    {
        return m_values;
    }

    const std::vector<float>& Data() const
    {
        return m_values;
    }

private:

    size_t Index(
        int i,
        int j,
        int k) const
    {
        return static_cast<size_t>(k) * m_ny * m_nx +
               static_cast<size_t>(j) * m_nx +
               static_cast<size_t>(i);
    }

private:

    int m_nx = 0;
    int m_ny = 0;
    int m_nz = 0;

    double m_voxelSize = 1.0;

    Vec3 m_origin;

    std::vector<float> m_values;
};

} // namespace bvh
