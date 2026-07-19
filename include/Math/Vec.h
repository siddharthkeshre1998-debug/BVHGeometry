#pragma once

#include <cmath>
#include <ostream>
#include <algorithm>

namespace bvh
{

class Vec3
{
public:

    double x;
    double y;
    double z;

public:

    //--------------------------------------------------------------------------
    // Constructors
    //--------------------------------------------------------------------------

    constexpr Vec3() noexcept
        : x(0.0), y(0.0), z(0.0)
    {
    }

    constexpr Vec3(double xx, double yy, double zz) noexcept
        : x(xx), y(yy), z(zz)
    {
    }

    //--------------------------------------------------------------------------
    // Element Access
    //--------------------------------------------------------------------------

    constexpr double& operator[](int index) noexcept
    {
        return (&x)[index];
    }

    constexpr const double& operator[](int index) const noexcept
    {
        return (&x)[index];
    }

    //--------------------------------------------------------------------------
    // Unary Operators
    //--------------------------------------------------------------------------

    constexpr Vec3 operator+() const noexcept
    {
        return *this;
    }

    constexpr Vec3 operator-() const noexcept
    {
        return {-x, -y, -z};
    }

    //--------------------------------------------------------------------------
    // Arithmetic Operators
    //--------------------------------------------------------------------------

    constexpr Vec3 operator+(const Vec3& rhs) const noexcept
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr Vec3 operator-(const Vec3& rhs) const noexcept
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr Vec3 operator*(double s) const noexcept
    {
        return {x * s, y * s, z * s};
    }

    constexpr Vec3 operator/(double s) const noexcept
    {
        return {x / s, y / s, z / s};
    }

    friend constexpr Vec3 operator*(double s, const Vec3& v) noexcept
    {
        return v * s;
    }

    //--------------------------------------------------------------------------
    // Compound Assignment
    //--------------------------------------------------------------------------

    Vec3& operator+=(const Vec3& rhs) noexcept
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(double s) noexcept
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vec3& operator/=(double s) noexcept
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    //--------------------------------------------------------------------------
    // Comparison
    //--------------------------------------------------------------------------

    constexpr bool operator==(const Vec3& rhs) const noexcept
    {
        return x == rhs.x &&
               y == rhs.y &&
               z == rhs.z;
    }

    constexpr bool operator!=(const Vec3& rhs) const noexcept
    {
        return !(*this == rhs);
    }

    //--------------------------------------------------------------------------
    // Geometry
    //--------------------------------------------------------------------------

    constexpr double LengthSquared() const noexcept
    {
        return x * x + y * y + z * z;
    }

    double Length() const noexcept
    {
        return std::sqrt(LengthSquared());
    }

    void Normalize() noexcept
    {
        double len = Length();

        if (len > 0.0)
        {
            (*this) /= len;
        }
    }

    Vec3 Normalized() const noexcept
    {
        double len = Length();

        if (len == 0.0)
        {
            return {};
        }

        return (*this) / len;
    }

    //--------------------------------------------------------------------------
    // Static Math Functions
    //--------------------------------------------------------------------------

    static constexpr double Dot(
        const Vec3& a,
        const Vec3& b) noexcept
    {
        return a.x * b.x +
               a.y * b.y +
               a.z * b.z;
    }

    static constexpr Vec3 Cross(
        const Vec3& a,
        const Vec3& b) noexcept
    {
        return
        {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    static constexpr Vec3 Min(
        const Vec3& a,
        const Vec3& b) noexcept
    {
        return
        {
            std::min(a.x, b.x),
            std::min(a.y, b.y),
            std::min(a.z, b.z)
        };
    }

    static constexpr Vec3 Max(
        const Vec3& a,
        const Vec3& b) noexcept
    {
        return
        {
            std::max(a.x, b.x),
            std::max(a.y, b.y),
            std::max(a.z, b.z)
        };
    }
};

//------------------------------------------------------------------------------
// Stream Output
//------------------------------------------------------------------------------

inline std::ostream&
operator<<(std::ostream& os,
           const Vec3& v)
{
    os << "("
       << v.x << ", "
       << v.y << ", "
       << v.z << ")";

    return os;
}

} // namespace bvh
