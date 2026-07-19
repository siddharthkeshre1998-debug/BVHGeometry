#pragma once

#include <cmath>
#include <ostream>

namespace bvh
{

class Vec3
{
public:

    double x;
    double y;
    double z;

public:

    constexpr Vec3()
        : x(0.0), y(0.0), z(0.0)
    {}

    constexpr Vec3(double xx, double yy, double zz)
        : x(xx), y(yy), z(zz)
    {}

    constexpr Vec3 operator+(const Vec3& rhs) const
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr Vec3 operator-(const Vec3& rhs) const
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr Vec3 operator*(double s) const
    {
        return {x * s, y * s, z * s};
    }

    constexpr Vec3 operator/(double s) const
    {
        return {x / s, y / s, z / s};
    }

    Vec3& operator+=(const Vec3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr bool operator==(const Vec3& rhs) const
    {
        return x == rhs.x &&
               y == rhs.y &&
               z == rhs.z;
    }

    constexpr bool operator!=(const Vec3& rhs) const
    {
        return !(*this == rhs);
    }

    double Length() const
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    double LengthSquared() const
    {
        return x*x + y*y + z*z;
    }

    Vec3 Normalized() const
    {
        double len = Length();

        if(len == 0.0)
            return {};

        return *this / len;
    }

    static constexpr double Dot(const Vec3& a,
                                const Vec3& b)
    {
        return a.x*b.x +
               a.y*b.y +
               a.z*b.z;
    }

    static constexpr Vec3 Cross(const Vec3& a,
                                const Vec3& b)
    {
        return
        {
            a.y*b.z - a.z*b.y,
            a.z*b.x - a.x*b.z,
            a.x*b.y - a.y*b.x
        };
    }
};

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

}
