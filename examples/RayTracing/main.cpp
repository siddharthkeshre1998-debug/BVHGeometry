#include <iostream>

#include "BVH/BVH.h"
#include "Geometry/Triangle.h"
#include "Math/Ray.h"
#include "Math/Vec3.h"
#include "Mesh/Mesh.h"

using namespace bvh;

int main()
{
    //------------------------------------------------------------
    // Create a simple mesh
    //------------------------------------------------------------

    Mesh mesh;

    mesh.AddTriangle(
        Triangle(
            Vec3(-1.0, -1.0, 0.0),
            Vec3( 1.0, -1.0, 0.0),
            Vec3( 0.0,  1.0, 0.0)));

    mesh.AddTriangle(
        Triangle(
            Vec3(-1.0, -1.0, 2.0),
            Vec3( 1.0, -1.0, 2.0),
            Vec3( 0.0,  1.0, 2.0)));

    //------------------------------------------------------------
    // Build BVH
    //------------------------------------------------------------

    BVH tree(mesh);

    tree.Build();

    std::cout << "BVH built successfully.\n";

    //------------------------------------------------------------
    // Ray query
    //------------------------------------------------------------

    Ray ray(
        Vec3(0.0, 0.0, -5.0),
        Vec3(0.0, 0.0, 1.0));

    RayHit hit;

    if (tree.Intersect(ray, hit))
    {
        std::cout << "\nRay Hit\n";

        std::cout << "Triangle Index : "
                  << hit.triangleIndex << '\n';

        std::cout << "Distance : "
                  << hit.t << '\n';

        std::cout << "Position : "
                  << hit.position.x << ", "
                  << hit.position.y << ", "
                  << hit.position.z << '\n';
    }
    else
    {
        std::cout << "\nRay Missed\n";
    }

    //------------------------------------------------------------
    // Closest Triangle Query
    //------------------------------------------------------------

    ClosestTriangleResult result;

    Vec3 queryPoint(0.25, 0.25, 1.1);

    if (tree.FindClosestTriangle(queryPoint, result))
    {
        std::cout << "\nNearest Triangle\n";

        std::cout << "Triangle Index : "
                  << result.triangleIndex << '\n';

        std::cout << "Distance : "
                  << std::sqrt(result.distanceSquared)
                  << '\n';

        std::cout << "Closest Point : "
                  << result.closestPoint.x << ", "
                  << result.closestPoint.y << ", "
                  << result.closestPoint.z << '\n';
    }

    return 0;
}
