#pragma once

#include <array>
#include <glm/glm.hpp>
#include "shape.hpp"
#include "body.hpp"
#include "math.hpp"

struct Point{
    glm::vec3 xyz{0}, pointA{0}, pointB{0};

    bool operator==(const Point& rhs) const {
        return (pointA == rhs.pointA) && (pointB == rhs.pointB) && (xyz == rhs.xyz);
    }
};

/**
 * Expanding Polytope Algorithm (EPA)
 */
class EPA{
public:
    static glm::vec3 normalDirection(const Tri& tri, const std::vector<Point>& points);

    static float signedDistanceToTriangle(const Tri& tri, const glm::vec3& point, const std::vector<Point>& points);

    static int closestTriangle(const std::vector<Tri>& triangles, const std::vector<Point>& points);

    static bool hasPoint(const glm::vec3& w, const std::vector<Tri>& triangles, const std::vector<Point>& points);

    static int removeTrianglesFacingPoint(const glm::vec3& point, std::vector<Tri>& triangles, const std::vector<Point>& points);

    static void findDanglingEdges(std::vector<Edge>& danglingEdges, const std::vector<Tri>& triangles);

    static float expand(const Body* bodyA, const Body* bodyB, float bias, const std::array<Point, 4>& simplexPoints, glm::vec3& pointOnA, glm::vec3& pointOnB);
};

/**
 * Gilbert-Johnson-Keerthi (GJK) convex hull intersection algorithm
 */
class GJK{
public:
    static Point support(const Body* bodyA, const Body* bodyB, glm::vec3 dir, float bias = 0.0f);

    static bool simplexSignedVolumes(Point* points, int num, glm::vec3& newDir, glm::vec4& lambdasOut);

    static bool hasPoint(const std::array<Point, 4>& simplexPoints, const Point& newPoint);

    static void sortValids(std::array<Point, 4>& simplexPoints, glm::vec4& lambdas);

    static int numValids(const glm::vec4& lambdas);

    static bool doesIntersect(const Body* bodyA, const Body* bodyB, float bias, glm::vec3& pointOnA, glm::vec3& pointOnB);

    static void closestPoint(const Body* bodyA, const Body* bodyB, glm::vec3& pointOnA, glm::vec3 pointOnB);

};