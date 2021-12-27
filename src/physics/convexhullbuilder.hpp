#pragma once

#include <vector>
#include <array>
#include <glm/glm.hpp>


class ConvexHullBuilder{
public:
    struct Tri{
        int a, b, c;
    };

    struct Edge{
        int a,  b;

        bool operator==(const Edge& rhs) const {
            return (a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a);
        }
    };

    explicit ConvexHullBuilder(const std::vector<glm::vec3>& points);

private:
    static int findPointFurthestInDir(const std::vector<glm::vec3>& points, const glm::vec3& dir);

    static float distanceFromLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point);

    static glm::vec3 findPointFurthestFromLine(const std::vector<glm::vec3>& points, const glm::vec3& pointA, const glm::vec3& pointB);

    static float distanceFromTriangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& point);

    static glm::vec3 findPointFurthestFromTriangle(const std::vector<glm::vec3>& points, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    static void buildTetrahedron(const std::vector<glm::vec3>& vertices, std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris);

    static void expandConvexHull(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris, const std::vector<glm::vec3>& vertices);

    static void removeInternalPoints(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris, std::vector<glm::vec3>& checkPoints);

    static void addPoint(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris, glm::vec3 point);

    static bool isEdgeUnique(const std::vector<Tri>& tris, const std::vector<int>& facingTris, int ignoreTri, const Edge& edge);

    static void removeUnreferencedVertices(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris);

    void build();

private:
    const std::vector<glm::vec3>& m_points;
    std::vector<glm::vec3> m_hullPoints;
    std::vector<Tri> m_hullTris;
};