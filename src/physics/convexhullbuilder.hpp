#pragma once

#include <vector>
#include <array>
#include <glm/glm.hpp>
#include <algorithm>
#include "math.hpp"

class ConvexHullBuilder{
public:
    friend class ConvexHullShape;

    explicit ConvexHullBuilder(const std::vector<glm::vec3>& points);

private:
    glm::vec3 calculateCenterOfMass();

    glm::mat4 calculateInertiaTensor(const glm::vec3& centerOfMass);

    static void expandConvexHull(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris, const std::vector<glm::vec3>& vertices);

    static void removeInternalPoints(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris, std::vector<glm::vec3>& checkPoints);

    static bool isEdgeUnique(const std::vector<Tri>& tris, const std::vector<int>& facingTris, int ignoreTri, const Edge& edge);

    static void removeUnreferencedVertices(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris);

    static bool isExternal(const std::vector<glm::vec3>& points, const std::vector<Tri>& tris, const glm::vec3& point);

    static void addPoint(std::vector<glm::vec3>& hullPoints, std::vector<Tri>& hullTris, glm::vec3 point);

    void build();

private:
    const std::vector<glm::vec3>& m_points;
    std::vector<glm::vec3> m_hullPoints;
    std::vector<Tri> m_hullTris;
};