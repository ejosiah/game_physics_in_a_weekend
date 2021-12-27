#include "convexhullshape.hpp"

Shape::Type ConvexHullShape::type() const {
    return Type::CONVEX_HULL;
}

glm::mat3 ConvexHullShape::inertiaTensor() const {
    return glm::mat3();
}

glm::vec3 ConvexHullShape::centerOfMass() const {
    return Shape::centerOfMass();
}

Bounds ConvexHullShape::bounds(const glm::vec3 &pos, const glm::quat &orient) const {
    std::array<glm::vec3, 8> corners{};
    const auto min = m_bounds.min;
    const auto max = m_bounds.max;
    corners[0] = min;
    corners[1] = glm::vec3(min.xy, max.z);
    corners[2] = glm::vec3(min.x, max.y, min.z);
    corners[3] = glm::vec3(max.x, min.yz);

    corners[4] = max;
    corners[5] = glm::vec3(max.xy, min.z);
    corners[6] = glm::vec3(max.x, min.y, max.z);
    corners[7] = glm::vec3(min.x, max.yz);

    Bounds cBounds{};
    auto rotate = glm::mat3(orient);
    for(auto& corner : corners){
        corner = rotate * corner;
        cBounds.expand(corner);
    }

    return cBounds;
}

Bounds ConvexHullShape::bounds() const {
    return Bounds();
}

void ConvexHullShape::build(const std::vector<glm::vec3> &points) {
    Shape::build(points);
}

glm::vec3
ConvexHullShape::support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient, float bias) const {
    return glm::vec3();
}

float ConvexHullShape::fastLinearSpeed(const glm::vec3 &angularVelocity, const glm::vec3 &dir) const {
    auto maxSpeed = std::numeric_limits<float>::min();
    for(const auto& point : m_points){
        auto r = point - m_centerOfMass;
        auto linearVelocity = glm::cross(angularVelocity, r);
        auto speed = glm::dot(dir, linearVelocity);
        maxSpeed = glm::max(speed, maxSpeed);
    }
    return maxSpeed;
}
}