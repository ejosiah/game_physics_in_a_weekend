#pragma once

#include "shape.hpp"

class ConvexHullShape final : public Shape{
public:
    ConvexHullShape(const std::vector<glm::vec3>& points){
        build(points);
    }

    [[nodiscard]]
    Shape::Type type() const override;

    [[nodiscard]]
    glm::mat3 inertiaTensor() const override;

    [[nodiscard]]
    glm::vec3 centerOfMass() const override;

    [[nodiscard]]
    Bounds bounds(const glm::vec3 &pos, const glm::quat &orient) const override;

    [[nodiscard]]
    Bounds bounds() const override;

    void build(const std::vector<glm::vec3> &points) override;

    [[nodiscard]]
    glm::vec3 support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient, float bias) const override;

    [[nodiscard]]
    float fastLinearSpeed(const glm::vec3 &angularVelocity, const glm::vec3 &dir) const override;

private:
    Bounds m_bounds;
    std::vector<glm::vec3> m_points;
    glm::mat3 tensor;
};