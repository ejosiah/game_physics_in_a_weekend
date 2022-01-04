#pragma once

#include "math.hpp"
#include "shape.hpp"
#include "convexhullbuilder.hpp"

class ConvexHullShape final : public Shape{
public:
    explicit ConvexHullShape(const std::vector<glm::vec3>& points = {}){
        if(!points.empty()) {
            build(points);
        }
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

    [[nodiscard]]
    std::vector<uint32_t> indices() const;

    [[nodiscard]]
    std::vector<glm::vec3> vertices() const;

    void build(const std::vector<glm::vec3> &points) override;

    [[nodiscard]]
    glm::vec3 support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient, float bias) const override;

    [[nodiscard]]
    float fastLinearSpeed(const glm::vec3 &angularVelocity, const glm::vec3 &dir) const override;


private:
    Bounds m_bounds;
    std::vector<glm::vec3> m_points;
    glm::mat3 m_inertiaTensor{};
    glm::vec3 m_centerOfMass{0};
    std::vector<Tri> m_hullTris;
};