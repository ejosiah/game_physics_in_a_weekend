#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "intersectiontest.hpp"
#include "bounds.hpp"

class Shape{
public:
    enum class Type : uint32_t {
        SPHERE, BOX, CONVEX_HULL
    };

    virtual Type type() const = 0;

    [[nodiscard]]
    virtual glm::mat3 inertiaTensor() const = 0;

    [[nodiscard]]
    virtual glm::vec3 centerOfMass() const { return m_centerOfMass; }

    [[nodiscard]]
    virtual Bounds bounds(const glm::vec3& pos, const glm::quat& orient) const = 0;

    [[nodiscard]]
    virtual Bounds bounds() const = 0;

    virtual void build(const std::vector<glm::vec3>& points){}

    [[nodiscard]]
    virtual glm::vec3 support(const glm::vec3& dir, const glm::vec3& pos, const glm::quat& orient, float bias) const = 0;

    [[nodiscard]]
    virtual float fastLinearSpeed(const glm::vec3& angularVelocity, const glm::vec3& dir) const {
        return 0.0f;
    }

protected:
    glm::vec3 m_centerOfMass{0};
};

class SphereShape final : public Shape{
public:
    friend class GameWorld;
    friend class ObjectBuilder;
    explicit SphereShape(float radius) : m_radius(radius){}

    [[nodiscard]]
    Type type() const final {
        return Type::SPHERE;
    }

    [[nodiscard]]
    glm::mat3 SphereShape::inertiaTensor() const {
        return glm::mat3(2.0f * m_radius * m_radius / 5.0f);
    }

    [[nodiscard]]
    Bounds bounds(const glm::vec3 &pos, const glm::quat &orient) const final;

    [[nodiscard]]
    Bounds bounds() const final;

    [[nodiscard]]
    glm::vec3 support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient,  float bias) const final;

    float m_radius{1};
};

class BoxShape final : public Shape{
public:
    BoxShape(const std::vector<glm::vec3>& points){
        build(points);
    }

    [[nodiscard]]
    inline Type type() const final {
        return Shape::Type::BOX;
    }

     [[nodiscard]]
     glm::mat3 inertiaTensor() const final;

    [[nodiscard]]
    glm::vec3 centerOfMass() const final;

    [[nodiscard]]
    Bounds bounds(const glm::vec3 &pos, const glm::quat &orient) const final;

    [[nodiscard]]
    Bounds bounds() const final;

    void build(const std::vector<glm::vec3> &points) final;

    [[nodiscard]]
    glm::vec3 support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient, float bias) const final;

    [[nodiscard]]
    float fastLinearSpeed(const glm::vec3 &angularVelocity, const glm::vec3 &dir) const final;

public:
    Bounds m_bounds;
    std::vector<glm::vec3> m_points;
};