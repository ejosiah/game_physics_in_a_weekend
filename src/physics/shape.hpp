#pragma once

#include <cstdint>
#include <glm/glm.hpp>

class Shape{
public:
    enum class ShapeType_t : uint32_t {
        SHAPE_SPHERE
    };

    virtual ShapeType_t type() const = 0;

    virtual glm::mat3 inertiaTensor() const = 0;

    virtual glm::vec3 centerOfMass() const { return m_centerOfMass; }
protected:
    glm::vec3 m_centerOfMass{0};
};

class SphereShape final : public Shape{
public:
    friend class GameWorld;
    explicit SphereShape(float radius) : m_radius(radius){}

    [[nodiscard]]
    ShapeType_t type() const final {
        return ShapeType_t::SHAPE_SPHERE;
    }

    glm::mat3 SphereShape::inertiaTensor() const {
        return glm::mat3(2.0f * m_radius * m_radius / 5.0f);
    }

private:
    float m_radius{1};
};