#pragma once

#include <memory>
#include "shape.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct Body{
    glm::vec3 position{0};
    glm::quat orientation{1, 0, 0, 0};
    glm::vec3 linearVelocity{0};
    glm::vec3 angularVelocity{0};
    float invMass{0};
    float elasticity{1};
    float friction{0.5};
    std::shared_ptr<Shape> shape;

    [[nodiscard]]
    glm::vec3 centerOfMassWorldSpace() const;

    [[nodiscard]]
    glm::vec3 centerOfMassModelSpace() const;

    [[nodiscard]]
    glm::vec3 worldSpaceToBodySpace(const glm::vec3& pt) const;

    [[nodiscard]]
    glm::vec3 bodySpaceToWorldSpace(const glm::vec3& pt) const;

    [[nodiscard]]
    glm::mat3 inverseInertialTensorBodySpace() const;

    [[nodiscard]]
    glm::mat3 inverseInertialTensorWorldSpace() const;

    void applyImpulseLinear( const glm::vec3& impulse);

    void applyImpulseAngular(const glm::vec3& impulse);

    void applyImpulse(const glm::vec3& impulsePoint, const glm::vec3& impulse);

    void update(float dt);

    bool hasInfiniteMass() const;
};