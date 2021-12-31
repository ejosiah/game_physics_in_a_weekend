#pragma once

#include <glm/glm.hpp>
#include <body.hpp>

struct Contact{
    struct{
        glm::vec3 pointOnA;
        glm::vec3 pointOnB;
    } worldSpace;

    struct{
        glm::vec3 pointOnA;
        glm::vec3 pointOnB;
    } LocalSpace;

    glm::vec3 normal;   //in world space
    float separationDistance{0};   // positive when non-penetrating, negative when penetrating
    float timeOfImpact{0};

    Body* bodyA{nullptr};
    Body* bodyB{nullptr};
};