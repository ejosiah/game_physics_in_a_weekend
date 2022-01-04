#pragma once

#include "shape.hpp"

inline bool sphereSphereStatic(const SphereShape* sphereA, const SphereShape* sphereB, const glm::vec3& posA
        , const glm::vec3& posB, const glm::vec3& velA, const glm::vec3& velB
        , glm::vec3& pointOnA, glm::vec3& pointOnB){

    const auto ab = posB - posA;
    const auto normal = glm::normalize(ab);
    pointOnA = posA + normal * sphereA->m_radius;
    pointOnB = posB + normal * sphereB->m_radius;

    const auto radiusAB = sphereA->m_radius + sphereB->m_radius;

    return glm::dot(ab, ab) <= (radiusAB * radiusAB);
}

inline bool sphereSphereDynamic(const SphereShape* sphereA, const SphereShape* sphereB, const glm::vec3& posA
        , const glm::vec3& posB, const glm::vec3& velA, const glm::vec3& velB, const float dt
        , glm::vec3& pointOnA, glm::vec3& pointOnB, float& timeOfImpact){
    const auto relVelocity = velA - velB;

    const auto startPointA = posA;
    const auto endPointA = posA + relVelocity * dt;
    const auto rayDir = endPointA - startPointA;

    float t0 = 0;
    float t1 = 0;

    if(glm::dot(rayDir, rayDir) < 0.000001){
        // Ray is too short, just check if already intersecting
        auto ab = posB - posA;
        float radius = sphereA->m_radius + sphereB->m_radius + 0.001f;
        if(glm::dot(ab, ab) > radius * radius){
            return false;
        }
    }else if (!raySphere(posA, rayDir, posB, sphereA->m_radius + sphereB->m_radius, t0, t1)){
        return false;
    }

    // Change from [0, 1] range to [0, dt] range
    t0 *= dt;
    t1 *= dt;

    // if the collision is only in the past, then there's not future collision this frame
    if(t1 < 0.0f){
        return false;
    }

    // Get the earliest positive time of impact;
    timeOfImpact = (t0 < 0.0f) ? 0.0f : t0;

    // if the earliest collision is too far in the future, then there's no collision this frame
    if(timeOfImpact > dt){
        return false;
    }

    // Get the points on the respective points of collision and return true
    auto newPosA = posA + velA * timeOfImpact;
    auto newPosB = posB + velB * timeOfImpact;
    auto ab = glm::normalize(newPosB - newPosA);
    pointOnA = newPosA + ab * sphereA->m_radius;
    pointOnB = newPosB - ab * sphereB->m_radius;
    return true;
}