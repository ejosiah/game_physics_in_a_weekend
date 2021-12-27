#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include "shape.hpp"

inline bool raySphere(const glm::vec3 rayStart, const glm::vec3 rayDir, const glm::vec3& sphereCenter, const float sphereRadius, float& t1, float& t2){
    const auto m = sphereCenter - rayStart;
    const auto a = glm::dot(rayDir, rayDir);
    const auto b = glm::dot(m, rayDir);
    const auto c = glm::dot(m, m) - sphereRadius * sphereRadius;

    const auto delta = b * b - a * c;
    const auto invA = 1.0f/a;

    if(delta < 0){
        return false;
    }

    const auto deltaRoot = sqrtf(delta);
    t1 = invA * (b - deltaRoot);
    t2 = invA * (b + deltaRoot);

    return true;
}