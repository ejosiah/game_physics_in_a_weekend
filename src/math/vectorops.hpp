#pragma once

#include <glm/glm.hpp>

inline void orthonormal(glm::vec3& a, glm::vec3& b, glm::vec3 c){
    auto n = glm::normalize(a);

    const auto w = (n.z * n.z > 0.9f * 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);

    b = glm::cross(w, n);
    b = glm::normalize(b);

    c = glm::cross(n, b);
    c = glm::normalize(c);
    b = glm::cross(c, n);
    b = glm::normalize(b);
}