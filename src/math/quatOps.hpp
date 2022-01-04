#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

inline glm::vec3 rotatePoint(const glm::quat& q, const glm::vec3& p){
    return q * p * glm::inverse(q);
}