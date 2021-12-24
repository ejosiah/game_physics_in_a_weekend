#include "shape.hpp"

Bounds SphereShape::bounds(const glm::vec3 &pos, const glm::quat &orient) const {
    Bounds bounds;
    bounds.expand(glm::vec3(-m_radius) + pos);
    bounds.expand(glm::vec3(m_radius) + pos);
    return bounds;
}

Bounds SphereShape::bounds() const {
    Bounds bounds;
    bounds.expand(glm::vec3(-m_radius));
    bounds.expand(glm::vec3(m_radius) );
    return bounds;
}