#include "bounds.hpp"

Bounds::Bounds() {
    clear();
}

void Bounds::clear() {
    min = glm::vec3(std::numeric_limits<float>::max());
    max = glm::vec3(std::numeric_limits<float>::lowest());
}

bool Bounds::intersect(const Bounds &rhs) const {
    return !glm::any(glm::lessThan(max, rhs.min)) || !glm::any(glm::lessThan(rhs.max, min));
}

void Bounds::expand(const std::vector<glm::vec3>& points) {
    for(const auto& point : points){
        expand(point);
    }
}

void Bounds::expand(const glm::vec3 &rhs) {
    max = glm::max(max, rhs);
    min = glm::min(min, rhs);
}

void Bounds::expand(const Bounds &rhs) {
    expand(rhs.min);
    expand(rhs.max);
}

float Bounds::widthX() const {
    return max.x - min.x;
}

float Bounds::widthY() const {
    return max.y - min.y;
}

float Bounds::widthZ() const {
    return max.z - min.z;
}
