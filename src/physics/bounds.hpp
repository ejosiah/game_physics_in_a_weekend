#pragma once

#include <glm/glm.hpp>
#include <limits>
#include <vector>

struct Bounds{
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{std::numeric_limits<float>::min()};

    Bounds();

    void clear();

    [[nodiscard]]
    bool intersect(const Bounds& rhs) const;

    void expand(const std::vector<glm::vec3>& points);

    void expand(const glm::vec3& rhs);

    void expand(const Bounds& rhs);

    [[nodiscard]]
    float widthX() const;

    [[nodiscard]]
    float widthY() const;

    [[nodiscard]]
    float widthZ() const;

};