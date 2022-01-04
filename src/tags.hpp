#pragma once

#include <glm/glm.hpp>

struct InstanceData{
    glm::mat4 transform;
    glm::vec3 color;
    glm::vec3 scale;
};

struct Color{
    glm::vec3 value;
};

struct SkyBoxTag{};
struct SphereTag{};
struct BoxTag{};
struct SceneObjectTag{};
struct Offset{
    glm::vec3 value{0};
};