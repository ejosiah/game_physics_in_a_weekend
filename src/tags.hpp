#pragma once

struct InstanceData{
    glm::mat4 transform;
    glm::vec3 color;
};

struct Color{
    glm::vec3 value;
};

struct SkyBoxTag{};
struct SphereTag{};
struct BoxTag{};
struct SceneObjectTag{};