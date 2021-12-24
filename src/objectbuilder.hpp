#pragma once

#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include "body.hpp"
#include <vulkan_util/Entity.hpp>
#include <vulkan_util/components.h>

struct InstanceData{
    glm::mat4 transform;
    glm::vec3 color;
};

struct Color{
    glm::vec3 value;
};

struct SkyBoxTag{};
struct SphereTag{};

class ObjectBuilder{
public:
    ObjectBuilder(Entity entity, entt::registry* registry = nullptr);

    ObjectBuilder& color(const glm::vec3& co);

    ObjectBuilder& position(const glm::vec3& po);

    ObjectBuilder& position(const float x, const float y, const float z);

    ObjectBuilder& orientation(const float w, const float x, const float y, const float z);

    ObjectBuilder& rotation(const glm::quat& qu);

    ObjectBuilder& mass(const float ma);

    ObjectBuilder& elasticity(const float el);

    ObjectBuilder& shape(std::shared_ptr<Shape> sh);

    ObjectBuilder& name(const std::string& name);

    ObjectBuilder& scale(const glm::vec3& sc);

    ObjectBuilder& friction(const float fr);

    ObjectBuilder& linearVelocity(const glm::vec3& lVel);

    ObjectBuilder& linearVelocity(const float x, const float y, const float z);

    ObjectBuilder& angularVelocity(const glm::vec3& aVel);

    ObjectBuilder& angularVelocity(const float x, const float y, const float z);

    Entity build();

private:
    glm::vec3 m_color{1, 0, 0};
    glm::vec3 m_position{0};
    glm::vec3 m_scale{1};
    glm::quat m_rotation{1, 0, 0, 0};
    glm::vec3 m_linearVelocity{0};
    glm::vec3 m_angularVelocity{0};
    float m_mass{1};
    float m_elasticity{1};
    float m_friction{1};
    std::string m_name;
    std::shared_ptr<Shape> m_shape;
    entt::registry* m_registry;
    Entity m_baseEntity;

};