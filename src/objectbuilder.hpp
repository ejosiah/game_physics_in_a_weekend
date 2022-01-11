#pragma once

#include <glm/glm.hpp>
#include <entt/entt.hpp>
#include "body.hpp"
#include <vulkan_util/Entity.hpp>
#include <vulkan_util/components.h>
#include "tags.hpp"

class ObjectBuilder{
public:
    ObjectBuilder(Entity entity, entt::registry* registry = nullptr);

    ObjectBuilder& color(const glm::vec3& co);

    ObjectBuilder& position(const glm::vec3& po);

    ObjectBuilder& position(float x, float y, float z);

    ObjectBuilder& orientation(float w, float x, float y, float z);

    ObjectBuilder& orientation(const glm::quat& q);

    ObjectBuilder& rotation(const glm::quat& qu);

    ObjectBuilder& mass(float ma);

    ObjectBuilder& elasticity(float el);

    ObjectBuilder& shape(std::shared_ptr<Shape> sh);

    ObjectBuilder& name(const std::string& name);

    ObjectBuilder& scale(const glm::vec3& sc);

    ObjectBuilder& friction(float fr);

    ObjectBuilder& linearVelocity(const glm::vec3& lVel);

    ObjectBuilder& linearVelocity(float x, float y, float z);

    ObjectBuilder& angularVelocity(const glm::vec3& aVel);

    ObjectBuilder& angularVelocity(float x, float y, float z);

    Entity build();

    Body buildBody();

private:
    glm::vec3 m_color{1, 0, 0};
    glm::vec3 m_position{0};
    glm::vec3 m_scale{1};
    glm::vec3 m_offset{0};
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