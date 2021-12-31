#include "objectbuilder.hpp"
#include <vulkan_util/components.h>

ObjectBuilder::ObjectBuilder(Entity entity, entt::registry *registry)
: m_baseEntity(entity) 
, m_registry(registry)
{}

ObjectBuilder & ObjectBuilder::position(const glm::vec3 &po) {
    m_position = po;
    return *this;
}

ObjectBuilder & ObjectBuilder::color(const glm::vec3 &co) {
    m_color = co;
    return *this;
}

ObjectBuilder & ObjectBuilder::rotation(const glm::quat &qu) {
    m_rotation = qu;
    return *this;
}

ObjectBuilder & ObjectBuilder::shape(std::shared_ptr<Shape> sh) {
    m_shape = sh;
    return *this;
}

ObjectBuilder & ObjectBuilder::name(const std::string &name) {
    m_name = name;
    return *this;
}

ObjectBuilder & ObjectBuilder::elasticity(const float el) {
    m_elasticity = el;
    return *this;
}

ObjectBuilder & ObjectBuilder::mass(const float ma) {
    m_mass = ma;
    return *this;
}

ObjectBuilder &ObjectBuilder::scale(const glm::vec3 & sc) {
    m_scale = sc;
    return *this;
}

ObjectBuilder &ObjectBuilder::friction(const float fr) {
    m_friction = fr;
    return *this;
}

ObjectBuilder & ObjectBuilder::linearVelocity(const glm::vec3 &lVel) {
    m_linearVelocity = lVel;
    return *this;
}

ObjectBuilder & ObjectBuilder::linearVelocity(const float x, const float y, const float z) {
    m_linearVelocity = glm::vec3(x, y, z);
    return *this;
}

ObjectBuilder & ObjectBuilder::angularVelocity(const glm::vec3 &aVel) {
    m_angularVelocity = aVel;
    return *this;
}

ObjectBuilder & ObjectBuilder::angularVelocity(const float x, const float y, const float z) {
    m_angularVelocity = glm::vec3(x, y, z);
    return *this;
}

Entity ObjectBuilder::build() {
    assert(m_registry &&  entt::entity(m_baseEntity) != entt::null);
    Entity entity{ *m_registry };

    if(auto sphere = dynamic_cast<SphereShape*>(m_shape.get())){
        m_scale = glm::vec3(sphere->m_radius);
        entity.add<SphereTag>();
    }

    if(auto cube = dynamic_cast<BoxShape*>(m_shape.get())){
        m_scale = glm::abs(cube->m_bounds.max - cube->m_bounds.min);
        entity.add<BoxTag>();
    }
    auto& render = m_baseEntity.get<component::Render>();
    if(m_name.empty()){
        auto baseName = m_baseEntity.get<component::Name>().value;
        m_name = fmt::format("{}_{}", baseName, render.instanceCount);
    }

    entity.add<component::Name>().value = m_name;
    entity.add<component::Position>().value = m_position;
    entity.add<component::Scale>().value = m_scale;
    entity.add<component::Rotation>().value = m_rotation;
    entity.add<component::Transform>().value =
            glm::translate(glm::mat4(1), m_position) * glm::mat4(m_rotation) * glm::scale(glm::mat4(1), m_scale);

    auto& body = entity.add<Body>();
    body.position = m_position;
    body.orientation = m_rotation;
    body.invMass = m_mass <= 0 ? 0 : 1.0f/m_mass;
    body.linearVelocity = m_linearVelocity;
    body.angularVelocity = m_angularVelocity;
    body.elasticity = m_elasticity;
    body.friction = m_friction;
    body.shape = m_shape;
    

    auto instances = reinterpret_cast<InstanceData*>(render.vertexBuffers[1].map());
    instances[render.instanceCount].transform = entity.get<component::Transform>().value;
    instances[render.instanceCount].color = m_color;
    render.vertexBuffers[1].unmap();
    render.instanceCount++;

    return entity;
}

ObjectBuilder &ObjectBuilder::orientation(const float w, const float x, const float y, const float z) {
    m_rotation = glm::quat(w, x, y, z);
    return *this;
}

ObjectBuilder &ObjectBuilder::position(const float x, const float y, const float z) {
    m_position = glm::vec3(x, y, z);

    return *this;
}

Body ObjectBuilder::buildBody() {
    Body body;
    body.position = m_position;
    body.orientation = m_rotation;
    body.invMass = m_mass <= 0 ? 0 : 1.0f/m_mass;
    body.linearVelocity = m_linearVelocity;
    body.angularVelocity = m_angularVelocity;
    body.elasticity = m_elasticity;
    body.friction = m_friction;
    body.shape = m_shape;

    return body;
}


