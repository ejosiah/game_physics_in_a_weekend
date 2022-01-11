#pragma once

#include "shape.hpp"
#include "body.hpp"
#include "constraints.hpp"
#include "models.hpp"
#include <entt/entt.hpp>
#include "objectbuilder.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vulkan_util/Entity.hpp>

struct Ragdoll{
    static constexpr auto t2 = 0.25f;
    static constexpr auto w2 = t2 * 2.0f;
    static constexpr float h2 = 0.25f;
    static constexpr auto h3 = t2 * 4.0f;

    static std::vector<Entity> build(Entity baseEntity, entt::registry& registry
                                     , std::vector<std::unique_ptr<ConstraintBase>>& constraints
                                     , const glm::vec3& offset = glm::vec3(0)){
        glm::vec3 bodyScale{t2, h3, w2};

        std::vector<glm::vec3> bodyBox;
        for(auto& v : g_halfBoxUnit){
            auto bv = v * bodyScale;
            bodyBox.push_back(bv);
        }

        glm::vec3 limbScale{h3, h2, h2};
        std::vector<glm::vec3> limbBox;
        for(auto& v : g_halfBoxUnit){
            auto bv = v * limbScale;
            limbBox.push_back(bv);
        }


        auto builder = ObjectBuilder(baseEntity, &registry);

        auto head =
            builder
                .position(glm::vec3(0, 5.5f, 0) + offset)
                .shape(std::make_shared<BoxShape>(g_boxSmall))
                .mass(0.5)
                .elasticity(1)
                .friction(1)
            .build();

        auto torso =
            builder
                .position(glm::vec3(0, 4, 0) + offset)
                .shape(std::make_shared<BoxShape>(bodyBox))
                .mass(2)
                .elasticity(1)
                .friction(1)
            .build();

        constexpr auto HALF_PI = glm::half_pi<float>();
        auto leftArm =
            builder
                .position(glm::vec3(0.0f, 4.75, 2.0f) + offset)
                .orientation(glm::angleAxis(-HALF_PI, glm::vec3(0, 1, 0)))
                .shape(std::make_shared<BoxShape>(limbBox))
                .mass(1)
                .elasticity(1)
                .friction(1)
            .build();

        auto rightArm =
            builder
                .position(glm::vec3(0.0f, 4.75, -2.0f) + offset)
                .orientation(glm::angleAxis(HALF_PI, glm::vec3(0, 1, 0)))
                .shape(std::make_shared<BoxShape>(limbBox))
                .mass(1)
                .elasticity(1)
                .friction(1)
            .build();

        auto leftLeg =
            builder
                .position(glm::vec3(0, 2.5, 1) + offset)
                .orientation(glm::angleAxis(HALF_PI, glm::vec3(0, 0, 1)))
                .shape(std::make_shared<BoxShape>(limbBox))
                .mass(1)
                .elasticity(1)
                .friction(1)
            .build();

        auto rightLeg =
            builder
                .position(glm::vec3(0, 2.5, -1) + offset)
                .orientation(glm::angleAxis(HALF_PI, glm::vec3(0, 0, 1)))
                .shape(std::make_shared<BoxShape>(limbBox))
                .mass(1)
                .elasticity(1)
                .friction(1)
            .build();


        // Neck
        {
            auto joint = std::make_unique<ConstraintHingeQuatLimited>();
            joint->m_bodyA = &head.get<Body>();
            joint->m_bodyB = &torso.get<Body>();

            const auto worldSpaceAnchor = joint->m_bodyA->position + glm::vec3{0, -0.5, 0};
            joint->m_anchorA = joint->m_bodyA->worldSpaceToBodySpace(worldSpaceAnchor);
            joint->m_anchorB = joint->m_bodyB->worldSpaceToBodySpace(worldSpaceAnchor);
            joint->m_axisA = glm::rotate(glm::inverse(joint->m_bodyA->orientation), {0, 0, 1});
            joint->m_q0 = glm::inverse(joint->m_bodyA->orientation) * joint->m_bodyB->orientation;
            constraints.push_back(std::move(joint));
        }

        // left Shoulder
        {
            auto joint = std::make_unique<ConstraintConstantVelocityLimited>();
            joint->m_bodyB = &leftArm.get<Body>();
            joint->m_bodyA = &torso.get<Body>();

            const auto anchor = joint->m_bodyB->position + glm::vec3(0, 0, -1);
            joint->m_anchorA = joint->m_bodyA->worldSpaceToBodySpace(anchor);
            joint->m_anchorB = joint->m_bodyB->worldSpaceToBodySpace(anchor);

            joint->m_axisA = glm::rotate(glm::inverse(joint->m_bodyA->orientation), glm::vec3(0, 0, 1));
            joint->m_q0 = glm::inverse(joint->m_bodyA->orientation) * joint->m_bodyB->orientation;
            constraints.push_back(std::move(joint));
        }
        // right Shoulder
        {
            auto joint = std::make_unique<ConstraintConstantVelocityLimited>();
            joint->m_bodyB = &rightArm.get<Body>();
            joint->m_bodyA = &torso.get<Body>();

            const auto anchor = joint->m_bodyB->position + glm::vec3(0, 0, 1);
            joint->m_anchorA = joint->m_bodyA->worldSpaceToBodySpace(anchor);
            joint->m_anchorB = joint->m_bodyB->worldSpaceToBodySpace(anchor);

            joint->m_axisA = glm::rotate(glm::inverse(joint->m_bodyA->orientation), glm::vec3(0, 0, -1));
            joint->m_q0 = glm::inverse(joint->m_bodyA->orientation) * joint->m_bodyB->orientation;
            constraints.push_back(std::move(joint));
        }

        // left hip
        {
            auto joint = std::make_unique<ConstraintHingeQuatLimited>();
            joint->m_bodyB = &leftLeg.get<Body>();
            joint->m_bodyA = &torso.get<Body>();

            const auto anchor = joint->m_bodyB->position + glm::vec3(0, 0.5, 0);
            joint->m_anchorA = joint->m_bodyA->worldSpaceToBodySpace(anchor);
            joint->m_anchorB = joint->m_bodyB->worldSpaceToBodySpace(anchor);

            joint->m_axisA = glm::rotate(glm::inverse(joint->m_bodyA->orientation), glm::vec3(0, 0, 1));
            joint->m_q0 = glm::inverse(joint->m_bodyA->orientation) * joint->m_bodyB->orientation;
            constraints.push_back(std::move(joint));
        }

        // right hip
        {
            auto joint = std::make_unique<ConstraintHingeQuatLimited>();
            joint->m_bodyB = &rightLeg.get<Body>();
            joint->m_bodyA = &torso.get<Body>();

            const auto anchor = joint->m_bodyB->position + glm::vec3(0, 0.5, 0);
            joint->m_anchorA = joint->m_bodyA->worldSpaceToBodySpace(anchor);
            joint->m_anchorB = joint->m_bodyB->worldSpaceToBodySpace(anchor);

            joint->m_axisA = glm::rotate(glm::inverse(joint->m_bodyA->orientation), glm::vec3(0, 0, 1));
            joint->m_q0 = glm::inverse(joint->m_bodyA->orientation) * joint->m_bodyB->orientation;
            constraints.push_back(std::move(joint));
        }

        return { head, torso, leftArm, rightArm, leftLeg, rightLeg};

    }
};