#pragma once

#include <memory>
#include <glm/glm.hpp>
#include "shape.hpp"
#include "body.hpp"
#include "vector"
#include "objectbuilder.hpp"
#include <vulkan_util/Entity.hpp>
#include <vulkan_util/components.h>
#include "tags.hpp"

struct SandBox{
    static constexpr float w = 50;
    static constexpr float h = 25;

    std::vector<glm::vec3> boxGround = {
            {-w, -1, -h},
            {-w, -1,  h},
            {-w,  0,  h},
            {-w,  0, -h},

            {w,   0,  h},
            {w,   0, -h},
            {w,  -1, -h},
            {w,  -1,  h}
    };

    std::vector<glm::vec3> boxWall0 = {
            {-1, 0, -h},
            {-1, 0,  h},
            {-1, 5,  h},
            {-1, 5, -h},

            { 1, 5,  h},
            { 1, 5, -h},
            {-1, 0, -h},
            { 1, 0,  h}
    };

    std::vector<glm::vec3> boxWall1 = {
            {-w, 0, -1},
            {-w, 0,  1},
            {-w, 5,  1},
            { w, 5,  -1},

            { w, 5,  1},
            { w, 5, -1},
            { w, 0, -1},
            { w, 0,  1}
    };

     Entity build(ObjectBuilder&& builder, entt::registry& registry){
         auto groundBoxEntity =
             builder
                .position(0, 0, 0)
                .mass(0)
                .elasticity(0.5)
                .friction(0)
                .shape(std::make_shared<BoxShape>(boxGround))
            .build();

        auto boxWall0Shape = std::make_shared<BoxShape>(boxWall0);
        auto boxWall0RightEntity =
            builder
                .position(50, 0, 0)
                .mass(0)
                .elasticity(0.5)
                .friction(1)
                .shape(boxWall0Shape)
            .build();

        auto boxWall0LeftEntity =
            builder
                .position(-50, 0, 0)
                .shape(boxWall0Shape)
            .build();

        auto boxWall1Shape = std::make_shared<BoxShape>(boxWall1);
         auto boxWall1RightEntity =
             builder
                 .position(0, 0, 25)
                 .shape(boxWall1Shape)
             .build();

         auto boxWall1LeftEntity =
             builder
                 .position(0, 0, -25)
                 .shape(boxWall1Shape)
             .build();

         auto sandBoxEntity = Entity{ registry};

//         groundBoxEntity.add<component::Parent>().entity = sandBoxEntity;
//         boxWall0RightEntity.add<component::Parent>().entity = sandBoxEntity;
//         boxWall0LeftEntity.add<component::Parent>().entity = sandBoxEntity;
//         boxWall1RightEntity.add<component::Parent>().entity = sandBoxEntity;
//         boxWall1LeftEntity.add<component::Parent>().entity = sandBoxEntity;

         return sandBoxEntity;
     }
};