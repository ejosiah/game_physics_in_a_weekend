#pragma once

#include <memory>
#include <glm/glm.hpp>
#include "shape.hpp"
#include "body.hpp"
#include "vector"
#include "objectbuilder.hpp"
#include <vulkan_util/Entity.hpp>
#include <vulkan_util/components.h>
#include <tuple>
#include "math.hpp"
#include "models.hpp"


struct Objects{

    void build(ObjectBuilder& builder, entt::registry& registry){
        builder
            .position(0, 0.5, 0)
            .shape(std::make_shared<BoxShape>(g_boxSmall))
            .mass(0)
            .elasticity(1.0f)
        .build();

        builder
            .position(1, 5, 0)
            .mass(1)
        .build();

    }
};