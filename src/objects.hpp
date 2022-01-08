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

static constexpr float t = 0.25f;
static constexpr float l = 3.0f;

std::vector<glm::vec3> g_boxUnit = {
        {-1, -1, -1},
        {-1, -1,  1},
        {-1,  1,  1},
        {-1,  1, -1},

        {1,  1,  1},
        {1,  1, -1},
        {1, -1, -1},
        {1, -1,  1}
};


std::vector<glm::vec3> g_halfBoxUnit = {
        {-0.5, -0.5, -0.5},
        {-0.5, -0.5,  0.5},
        {-0.5,  0.5,  0.5},
        {-0.5,  0.5, -0.5},

        {0.5,  0.5,  0.5},
        {0.5,  0.5, -0.5},
        {0.5, -0.5, -0.5},
        {0.5, -0.5,  0.5}
};

std::vector<glm::vec3> g_boxSmall = {
        {-t, -t, -t},
        { t, -t, -t},
        {-t, -t,  t},
        { t, -t,  t},

        {-t, t, -t},
        { t, t, -t},
        {-t, t,  t},
        { t, t,  t}
};

std::vector<glm::vec3> g_boxBeam = {
        {-l, -t, -t},
        { l, -t, -t},
        {-l, -t,  t},
        { l, -t,  t},

        {-l, t, -t},
        { l, t, -t},
        {-l, t,  t},
        { l, t,  t}
};

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