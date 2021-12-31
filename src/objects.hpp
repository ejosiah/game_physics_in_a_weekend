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

struct Objects{
    std::vector<glm::vec3> boxUnit = {
            {-1, -1, -1},
            { 1, -1, -1},
            {-1, -1,  1},
            { 1, -1,  1},

            {-1, 1, -1},
            { 1, 1, -1},
            {-1, 1,  1},
            { 1, 1,  1}
    };

    static constexpr float t = 0.25f;
    std::vector<glm::vec3> boxSmall = {
            {-t, -t, -t},
            { t, -t, -t},
            {-t, -t,  t},
            { t, -t,  t},

            {-t, t, -t},
            { t, t, -t},
            {-t, t,  t},
            { t, t,  t}
    };

    static constexpr float l = 3.0f;
    std::vector<glm::vec3> boxBeam = {
            {-l, -t, -t},
            { l, -t, -t},
            {-l, -t,  t},
            { l, -t,  t},

            {-l, t, -t},
            { l, t, -t},
            {-l, t,  t},
            { l, t,  t}
    };

    void build(ObjectBuilder& builder, entt::registry& registry){
//        builder
//            .position(2, 10, 0)
//            .mass(1.0)
//            .elasticity(0.5)
//            .friction(0.5)
//            .shape(std::make_shared<BoxShape>(boxUnit))
//        .build();

//        builder
//            .position(4, 5, 10)
//            .shape(std::make_shared<BoxShape>(boxSmall))
//        .build();
//
//        builder
//            .position(8, 8, 10)
//            .shape(std::make_shared<BoxShape>(boxBeam))
//        .build();
//
//        builder
//            .position(0, 10, 0)
//            .mass(1.0)
//            .elasticity(0.5)
//            .friction(0.5)
//            .shape(std::make_shared<BoxShape>(boxUnit))
//        .build();

        builder
            .position(0.8, 10, 0)
            .mass(1.0)
            .elasticity(0.5)
            .friction(0.5)
            .shape(std::make_shared<BoxShape>(boxUnit))
        .build();

//        auto boxA = boxUnit;
//        glm::mat4 xform = glm::translate(glm::mat4(1), {0.8, 10, 0});
//        for(auto& v : boxA){
//            v = (xform * glm::vec4(v, 1)).xyz();
//        }
//
//        auto boxB = boxUnit;
//        xform = glm::translate(glm::mat4(1), {0, 10, 0});
//        for(auto& v : boxB){
//            v = (xform * glm::vec4(v, 1)).xyz();
//        }
//
//        std::vector<glm::vec3> CSO;
//        for(auto& v1 : boxA){
//            for(auto& v2 : boxB){
//                auto v3 = v1 - v2;
//                CSO.push_back(v3);
//            }
//        }
//        auto CSOShape = std::make_shared<BoxShape>(CSO);
//        builder
//            .position(0, 0, 0)
//            .mass(1.0)
//            .elasticity(0.5)
//            .friction(0.5)
//            .shape(CSOShape)
//        .build();
//
//        auto min = CSOShape->m_bounds.min;
//        auto max = CSOShape->m_bounds.max;
//        auto d = max - min;
//        auto origin = glm::vec3(0);
//        spdlog::info("min: {}, max: {}, d: {}", min, max, d);
//        std::vector<glm::vec4> faces{
//                {1, 0, 0, d.z - origin.x - min.x},
//                {-1, 0, 0, origin.x - min.x},
//                {0, 1, 0, d.z - origin.y - min.y},
//                {0, -1, 0, origin.y - min.y},
//                {0, 0, 1, d.z - origin.z - min.z},
//                {0, 0, -1, origin.z - min.z},
//        };
//
//        for(auto face : faces){
//            spdlog::info("plane[n: {}, d: {}]", face.xyz(), face.w);
//        }
//
//        float closestDist = std::numeric_limits<float>::max();
//        glm::vec3 closestPoint{0};
//        int idx = -1;
//        for(int i = 0; i < faces.size(); i++){
//            auto& face = faces[i];
//            auto o = origin;
//            auto n = face.xyz();
//            auto d = face.w;
//            auto dist = (glm::dot(n, o) - d)/ glm::dot(n, n);
//            auto p = o - dist * n;
//            if(abs(dist) < closestDist){
//                closestDist = abs(dist);
//                closestPoint = p;
//                idx = i;
//            }
//        }
//        spdlog::info("closest face: {}, distance: {}, point on face: {}", faces[idx], closestDist, closestPoint);
//        auto pointA = min;
//        auto n = faces[idx].xyz();
//        std::array<glm::vec3, 2> pointsBC{};
//
//        for(int i = 0; i < 3; i++){
//            glm::vec3 axis(0);
//            axis[i] = 1;
//            if(glm::dot(axis, n) == 0){
//            spdlog::info("n: {}, axis: {}", n, axis);
//                int pointId = i % 2;
//                float dist = d[i];
//                if(pointId != 0) dist *= 0.5f;
//                pointsBC[pointId] = min + axis * dist;
//            }
//        }
//
//        spdlog::info("face triangle[{}, {}, {}]", pointA, pointsBC[0], pointsBC[1]);
//        glm::vec3 pointB = {min.x, min.y, max.z};
//        glm::vec3 pointC = {min.x, max.y, min.z + d[2] * 0.5};
//        spdlog::info("face triangle[{}, {}, {}]", pointA, pointB, pointC);
//
//        auto uvw = barycentricCoordinates(pointA, pointB, pointC, glm::vec3(0));
//        auto uvw1 = barycentricCoordinates0(pointA, pointB, pointC, closestPoint);
//
//        spdlog::info("uvw: {}", uvw);
//        spdlog::info("uvw: {}", uvw1);

    }
};