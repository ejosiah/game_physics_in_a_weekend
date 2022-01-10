#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vulkan_util/Vertex.h>
namespace basis{

    inline std::vector<Vertex> create(){
        std::vector<Vertex> vertices;
        static constexpr auto HALF_PI = glm::half_pi<float>();
        std::vector<glm::vec3> zArrow{
                { 0,     0,    0},
                { 0,     0,    1},
                { 0,     0,    1},
                { 0.05,  0,  0.9},
                { 0,     0,    1},
                {-0.05, 0,   0.9}
        };
        vertices.reserve(zArrow.size() * 3);
        for(auto point : zArrow){
            Vertex v{};
            v.position = glm::vec4(point, 1);
            v.color = glm::vec4(0, 0, 1, 1);
            vertices.push_back(v);
        }

        glm::mat4 transform = glm::rotate(glm::mat4(1), HALF_PI, {0, 1, 0});
        for(auto& zV : zArrow){
            Vertex v{};
            v.position = transform * glm::vec4(zV, 1);
            v.color = glm::vec4(1, 0, 0, 1);
            vertices.push_back(v);
        }

        transform = glm::rotate(glm::mat4(1), -HALF_PI, {1, 0, 0});
        for(auto& zV : zArrow){
            Vertex v{};
            v.position = transform * glm::vec4(zV, 1);
            v.color = glm::vec4(0, 1, 0, 1);
            vertices.push_back(v);
        }
        return vertices;
    }
}