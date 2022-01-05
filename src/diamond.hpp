#pragma once


#include <memory>
#include <vector>
#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vulkan_util/Entity.hpp>
#include <vulkan_util/components.h>
#include <vulkan_util/VulkanDevice.h>
#include <vulkan_util/Vertex.h>
#include "shape.hpp"
#include "convexhullshape.hpp"
#include "body.hpp"
#include "objectbuilder.hpp"

struct Diamond{
    std::vector<glm::vec3> points{
            {0.1, -1, 0},
            {1, 0, 0},
            {1, 0.1, 0},
            {0.4, 0.4, 0},
            {0.739104, 0.3, 0.306147},
            {0.92388, 0, 0.382683},
            {0.92388, 0.1, 0.382683},
            {0.0707107, -1, 0.0707107},
            {0.707107, 0, 0.707107},
            {0.707107, 0.1, 0.707107},
            {0.282843, 0.4, 0.282843},
            {0.306147, 0.3, 0.739104},
            {0.382683, 0, 0.92388},
            {0.382683, 0.1, 0.92388},
            {-1.11759e-08, -1, 0.1},
            {-1.49012e-07, 0, 1},
            {-1.49012e-07, 0.1, 1},
            {-4.47035e-08, 0.4, 0.4},
            {-0.306147, 0.3, 0.739104},
            {-0.382684, 0, 0.92388},
            {-0.382684, 0.1, 0.92388},
            {-0.0707107, -1, 0.0707107},
            {-0.707107, 0, 0.707107},
            {-0.707107, 0.1, 0.707107},
            {-0.282843, 0.4, 0.282843},
            {-0.739104, 0.3, 0.306147},
            {-0.92388, 0, 0.382683},
            {-0.92388, 0.1, 0.382683},
            {-0.1, -1, -2.98023e-08},
            {-1, 0, -2.98023e-07},
            {-1, 0.1, -2.98023e-07},
            {-0.4, 0.4, -1.19209e-07},
            {-0.739104, 0.3, -0.306147},
            {-0.923879, 0, -0.382684},
            {-0.923879, 0.1, -0.382684},
            {-0.0707107, -1, -0.0707107},
            {-0.707106, 0, -0.707107},
            {-0.707106, 0.1, -0.707107},
            {-0.282843, 0.4, -0.282843},
            {-0.306146, 0.3, -0.739104},
            {-0.382683, 0, -0.92388},
            {-0.382683, 0.1, -0.92388},
            {4.84288e-08, -1, -0.1},
            {5.06639e-07, 0, -1},
            {5.06639e-07, 0.1, -1},
            {1.93715e-07, 0.4, -0.4},
            {0.306147, 0.3, -0.739103},
            {0.382684, 0, -0.923879},
            {0.382684, 0.1, -0.923879},
            {0.0707107, -1, -0.0707106},
            {0.707107, 0, -0.707106},
            {0.707107, 0.1, -0.707106},
            {0.282843, 0.4, -0.282843},
            {0.739104, 0.3, -0.306146},
            {0.92388, 0, -0.382683},
            {0.92388, 0.1, -0.382683}
    };
    std::vector<uint32_t> indices;

    Entity build(VulkanDevice& device, VkPipeline pipeline, VkPipelineLayout layout, Entity diamondEntity, entt::registry& registry){
        auto hullShape = std::make_shared<ConvexHullShape>(points);
        auto hullVertices = hullShape->vertices();
        std::vector<Vertex> vertices;
        for(const auto& point : hullVertices){
            Vertex vertex{};
            vertex.position = glm::vec4(point, 1);
            vertices.push_back(vertex);
        }
        
        indices = hullShape->indices();
        
        for(int i = 0; i < indices.size()/3; i+= 3){
            auto& a = vertices[indices[i]];
            auto& b = vertices[indices[i + 1]];
            auto& c = vertices[indices[i + 2]];
            
            auto ab = b.position.xyz() - a.position.xyz();
            auto ac = c.position.xyz() - a.position.xyz();
            
            auto n = glm::normalize(glm::cross(ab, ac));
            a.normal = n;
            b.normal = n;
            c.normal = n;
        }
        
        diamondEntity.add<component::Render>();
        
        auto vertexBuffer = device.createDeviceLocalBuffer(vertices.data(), BYTE_SIZE(vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        device.setName<VK_OBJECT_TYPE_BUFFER>("diamond_vertices", vertexBuffer.buffer);
        auto indexBuffer = device.createDeviceLocalBuffer(indices.data(), BYTE_SIZE(indices), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
        device.setName<VK_OBJECT_TYPE_BUFFER>("diamond_indices", indexBuffer.buffer);
        auto instanceBuffer = device.createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, sizeof(InstanceData) * 1000, "diamond_xforms");

        auto& renderComponent = diamondEntity.get<component::Render>();
        renderComponent.instanceCount = 0;
        renderComponent.indexCount = indices.size();
        renderComponent.vertexBuffers.push_back(vertexBuffer);
        renderComponent.vertexBuffers.push_back(instanceBuffer);
        renderComponent.indexBuffer = indexBuffer;

        auto indexCount = indices.size();
        auto vertexCount = vertices.size();
        renderComponent.primitives.push_back(vkn::Primitive::indexed(indexCount, 0, vertexCount, 0));

        auto& pipelines = diamondEntity.add<component::Pipelines>();
        pipelines.add({pipeline, layout});

        auto entity =
            ObjectBuilder{ diamondEntity, &registry}
                .position(-10, 3, 0)
                .angularVelocity(0, 0, 10)
                .mass(1)
                .elasticity(1)
                .friction(0.5)
                .shape(hullShape)
            .build();
        entity.add<Diamond>();
        return entity;
    }
};