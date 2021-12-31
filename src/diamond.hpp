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
    std::vector<glm::vec3> points;
    std::vector<uint32_t> indices;

    Entity build(VulkanDevice& device, VkPipeline pipeline, VkPipelineLayout layout, Entity diamondEntity, entt::registry& registry){
        diamondEntity.add<Diamond>();
        points.resize(7 * 8);
        std::array<glm::vec3, 8> pts{};
        pts[0] = {0.1f, -1, 0.0f};
        pts[1] = {1, 0, 0};
        pts[2] = {1, 0.1f, 0};
        pts[3] = {0.4f, 0.4f, 0};

        int idx = 0;
        for(auto i = 0; i < 7; i++){
            points[idx] = pts[i];
            idx++;
        }

        const auto pi = glm::pi<float>();
        const glm::quat quat{2.0f * pi * 0.125f * 0.5f, 0, 1, 0};
        glm::quat quatAccum;
        for(auto i = 1; i < 8; i++){
            quatAccum *= quat;
            quatAccum = glm::normalize(quatAccum);
            for (auto pt = 0; pt < 7; pt++) {
                points[idx] = glm::mat3(quatAccum) * pts[pt];
                idx++;
            }
        }
        
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

        return
            ObjectBuilder{ diamondEntity, &registry}
                .position(0, 10, 0)
                .mass(1)
                .elasticity(1)
                .friction(0.5)
                .shape(hullShape)
            .build();
    }
};