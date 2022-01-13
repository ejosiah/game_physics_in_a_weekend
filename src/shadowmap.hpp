#pragma once

#include <array>
#include <glm/glm.hpp>
#include <vulkan_util/VulkanRAII.h>
#include <vulkan_util/VulkanDevice.h>
#include <vulkan_util/GraphicsPipelineBuilder.hpp>
#include <vulkan_util/filemanager.hpp>
#include "shape.hpp"
#include "tags.hpp"


struct ShadowMap{
    VulkanPipeline pipeline;
    VulkanPipelineLayout layout;
    FramebufferAttachment framebufferAttachment;
    VulkanFramebuffer framebuffer;
    VulkanRenderPass renderPass;
    VulkanSampler sampler;
    glm::mat4 lightProjection{1};
    glm::mat4 lightView{1};
    glm::mat4 lightSpaceMatrix{1};
    uint32_t size{2048};
    // Depth bias (and slope) are used to avoid shadowing artifacts
    // Constant depth bias factor (always applied)
    float depthBiasConstant{1.25f};
    // Slope depth bias factor, applied depending on polygon's slope
    float depthBiasSlope{1.75f};
    VulkanDescriptorSetLayout shadowMapDescriptorSetLayout;
    VkDescriptorSet shadowMapDescriptorSet;

    void updateMatrix(const Bounds& scene, const glm::vec3& lightDir){

        lightProjection = vkn::ortho(-80, 80, -80, 80, 1, 200);
        lightView = glm::lookAt(lightDir.xyz() * 50.0f, glm::vec3(0), {0, 1, 0});
    }

    static void initShadowMap(VulkanDevice& device, FileManager& fileMgr, ShadowMap& shadowMap, Bounds scene, const glm::vec4& lightDir = glm::vec4(1)){
        VkFormat format = VK_FORMAT_D16_UNORM;
        auto createFrameBufferAttachment = [&]{
            VkImageCreateInfo createInfo = initializers::imageCreateInfo(
                    VK_IMAGE_TYPE_2D,
                    format,
                    VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                    shadowMap.size,
                    shadowMap.size);

            shadowMap.framebufferAttachment.image = device.createImage(createInfo, VMA_MEMORY_USAGE_GPU_ONLY);

            VkImageSubresourceRange subresourceRange = initializers::imageSubresourceRange(VK_IMAGE_ASPECT_DEPTH_BIT);
            shadowMap.framebufferAttachment.imageView = shadowMap.framebufferAttachment.image.createView(format, VK_IMAGE_VIEW_TYPE_2D, subresourceRange);

            auto samplerInfo = initializers::samplerCreateInfo();
            samplerInfo.magFilter = VK_FILTER_LINEAR;
            samplerInfo.minFilter = VK_FILTER_LINEAR;
            samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
            samplerInfo.addressModeV = samplerInfo.addressModeU;
            samplerInfo.addressModeW = samplerInfo.addressModeU;
            samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
            samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
            shadowMap.sampler = device.createSampler(samplerInfo);
        };

        auto createRenderPass = [&]{
            VkAttachmentDescription attachmentDesc{
                    0,
                    format,
                    VK_SAMPLE_COUNT_1_BIT,
                    VK_ATTACHMENT_LOAD_OP_CLEAR,
                    VK_ATTACHMENT_STORE_OP_STORE,
                    VK_ATTACHMENT_LOAD_OP_DONT_CARE,
                    VK_ATTACHMENT_STORE_OP_DONT_CARE,
                    VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
            };

            SubpassDescription subpass{};
            subpass.depthStencilAttachments = {0, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

            // Use subpass dependencies for layout transitions
            std::vector<VkSubpassDependency> dependencies(2);

            dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
            dependencies[0].dstSubpass = 0;
            dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            dependencies[0].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
            dependencies[0].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

            dependencies[1].srcSubpass = 0;
            dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
            dependencies[1].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
            dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
            dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

            shadowMap.renderPass = device.createRenderPass({attachmentDesc}, {subpass}, {dependencies});
        };

        auto createFrameBuffer = [&]{
            std::vector<VkImageView> attachments{ shadowMap.framebufferAttachment.imageView };
            shadowMap.framebuffer = device.createFramebuffer(shadowMap.renderPass, attachments, shadowMap.size, shadowMap.size);
        };

        auto createPipeline = [&]{
            shadowMap.pipeline =
                device.graphicsPipelineBuilder()
                    .shaderStage()
                        .vertexShader(fileMgr.load("shadowmap.vert.spv"))
                        .fragmentShader(fileMgr.load("shadowmap.frag.spv"))
                    .vertexInputState()
                        .addVertexBindingDescription(0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX)
                        .addVertexBindingDescription(1, sizeof(InstanceData), VK_VERTEX_INPUT_RATE_INSTANCE)
                        .addVertexAttributeDescription(0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(Vertex, position))
                        .addVertexAttributeDescription(1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetOf(Vertex, normal))
                        .addVertexAttributeDescription(2, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(Vertex, color))
                        .addVertexAttributeDescription(3, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform))
                        .addVertexAttributeDescription(4, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 16)
                        .addVertexAttributeDescription(5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 32)
                        .addVertexAttributeDescription(6, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 48)
                    .inputAssemblyState()
                        .triangles()
                    .viewportState()
                        .viewport()
                            .origin(0, 0)
                            .dimension(shadowMap.size, shadowMap.size)
                            .minDepth(0)
                            .maxDepth(1)
                        .scissor()
                            .offset(0, 0)
                        .extent(shadowMap.size, shadowMap.size)
                        .add()
                    .rasterizationState()
                        .enableDepthBias()
                        .depthBiasConstantFactor(shadowMap.depthBiasConstant)
                        .depthBiasSlopeFactor(shadowMap.depthBiasSlope)
                        .cullFrontFace()
                        .frontFaceCounterClockwise()
                        .polygonModeFill()
                    .depthStencilState()
                        .enableDepthWrite()
                        .enableDepthTest()
                        .compareOpLessOrEqual()
                        .minDepthBounds(0)
                        .maxDepthBounds(1)
                    .colorBlendState()
                        .attachment()
                        .add()
                    .layout()
                        .addPushConstantRange(VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4))
                        .renderPass(shadowMap.renderPass)
                    .subpass(0)
                    .name("shadow_map")
                .build(shadowMap.layout);
        };

        createFrameBufferAttachment();
        createRenderPass();
        createFrameBuffer();
        createPipeline();

    }
};
