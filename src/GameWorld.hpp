#pragma once

#include <vulkan/vulkan.h>
#include <vulkan_util/VulkanBaseApp.h>
#include <vulkan_util/SkyBox.hpp>
#include "body.hpp"
#include "contact.hpp"

struct InstanceData{
    glm::mat4 transform;
    glm::vec3 color;
};

struct Color{
    glm::vec3 value;
};

struct SkyBoxTag{};
struct SphereTag{};

class GameWorld : public VulkanBaseApp{
public:
    explicit GameWorld(const Settings& settings = {});

protected:
    void initApp() override;

    void initCamera();

    void createSphereEntity();

    void createSphereInstance(glm::vec3 color, float mass = 1.0f, float elasticity = 1.0f, float radius = 1.0f, const glm::vec3& center = {0, 0, 0});

    void createDescriptorPool();

    void createCommandPool();

    void createPipelineCache();

    void createRenderPipeline();

    void createComputePipeline();

    bool intersect(Body& bodyA, Body& bodyB, Contact& contact);

    void onSwapChainDispose() override;

    void onSwapChainRecreation() override;

    VkCommandBuffer *buildCommandBuffers(uint32_t imageIndex, uint32_t &numCommandBuffers) override;

    void update(float time) override;

    void updateBodies(float time);

    void resolveContact(Contact& contact);

    void updateTransforms();

    void updateInstanceTransforms();

    void checkAppInputs() override;

    void cleanup() override;

    void onPause() override;

    void createSkyBox();

protected:
    struct {
        VulkanPipelineLayout layout;
        VulkanPipeline pipeline;
    } render;

    struct {
        VulkanPipelineLayout layout;
        VulkanPipeline pipeline;
    } compute;

    VulkanDescriptorPool descriptorPool;
    VulkanCommandPool commandPool;
    std::vector<VkCommandBuffer> commandBuffers;
    VulkanPipelineCache pipelineCache;
    std::unique_ptr<CameraController> cameraController;
    SkyBox skyBox;
    Entity sphereEntity;
    const glm::vec3 GRAVITY{0, -10, 0};
};
