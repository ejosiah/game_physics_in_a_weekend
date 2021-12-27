#pragma once

#include <vulkan/vulkan.h>
#include <vulkan_util/VulkanBaseApp.h>
#include <vulkan_util/SkyBox.hpp>
#include "body.hpp"
#include "contact.hpp"
#include "collision.hpp"
#include "objectbuilder.hpp"

//struct Color{
//    glm::vec3 value;
//};
//
//struct SkyBoxTag{};
//struct SphereTag{};

class GameWorld : public VulkanBaseApp{
public:
    explicit GameWorld(const Settings& settings = {});

protected:
    void initApp() override;

    void initCamera();

    void createSphereEntity();

    void createSphereInstance(glm::vec3 color, float mass = 1.0f, float elasticity = 1.0f, float radius = 1.0f, const glm::vec3& center = {0, 0, 0});

    void createSceneObjects();

    void createDescriptorPool();

    void createCommandPool();

    void createPipelineCache();

    void createRenderPipeline();

    void createComputePipeline();

    bool intersect(Body& bodyA, Body& bodyB, float dt, Contact& contact);

    void onSwapChainDispose() override;

    void onSwapChainRecreation() override;

    VkCommandBuffer *buildCommandBuffers(uint32_t imageIndex, uint32_t &numCommandBuffers) override;

    void update(float time) override;

    void fixedUpdate(float dt);

    void renderUI(VkCommandBuffer commandBuffer);

    void updateBodies(float time);

    void resolveContact(Contact& contact);

    void updateTransforms();

    void updateInstanceTransforms();

    void checkAppInputs() override;

    void cleanup() override;

    void onPause() override;

    void createSkyBox();

    template<typename Func, typename durationType = chrono::milliseconds>
    std::chrono::milliseconds profile(Func&& func){
        auto startTime = std::chrono::high_resolution_clock::now();
        func();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = endTime - startTime;
        return std::chrono::duration_cast<durationType>(duration);
    }

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
    bool m_runPhysics{false};
    const glm::vec3 GRAVITY{0, -10, 0};
    std::vector<Body*> bodies;
    Action* createSphereAction;
    float targetFrameRate{120};
    int iterations{1};
    float timeScale{1};
    bool moveCamera{false};

    struct {
        int numObjects{0};
        int numCollisions{0};
        float physicsTime{0};
    } simStates;
};
