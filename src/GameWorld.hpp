#pragma once

#include <vulkan_util/components.h>
#include <vulkan/vulkan.h>
#include <vulkan_util/VulkanBaseApp.h>
#include <vulkan_util/SkyBox.hpp>
#include "body.hpp"
#include "contact.hpp"
#include "collision.hpp"
#include "objectbuilder.hpp"
#include "constraints.hpp"
#include "manifold.hpp"
#include "shadowmap.hpp"

enum ObjectType{
    SPHERE, BOX, DIAMOND, STACK, RAG_DOLL
};

struct ObjectCreateProperties{
    glm::vec3 size{1};
    glm::vec3 velocity{0};
    glm::vec3 position;
    float mass{1};
    float elasticity{1};
    float friction{1};
    int type{BOX};
    float radius{1};
    float speed{200};
    float rotation{0};
    struct {
        int height{5};
        int type{BOX};
    } stack;
    bool create{false};
};

class GameWorld : public VulkanBaseApp{
public:
    explicit GameWorld(const Settings& settings = {});

protected:
    void initApp() override;

    void initBasis();

    void createDescriptorSetLayouts();

    void createUboBuffer();

    void updateDescriptorSets();

    void updateShadowMapDescriptorSet();

    void updateUboDescriptorSet();

    void initShadowMap();

    void initCamera();

    void createSphereEntity();

    void createCubeEntity();

    void createDiamondEntity();

    void newFrame() override;

    void createObject();

    void createSceneObjects();

    std::vector<Entity> createStack(int type = ObjectType::BOX, const glm::vec3& position = glm::vec3(0), int height = 5);

    std::shared_ptr<ConvexHullShape> diamondShape(float size = 1.0f);

    void createDescriptorPool();

    void createCommandPool();

    void createPipelineCache();

    void createRenderPipeline();

    void createRenderBasisPipeline();

    void createComputePipeline();

    bool conservativeAdvance(Body& bodyA, Body& bodyB, float dt, Contact& contact);

    bool intersect(Body& bodyA, Body& bodyB, float dt, Contact& contact);

    bool intersect(Body& bodyA, Body& bodyB, Contact& contact);

    void onSwapChainDispose() override;

    void onSwapChainRecreation() override;

    VkCommandBuffer *buildCommandBuffers(uint32_t imageIndex, uint32_t &numCommandBuffers) override;

    void castShadow(VkCommandBuffer commandBuffer);

    void update(float time) override;

    void fixedUpdate(float dt);

    void renderUI(VkCommandBuffer commandBuffer);

    void renderObjectCreateMenu(VkCommandBuffer commandBuffer);

    void debugMenu(VkCommandBuffer commandBuffer);

    void updateBodies(float time);

    void resolveContact(Contact& contact);

    void updateTransforms();

    void updateInstanceTransforms();

    void renderObjectBasis(VkCommandBuffer commandBuffer);

    template<typename Tag>
    void updateInstanceTransform(Entity entity){
        auto& renderComp = entity.get<component::Render>();
        auto instanceBuffer = reinterpret_cast<InstanceData*>(renderComp.vertexBuffers[1].map());

        auto i = renderComp.instanceCount - 1;
        auto view = m_registry.view<Tag, component::Transform>(entt::exclude<Delete>);
        for(auto e : view){
            auto& transform = view.get<component::Transform>(e);
            instanceBuffer[i].transform = transform.value;
            i--;
        }
        renderComp.vertexBuffers[1].unmap();
    }

    void checkAppInputs() override;

    void cleanup() override;

    void onPause() override;

    void createSkyBox();

    void updateUbo();

    void renderEntitiesLocal(VkCommandBuffer commandBuffer, entt::registry& registry);

    void renderSkyBox(VkCommandBuffer commandBuffer);

    void renderSceneObjects(VkCommandBuffer commandBuffer);

    template<typename Func, typename durationType = chrono::milliseconds>
    std::chrono::milliseconds profile(Func&& func){
        auto startTime = std::chrono::high_resolution_clock::now();
        func();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = endTime - startTime;
        return std::chrono::duration_cast<durationType>(duration);
    }

    void initFrustum();

    void renderFrustum(VkCommandBuffer commandBuffer);

protected:
    struct {
        VulkanPipelineLayout layout;
        VulkanPipeline pipeline;
        VulkanDescriptorSetLayout uboDescriptorSetLayout;
        VkDescriptorSet uboDescriptorSet;
    } render;

    struct {
        VulkanPipelineLayout layout;
        VulkanPipeline pipeline;
    } compute;

    struct {
        VulkanPipelineLayout layout;
        VulkanPipeline pipeline;
        VulkanBuffer vertices;
    } renderBasis;

    VulkanDescriptorPool descriptorPool;
    VulkanCommandPool commandPool;
    std::vector<VkCommandBuffer> commandBuffers;
    VulkanPipelineCache pipelineCache;
    std::unique_ptr<CameraController> cameraController;
    SkyBox skyBox;
    Entity sphereEntity;
    Entity cubeEntity;
    Entity sandBoxEntity;
    Entity diamondEntity;
    bool m_runPhysics{false};
    const glm::vec3 GRAVITY{0, -10, 0};
    std::vector<Body*> bodies;
    std::vector<ObjectCreateProperties> objectsToCreate;
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

    ObjectCreateProperties objectCreateProps;

    std::vector<std::unique_ptr<ConstraintBase>> m_constraints;
    ManifoldCollector m_manifolds;
    bool m_showBasis{false};
    ShadowMap shadowMap;
    Bounds scene{};

    struct {
        glm::mat4 view;
        glm::mat4 projection;
        glm::mat4 lightSpaceMatrix;
    } ubo;

    VulkanBuffer uboBuffer;

    struct {
        VulkanBuffer vertices;
        VulkanBuffer indices;
        VulkanPipelineLayout layout;
        VulkanPipeline pipeline;
    } frustum;
};
