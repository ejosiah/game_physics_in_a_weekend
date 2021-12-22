#include "GameWorld.hpp"
#include <vulkan_util/GraphicsPipelineBuilder.hpp>
#include <vulkan_util/DescriptorSetBuilder.hpp>
#include <vulkan_util/glm_format.h>

GameWorld::GameWorld(const Settings& settings) : VulkanBaseApp("Game Physics In One Weekend", settings) {
    fileManager.addSearchPath("spv");
    fileManager.addSearchPath("shaders");
}

void GameWorld::initApp() {
    SkyBox::init(this);
    createDescriptorPool();
    createCommandPool();
    initCamera();
    createSkyBox();
    createPipelineCache();
    createRenderPipeline();
    createComputePipeline();
    createSphereEntity();
    createSphereInstance({1, 0, 0}, 1.0f, 0.8f, 1.0f, {0, 10, 0});
    createSphereInstance({0, 1, 0}, 0.0f, 1.0f, 1000, {0, -1000, 0});
}

void GameWorld::createDescriptorPool() {
    constexpr uint32_t maxSets = 100;
    std::array<VkDescriptorPoolSize, 16> poolSizes{
            {
                    {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 100 * maxSets},
                    {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 100 * maxSets},
                    {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 100 * maxSets},
                    { VK_DESCRIPTOR_TYPE_SAMPLER, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_INLINE_UNIFORM_BLOCK_EXT, 100 * maxSets },
                    { VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 100 * maxSets }
            }
    };
    descriptorPool = device.createDescriptorPool(maxSets, poolSizes, VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT);

}

void GameWorld::createCommandPool() {
    commandPool = device.createCommandPool(*device.queueFamilyIndex.graphics, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);
    commandBuffers = commandPool.allocateCommandBuffers(swapChainImageCount);
}

void GameWorld::createPipelineCache() {
    pipelineCache = device.createPipelineCache();
}


void GameWorld::createRenderPipeline() {
    auto builder = device.graphicsPipelineBuilder();
    render.pipeline =
        builder
            .shaderStage()
                .vertexShader(load("render.vert.spv"))
                .fragmentShader(load("render.frag.spv"))
            .vertexInputState()
                .addVertexBindingDescription(0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX)
                .addVertexBindingDescription(1, sizeof(InstanceData), VK_VERTEX_INPUT_RATE_INSTANCE)
                .addVertexAttributeDescription(0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(Vertex, position))
                .addVertexAttributeDescription(1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetOf(Vertex, normal))
                .addVertexAttributeDescription(2, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetOf(InstanceData, color))
                .addVertexAttributeDescription(3, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform))
                .addVertexAttributeDescription(4, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 16)
                .addVertexAttributeDescription(5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 32)
                .addVertexAttributeDescription(6, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 48)
            .inputAssemblyState()
                .triangles()
            .viewportState()
                .viewport()
                    .origin(0, 0)
                    .dimension(swapChain.extent)
                    .minDepth(0)
                    .maxDepth(1)
                .scissor()
                    .offset(0, 0)
                    .extent(swapChain.extent)
                .add()
                .rasterizationState()
                    .cullBackFace()
                    .frontFaceCounterClockwise()
                    .polygonModeFill()
                .depthStencilState()
                    .enableDepthWrite()
                    .enableDepthTest()
                    .compareOpLess()
                    .minDepthBounds(0)
                    .maxDepthBounds(1)
                .colorBlendState()
                    .attachment()
                    .add()
                .layout()
                    .addPushConstantRange(Camera::pushConstant())
                .renderPass(renderPass)
                .subpass(0)
                .name("render")
                .pipelineCache(pipelineCache)
            .build(render.layout);
}

void GameWorld::createComputePipeline() {
//    auto module = VulkanShaderModule{ "../../data/shaders/pass_through.comp.spv", device};
//    auto stage = initializers::shaderStage({ module, VK_SHADER_STAGE_COMPUTE_BIT});
//
//    compute.layout = device.createPipelineLayout();
//
//    auto computeCreateInfo = initializers::computePipelineCreateInfo();
//    computeCreateInfo.stage = stage;
//    computeCreateInfo.layout = compute.layout;
//
//    compute.pipeline = device.createComputePipeline(computeCreateInfo, pipelineCache);
}


void GameWorld::onSwapChainDispose() {
    dispose(render.pipeline);
    dispose(compute.pipeline);
}

void GameWorld::onSwapChainRecreation() {
    createRenderPipeline();
    createComputePipeline();
}

VkCommandBuffer *GameWorld::buildCommandBuffers(uint32_t imageIndex, uint32_t &numCommandBuffers) {
    numCommandBuffers = 1;
    auto& commandBuffer = commandBuffers[imageIndex];

    VkCommandBufferBeginInfo beginInfo = initializers::commandBufferBeginInfo();
    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    static std::array<VkClearValue, 2> clearValues;
    clearValues[0].color = {0, 0, 1, 1};
    clearValues[1].depthStencil = {1.0, 0u};

    VkRenderPassBeginInfo rPassInfo = initializers::renderPassBeginInfo();
    rPassInfo.clearValueCount = COUNT(clearValues);
    rPassInfo.pClearValues = clearValues.data();
    rPassInfo.framebuffer = framebuffers[imageIndex];
    rPassInfo.renderArea.offset = {0u, 0u};
    rPassInfo.renderArea.extent = swapChain.extent;
    rPassInfo.renderPass = renderPass;

    vkCmdBeginRenderPass(commandBuffer, &rPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    renderEntities(commandBuffer);

    vkCmdEndRenderPass(commandBuffer);

    vkEndCommandBuffer(commandBuffer);

    return &commandBuffer;
}

void GameWorld::update(float time) {
    updateBodies(time);
    updateTransforms();
    updateEntityTransforms();
    updateInstanceTransforms();
    cameraController->update(time);
//    if(int(elapsedTime) % 5 == 0){
//        spdlog::info("cam pos: {}", cameraController->position());
//    }
}

void GameWorld::checkAppInputs() {
    cameraController->processInput();
}

void GameWorld::cleanup() {
    // TODO save pipeline cache
    VulkanBaseApp::cleanup();
}

void GameWorld::onPause() {
    VulkanBaseApp::onPause();
}

void GameWorld::createSkyBox() {
    SkyBox::create(skyBox, R"(C:\Users\Josiah\OneDrive\media\textures\skybox\005)"
            , {"right.jpg", "left.jpg", "top.jpg", "bottom.jpg", "front.jpg", "back.jpg"});

    auto entity = createEntity("sky_box");
    entity.add<SkyBoxTag>();
    auto& renderComponent = entity.add<component::Render>();
    renderComponent.vertexBuffers.push_back(skyBox.cube.vertices);
    renderComponent.indexBuffer = skyBox.cube.indices;



    auto indexCount = skyBox.cube.indices.size/sizeof(uint32_t);
    auto vertexCount = skyBox.cube.vertices.size/sizeof(glm::vec3);
    renderComponent.primitives.push_back(vkn::Primitive::indexed(indexCount, 0, vertexCount, 0));
    renderComponent.indexCount = indexCount;

    auto& pipelines = entity.add<component::Pipelines>();
    pipelines.add({ *skyBox.pipeline, *skyBox.layout, 0, {}, { skyBox.descriptorSet } });
}

void GameWorld::initCamera() {
    CameraSettings settings{};
    settings.aspectRatio = static_cast<float>(swapChain.extent.width)/static_cast<float>(swapChain.extent.height);
    settings.horizontalFov = true;
    settings.velocity = glm::vec3(10);
    settings.acceleration = glm::vec3(5);
    cameraController = std::make_unique<CameraController>(device, swapChain.imageCount(), currentImageIndex, dynamic_cast<InputManager&>(*this), settings);
    cameraController->setMode(CameraMode::SPECTATOR);
    cameraController->lookAt({28.174, 16.484, 28.524}, {0, 0, 0}, {0, 1, 0});

    auto cameraEntity = createEntity("camera");
    cameraEntity.add<component::Camera>().camera = const_cast<Camera*>(&cameraController->cam());
}

void GameWorld::createSphereEntity() {
    sphereEntity = createEntity("sphere");
    sphereEntity.add<component::Render>();

    auto sphere = primitives::sphere(50, 50, 1.0f, glm::mat4(1), glm::vec4(0), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);

    auto vertices = device.createDeviceLocalBuffer(sphere.vertices.data(), BYTE_SIZE(sphere.vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    auto indexes = device.createDeviceLocalBuffer(sphere.indices.data(), BYTE_SIZE(sphere.indices), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    auto instances = device.createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, sizeof(InstanceData) * 1000, "sphere_xforms");
    auto& renderComponent = sphereEntity.get<component::Render>();
    renderComponent.instanceCount = 0;
    renderComponent.indexCount = sphere.indices.size();
    renderComponent.vertexBuffers.push_back(vertices);
    renderComponent.vertexBuffers.push_back(instances);
    renderComponent.indexBuffer = indexes;

    auto indexCount = sphere.indices.size();
    auto vertexCount = sphere.vertices.size();
    renderComponent.primitives.push_back(vkn::Primitive::indexed(indexCount, 0, vertexCount, 0));

    auto& pipelines = sphereEntity.add<component::Pipelines>();
    pipelines.add({render.pipeline, render.layout});
}

void GameWorld::createSphereInstance(glm::vec3 color, float mass, float elasticity, float radius, const glm::vec3& center) {
    
    auto& sphereRender = sphereEntity.get<component::Render>();
    auto entity = createEntity(fmt::format("sphere_{}", sphereRender.instanceCount));
    entity.add<SphereTag>();
    entity.add<Color>().value = color;
    entity.get<component::Position>().value = center;
    entity.get<component::Scale>().value = glm::vec3(radius);
    entity.get<component::Rotation>().value = glm::quat(1, 0, 0 ,0);
    auto& body = entity.add<Body>();
    body.position = center;
    body.orientation = glm::quat(1, 0, 0, 0);
    body.invMass = mass <= 0 ? 0 : 1.0f/mass;
    body.shape = std::make_shared<SphereShape>(radius);
    body.elasticity = elasticity;

    updateEntityTransforms();
    InstanceData* instances = reinterpret_cast<InstanceData*>(sphereRender.vertexBuffers[1].map());
    instances[sphereRender.instanceCount].transform = entity.get<component::Transform>().value;
    instances[sphereRender.instanceCount].color = color;
    sphereRender.vertexBuffers[1].unmap();
    sphereRender.instanceCount++;
}

void GameWorld::updateBodies(float dt) {
    auto view = registry.view<Body>();

    for(auto entity : view){
        auto& body = view.get<Body>(entity);
        float mass = 1.0f/body.invMass;
        auto impulseGravity = GRAVITY * mass * dt;
        body.applyImpulseLinear(impulseGravity);

    }

    // collision check
    for(auto entityA : view){
        auto viewB = registry.view<Body>();
        for(auto entityB : viewB){
            if(entityA == entityB ) continue;

            auto& bodyA = view.get<Body>(entityA);
            auto& bodyB = viewB.get<Body>(entityB);
            if(bodyA.invMass == 0 && bodyB.invMass == 0) continue;

            Contact contact;
            if(intersect(bodyA, bodyB, contact)){
                resolveContact(contact);
            }
        }
    }

    for(auto entity : view){
        auto& body = view.get<Body>(entity);
        body.update(dt);
    }
}

void GameWorld::updateTransforms() {
    auto view = registry.view<Body, component::Position, component::Rotation>();

    for(auto entity : view){
        auto& body = view.get<Body>(entity);
//        spdlog::info("body[velocity: {}, position: {}", body.linearVelocity, body.position);
        auto& position = view.get<component::Position>(entity);
        auto& rotation = view.get<component::Rotation>(entity);
        position.value = body.position;
        rotation.value = body.orientation;
    }
}

void GameWorld::updateInstanceTransforms() {
    auto renderComp = sphereEntity.get<component::Render>();
    auto instanceBuffer = reinterpret_cast<InstanceData*>(renderComp.vertexBuffers[1].map());

    auto i = 0;
    auto view = registry.view<SphereTag, component::Transform>();
    for(auto entity : view){
        auto& transform = view.get<component::Transform>(entity);
        instanceBuffer[i].transform = transform.value;
    }
    renderComp.vertexBuffers[1].unmap();
}

bool GameWorld::intersect(Body &bodyA, Body &bodyB, Contact& contact) {
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;

    const auto ab = bodyB.position - bodyA.position;
    contact.normal = normalize(ab);

    const auto sphereA = dynamic_cast<SphereShape*>(bodyA.shape.get());
    const auto sphereB = dynamic_cast<SphereShape*>(bodyB.shape.get());
    const auto radiusAB = sphereA->m_radius + sphereB->m_radius;

    contact.worldSpace.pointOnA = bodyA.position + contact.normal * sphereA->m_radius;
    contact.worldSpace.pointOnB = bodyB.position - contact.normal * sphereB->m_radius;

    return dot(ab, ab) < (radiusAB * radiusAB);
}

void GameWorld::resolveContact(Contact &contact) {
    auto bodyA = contact.bodyA;
    auto bodyB = contact.bodyB;
    auto invMassA = bodyA->invMass;
    auto invMassB = bodyB->invMass;

    auto elasticity = bodyA->elasticity * bodyB->elasticity;

    const auto& n = contact.normal;
    const auto relVelocity = bodyA->linearVelocity - bodyB->linearVelocity;
    const auto  impulseJ = -(1.0f + elasticity) * dot(n, relVelocity) / (invMassA + invMassB);

    bodyA->applyImpulseLinear(n * impulseJ);
    bodyB->applyImpulseLinear(-n * impulseJ);

    const auto ta = bodyA->invMass / (bodyA->invMass + bodyB->invMass);
    const auto tb = bodyB->invMass / (bodyA->invMass + bodyB->invMass);

    const auto ds = contact.worldSpace.pointOnB - contact.worldSpace.pointOnA;
    bodyA->position += ds * ta;
    bodyB->position -= ds * tb;
}
