#include "GameWorld.hpp"
#include <vulkan_util/GraphicsPipelineBuilder.hpp>
#include <vulkan_util/DescriptorSetBuilder.hpp>
#include <vulkan_util/glm_format.h>
#include "utility.hpp"
#include <vulkan_util/ImGuiPlugin.hpp>
#include <vulkan_util/random.h>
#include <vulkan_util/random.h>
#include "sandbox.hpp"
#include "objects.hpp"
#include "diamond.hpp"
#include "gjk.hpp"
#include "formats.hpp"
#include "Basis.hpp"
#include "models.hpp"
#include "ragdoll.hpp"

GameWorld::GameWorld(const Settings& settings) : VulkanBaseApp("Game Physics In One Weekend", settings) {
    fileManager.addSearchPath("spv");
    fileManager.addSearchPath("shaders");
    createSphereAction = &mapToMouse(static_cast<int>(MouseEvent::Button::RIGHT), "sphere", Action::detectInitialPressOnly());
}

void GameWorld::initApp() {
    scene.expand(glm::vec3(80));
    scene.expand(glm::vec3(-20));
    SkyBox::init(this);
    createDescriptorPool();
    createDescriptorSetLayouts();
    initShadowMap();
    createCommandPool();
    initCamera();
    createSkyBox();
    initBasis();
    createPipelineCache();
    createRenderPipeline();
    createRenderBasisPipeline();
    createUboBuffer();
    updateDescriptorSets();
    createComputePipeline();
    createSphereEntity();
    createCubeEntity();
    createDiamondEntity();
    createSceneObjects();
    initFrustum();
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
                .addVertexAttributeDescription(2, 0, VK_FORMAT_R32G32_SFLOAT, offsetOf(Vertex, uv))
                .addVertexAttributeDescription(3, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetOf(InstanceData, color))
                .addVertexAttributeDescription(4, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform))
                .addVertexAttributeDescription(5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 16)
                .addVertexAttributeDescription(6, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 32)
                .addVertexAttributeDescription(7, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(InstanceData, transform) + 48)
                .addVertexAttributeDescription(8, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetOf(InstanceData, scale))
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
                    .addPushConstantRange(VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4))
                    .addDescriptorSetLayout(render.uboDescriptorSetLayout)
                    .addDescriptorSetLayout(shadowMap.shadowMapDescriptorSetLayout)
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

    castShadow(commandBuffer);

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

    renderSceneObjects(commandBuffer);
    renderSkyBox(commandBuffer);

    if(m_showBasis) {
        renderObjectBasis(commandBuffer);
    }

    renderUI(commandBuffer);

    vkCmdEndRenderPass(commandBuffer);

    vkEndCommandBuffer(commandBuffer);

    return &commandBuffer;
}

void GameWorld::castShadow(VkCommandBuffer commandBuffer) {


    static std::array<VkClearValue, 1> clearValues;
    clearValues[0].depthStencil = {1.0, 0u};

    VkRenderPassBeginInfo rPassInfo = initializers::renderPassBeginInfo();
    rPassInfo.clearValueCount = COUNT(clearValues);
    rPassInfo.pClearValues = clearValues.data();
    rPassInfo.framebuffer = shadowMap.framebuffer;
    rPassInfo.renderArea.offset = {0u, 0u};
    rPassInfo.renderArea.extent = {shadowMap.size, shadowMap.size };
    rPassInfo.renderPass = shadowMap.renderPass;

    vkCmdBeginRenderPass(commandBuffer, &rPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    auto view = m_registry.view<component::Render, component::Transform>(entt::exclude<SkyBoxTag>);
    static std::vector<VkBuffer> buffers;
    for(auto entity : view){
        auto model = view.get<component::Transform>(entity).value;
        auto& renderComp = view.get<component::Render>(entity);

        buffers.clear();
        for(auto& buffer : renderComp.vertexBuffers){
            buffers.push_back(buffer.buffer);
        }
        std::vector<VkDeviceSize> offsets(renderComp.vertexBuffers.size(), 0);
        vkCmdBindVertexBuffers(commandBuffer, 0, COUNT(buffers), buffers.data(), offsets.data());
        if (renderComp.indexCount > 0) {
            vkCmdBindIndexBuffer(commandBuffer, renderComp.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }

        shadowMap.lightSpaceMatrix = shadowMap.lightProjection * shadowMap.lightView;
        auto modelTransform = shadowMap.lightSpaceMatrix * model;

        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowMap.pipeline);
        vkCmdPushConstants(commandBuffer, shadowMap.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &modelTransform);
        for (auto primitive : renderComp.primitives) {
            if (renderComp.indexCount > 0) {
                primitive.drawIndexed(commandBuffer, 0, renderComp.instanceCount);
            } else {
                primitive.draw(commandBuffer, 0, renderComp.instanceCount);
            }
        }
    }


    vkCmdEndRenderPass(commandBuffer);
}

void GameWorld::update(float time) {
    static float dt = 1.0f/targetFrameRate;
    static int dtMs = static_cast<int>(dt * 1000);
    auto elapsedTimeMs = static_cast<uint64_t>(elapsedTime * 1000);

    bool runPhysics = m_runPhysics && (elapsedTimeMs % dtMs == 0) && (currentFrame % MAX_IN_FLIGHT_FRAMES == 0);

    if(runPhysics){
        fixedUpdate(dt);
    }
    if(moveCamera) {
        cameraController->update(time);
    }
    updateUbo();
    uboBuffer.copy(&ubo, sizeof(ubo));
}

void GameWorld::fixedUpdate(float dt) {
    static uint64_t physicsFrames = 1;

    float deltaTime = dt/float(iterations);
    float frameTime = 0.0f;
    for(auto i = 0; i < iterations; i++){
        auto duration = profile([&]{ updateBodies(deltaTime * timeScale); });
        frameTime += static_cast<float>(duration.count());
    }
    frameTime /= float(iterations);

    simStates.physicsTime = glm::mix(simStates.physicsTime, frameTime, 1.0/static_cast<float>(physicsFrames));
    physicsFrames++;

    updateTransforms();
    updateInstanceTransforms();
}

void GameWorld::checkAppInputs() {
    if(createSphereAction->isPressed()){
        createObject();
    }
    if(moveCamera) {
        cameraController->processInput();
    }
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
    pipelines.add({ *skyBox.pipeline, *skyBox.layout, 0, {}, { (uint64_t)skyBox.descriptorSet } });
}

void GameWorld::initCamera() {
    CameraSettings settings{};
    settings.aspectRatio = static_cast<float>(swapChain.extent.width)/static_cast<float>(swapChain.extent.height);
    settings.horizontalFov = true;
    settings.velocity = glm::vec3(10);
    settings.acceleration = glm::vec3(5);
    settings.zFar = 200;
    cameraController = std::make_unique<CameraController>(device, swapChain.imageCount(), currentImageIndex, dynamic_cast<InputManager&>(*this), settings);
    cameraController->setMode(CameraMode::SPECTATOR);
    cameraController->lookAt({18, 17, 25}, {0, 3, 0}, {0, 1, 0});

    auto cameraEntity = createEntity("camera");
    cameraEntity.add<component::Camera>().camera = const_cast<Camera*>(&cameraController->cam());
}

void GameWorld::createSphereEntity() {
    sphereEntity = createEntity("sphere");
    sphereEntity.add<component::Render>();

    auto sphere = primitives::sphere(50, 50, 1.0f, glm::mat4(1), glm::vec4(0), VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);

    auto vertices = device.createDeviceLocalBuffer(sphere.vertices.data(), BYTE_SIZE(sphere.vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    device.setName<VK_OBJECT_TYPE_BUFFER>("sphere_vertices", vertices.buffer);
    auto indexes = device.createDeviceLocalBuffer(sphere.indices.data(), BYTE_SIZE(sphere.indices), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    device.setName<VK_OBJECT_TYPE_BUFFER>("sphere_indices", indexes.buffer);
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

void GameWorld::createCubeEntity() {
    cubeEntity = createEntity("cube");
    cubeEntity.add<component::Render>();

    auto cube = primitives::cube();

    auto vertices = device.createDeviceLocalBuffer(cube.vertices.data(), BYTE_SIZE(cube.vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    device.setName<VK_OBJECT_TYPE_BUFFER>("cube_vertices", vertices.buffer);
    auto indexes = device.createDeviceLocalBuffer(cube.indices.data(), BYTE_SIZE(cube.indices), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    device.setName<VK_OBJECT_TYPE_BUFFER>("cube_indices", indexes.buffer);
    auto instances = device.createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, sizeof(InstanceData) * 1000, "cube_xforms");

    auto& renderComponent = cubeEntity.get<component::Render>();
    renderComponent.instanceCount = 0;
    renderComponent.indexCount = cube.indices.size();
    renderComponent.vertexBuffers.push_back(vertices);
    renderComponent.vertexBuffers.push_back(instances);
    renderComponent.indexBuffer = indexes;

    auto indexCount = cube.indices.size();
    auto vertexCount = cube.vertices.size();
    renderComponent.primitives.push_back(vkn::Primitive::indexed(indexCount, 0, vertexCount, 0));

    auto& pipelines = cubeEntity.add<component::Pipelines>();
    pipelines.add({render.pipeline, render.layout});
}

void GameWorld::createDiamondEntity() {
    diamondEntity = createEntity("diamond");
    VulkanDrawable drawable;
    phong::VulkanDrawableInfo info{};
    info.vertexUsage |= VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    phong::load("models/diamond.obj", device, descriptorPool, drawable, info);


    VulkanBuffer stagingBuffer = device.createStagingBuffer(drawable.vertexBuffer.size);
    device.copy(drawable.vertexBuffer, stagingBuffer, stagingBuffer.size);

    std::vector<glm::vec3> points;
    auto vertices0 = reinterpret_cast<Vertex*>(stagingBuffer.map());
    auto numPoints = stagingBuffer.size/sizeof(Vertex);
    points.reserve(numPoints);
    for(auto i = 0; i < numPoints; i++){
        points.push_back(vertices0[i].position.xyz());
    }
    stagingBuffer.unmap();
    auto& shape = diamondEntity.add<ConvexHullShape>();
    shape.build(points);

    auto hullShape = &shape;
    auto hullVertices = hullShape->vertices();
    std::vector<Vertex> vertices;
    for(const auto& point : hullVertices){
        Vertex vertex{};
        vertex.position = glm::vec4(point, 1);
        vertices.push_back(vertex);
    }

    auto indices = hullShape->indices();

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

//    auto& renderComponent = diamondEntity.add<component::Render>();
//    renderComponent.instanceCount = 0;
//    renderComponent.indexCount = drawable.indexBuffer.size/sizeof(uint32_t);
//    renderComponent.vertexBuffers.push_back(drawable.vertexBuffer);
//    renderComponent.indexBuffer = drawable.indexBuffer;
//
//    auto instances = device.createBuffer(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU, sizeof(InstanceData) * 1000, "diamond_xforms");
//    renderComponent.vertexBuffers.push_back(instances);
//
//    for(auto& mesh : drawable.meshes){
//        renderComponent.primitives.push_back(mesh);
//    }

    auto& pipelines = diamondEntity.add<component::Pipelines>();
    pipelines.add({render.pipeline, render.layout});
}

void GameWorld::createObject() {
    const auto& cam = cameraController->cam();
    auto dir = glm::unProject(glm::vec3(mouse.position, 1), cam.view, cam.proj, glm::vec4{0, 0, width, height});
    dir = glm::normalize(dir);
    auto t = glm::length(cameraController->position());
    glm::vec3 angularVelocity{0};
    if(objectCreateProps.speed == 0){
        objectCreateProps.position = cameraController->position() + dir * t;
        objectCreateProps.velocity = glm::vec3(0);
    }else{
        objectCreateProps.position = cameraController->position();
        objectCreateProps.velocity = dir * objectCreateProps.speed;
        if(objectCreateProps.rotation > 0){
            angularVelocity = glm::normalize(randomVec3()) * objectCreateProps.rotation;
        }
    }
    std::vector<Entity> entities;
    if (objectCreateProps.type == ObjectType::SPHERE) {
        auto entity =
                ObjectBuilder(sphereEntity, &m_registry)
                        .shape(std::make_shared<SphereShape>(objectCreateProps.radius))
                        .position(objectCreateProps.position)
                        .mass(objectCreateProps.mass)
                        .elasticity(objectCreateProps.elasticity)
                        .friction(objectCreateProps.friction)
                        .linearVelocity(objectCreateProps.velocity)
                        .angularVelocity(angularVelocity)
                        .build();
        entities.push_back(entity);
    } else if (objectCreateProps.type == ObjectType::BOX) {
        auto box = g_halfBoxUnit;
        for (auto &v : box) {
            v *= objectCreateProps.size;
        }
        auto  entity =
                ObjectBuilder(cubeEntity, &m_registry)
                        .shape(std::make_shared<BoxShape>(box))
                        .position(objectCreateProps.position)
                        .mass(objectCreateProps.mass)
                        .elasticity(objectCreateProps.elasticity)
                        .friction(objectCreateProps.friction)
                        .linearVelocity(objectCreateProps.velocity)
                        .angularVelocity(angularVelocity)
                        .build();
        entities.push_back(entity);
    } else if (objectCreateProps.type == ObjectType::DIAMOND) {
        auto entity =
                ObjectBuilder(diamondEntity, &m_registry)
                        .shape(diamondShape(objectCreateProps.radius))
                        .position(objectCreateProps.position)
                        .mass(objectCreateProps.mass)
                        .elasticity(objectCreateProps.elasticity)
                        .friction(objectCreateProps.friction)
                        .linearVelocity(objectCreateProps.velocity)
                        .angularVelocity(angularVelocity)
                        .build();
        entity.add<Diamond>();

        auto unitBounds = diamondShape()->m_bounds;
        auto unitSize = (unitBounds.max - unitBounds.min);
        auto scaledSize = unitSize * objectCreateProps.radius;
        entity.get<component::Scale>().value = scaledSize / unitSize;
        entity.get<Offset>().value = (unitBounds.min + unitBounds.max) * 0.5f * objectCreateProps.radius;
        entities.push_back(entity);
    } else if(objectCreateProps.type == ObjectType::STACK){
        entities = createStack(objectCreateProps.stack.type, objectCreateProps.position, objectCreateProps.stack.height);
    }else if(objectCreateProps.type == ObjectType::RAG_DOLL){
        entities = Ragdoll::build(cubeEntity, m_registry, m_constraints, objectCreateProps.position);
    }

    for(auto entity : entities) {
        auto &body = entity.get<Body>();
        bodies.push_back(&body);
        spdlog::debug("added body {} to scene", body.id);
    }
}


void GameWorld::updateBodies(float dt) {
    m_manifolds.removeExpired();

    for(auto body : bodies){
        float mass = 1.0f/body->invMass;
        auto impulseGravity = GRAVITY * mass * dt;
        body->applyImpulseLinear(impulseGravity);
    }

    // broad phase
    std::vector<CollisionPair> collisionPairs;
    broadPhase(bodies, collisionPairs, dt);

    // Narrow Phase
    auto numBodies = bodies.size();
    static std::vector<Contact> contacts;
    contacts.clear();
    contacts.reserve(numBodies * numBodies);

    for(const auto pair : collisionPairs){
        auto& bodyA = *bodies[pair.a];
        auto& bodyB = *bodies[pair.b];
        if(bodyA.invMass == 0 && bodyB.invMass == 0) continue;

        Contact contact{};
        if(intersect(bodyA, bodyB, dt, contact)){
            if(contact.timeOfImpact == 0.0f){
                m_manifolds.addContact(contact);
            }else {
                contacts.push_back(contact);
            }
        }
    }

    simStates.numCollisions = static_cast<int>(contacts.size());
    if(simStates.numCollisions > 1){
        // TODO check performance of std::sort
        std::sort(begin(contacts), end(contacts), [](auto a, auto b){
            return a.timeOfImpact < b.timeOfImpact;
        });
    }

    for(auto& constraint : m_constraints){
        constraint->preSolve(dt);
    }

    m_manifolds.preSolve(dt);
    static constexpr int maxIters = 5;
    for(auto iters = 0; iters < maxIters; iters++){
        for(auto& constraint : m_constraints){
            constraint->solve();
        }
        m_manifolds.solve();
    }

    for(auto& constraint : m_constraints){
        constraint->postSolve();
    }
    m_manifolds.postSolve();

    // apply ballistic impulse
    float accumulatedTime = 0.0f;
    for(auto& contact : contacts){
        const auto dt = contact.timeOfImpact - accumulatedTime;
        auto bodyA = contact.bodyA;
        auto bodyB = contact.bodyB;

        if(bodyA->hasInfiniteMass() && bodyB->hasInfiniteMass()){
            continue;
        }

        for(auto body : bodies){
            body->update(dt);
        }

        resolveContact(contact);
        accumulatedTime += dt;
    }

    for(auto body : bodies){
        body->update(dt);
    }
}

void GameWorld::updateTransforms() {
    auto view = m_registry.view<const Body, const Offset, const component::Scale, component::Transform>(entt::exclude<Delete>);

    for(auto entity : view){
        const auto& body = view.get<const Body>(entity);
        const auto& offset = view.get<const Offset>(entity);
        auto translate = glm::translate(glm::mat4(1), body.position + offset.value);
        auto rotate = glm::mat4(body.orientation);
        glm::mat4_cast(body.orientation);
        auto scale = glm::scale(glm::mat4(1), view.get<const component::Scale>(entity).value);

        auto& transform = view.get<component::Transform>(entity);
        transform.value = translate * rotate * scale;
    }
}

void GameWorld::updateInstanceTransforms() {
    updateInstanceTransform<SphereTag>(sphereEntity);
    updateInstanceTransform<BoxTag>(cubeEntity);
    updateInstanceTransform<Diamond>(diamondEntity);
}

bool GameWorld::intersect(Body &bodyA, Body &bodyB, float dt, Contact& contact) {
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;
    contact.timeOfImpact = 0.0f;

    const auto sphereB = dynamic_cast<SphereShape*>(bodyB.shape.get());
    const auto sphereA = dynamic_cast<SphereShape*>(bodyA.shape.get());
    if(sphereA && sphereB){
        auto posA = bodyA.position;
        auto posB = bodyB.position;

        auto velA = bodyA.linearVelocity;
        auto velB = bodyB.linearVelocity;

        if(sphereSphereDynamic(sphereA, sphereB, posA, posB, velA, velB, dt, contact.worldSpace.pointOnA,
                               contact.worldSpace.pointOnB, contact.timeOfImpact)){
            // set bodies forward to get local space collision points;
            bodyA.update(contact.timeOfImpact);
            bodyB.update(contact.timeOfImpact);

            // convert world space contact to local space;
            contact.LocalSpace.pointOnA = bodyA.worldSpaceToBodySpace(contact.worldSpace.pointOnA);
            contact.LocalSpace.pointOnB = bodyB.worldSpaceToBodySpace(contact.worldSpace.pointOnB);

            contact.normal = glm::normalize(bodyA.position - bodyB.position);

            // unwind time step;
            bodyA.update(-contact.timeOfImpact);
            bodyB.update(-contact.timeOfImpact);

            // calculate the separation distance
            auto ab = bodyB.position - bodyA.position;
            float r = glm::length(ab) - (sphereA->m_radius + sphereB->m_radius);
            contact.separationDistance = r;
            return true;
        }
    }else{
        auto res = conservativeAdvance(bodyA, bodyB, dt, contact);
        return res;
    }

    return false;
}
bool GameWorld::intersect(Body &bodyA, Body &bodyB, Contact& contact) {
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;
    contact.timeOfImpact = 0.0f;

    const auto sphereB = dynamic_cast<SphereShape*>(bodyB.shape.get());
    const auto sphereA = dynamic_cast<SphereShape*>(bodyA.shape.get());
    if(sphereA && sphereB){
        auto posA = bodyA.position;
        auto posB = bodyB.position;

        auto velA = bodyA.linearVelocity;
        auto velB = bodyB.linearVelocity;

        if(sphereSphereStatic(sphereA, sphereB, posA, posB, velA, velB, contact.worldSpace.pointOnA,
                               contact.worldSpace.pointOnB)){

            // convert world space contact to local space;
            contact.LocalSpace.pointOnA = bodyA.worldSpaceToBodySpace(contact.worldSpace.pointOnA);
            contact.LocalSpace.pointOnB = bodyB.worldSpaceToBodySpace(contact.worldSpace.pointOnB);

            contact.normal = glm::normalize(bodyA.position - bodyB.position);

            // calculate the separation distance
            auto ab = bodyB.position - bodyA.position;
            float r = glm::length(ab) - (sphereA->m_radius + sphereB->m_radius);
            contact.separationDistance = r;
            return true;
        }
    }else{
        glm::vec3 pointOnA, pointOnB;
        const auto bias = 0.001f;
        if(GJK::doesIntersect(&bodyA, &bodyB, bias, pointOnA, pointOnB)){
            // There was an intersection, so get the contact data
            auto normal = glm::normalize(pointOnB - pointOnA);

            pointOnA -= normal * bias;
            pointOnB += normal * bias;

            contact.normal = normal;

            contact.worldSpace.pointOnA = pointOnA;
            contact.worldSpace.pointOnB = pointOnB;

            contact.LocalSpace.pointOnA = bodyA.worldSpaceToBodySpace(contact.worldSpace.pointOnA);
            contact.LocalSpace.pointOnB = bodyB.worldSpaceToBodySpace(contact.worldSpace.pointOnB);

            auto ab = bodyA.position - bodyB.position;
            float r = glm::distance(pointOnA, pointOnB);
            contact.separationDistance = -r;
            return true;
        }

        // There was no collison, but we still want the contact data, so get it
        GJK::closestPoint(&bodyA, &bodyB, pointOnA, pointOnB);
        contact.worldSpace.pointOnA = pointOnA;
        contact.worldSpace.pointOnB = pointOnB;

        contact.LocalSpace.pointOnA = bodyA.worldSpaceToBodySpace(contact.worldSpace.pointOnA);
        contact.LocalSpace.pointOnB = bodyB.worldSpaceToBodySpace(contact.worldSpace.pointOnB);

        auto ab = bodyB.position - bodyA.position;
        float r = glm::distance(pointOnA, pointOnB);
        contact.separationDistance = r;
    }

    return false;
}

void GameWorld::resolveContact(Contact &contact) {
    auto bodyA = contact.bodyA;
    auto bodyB = contact.bodyB;

    const auto pointOnA = contact.worldSpace.pointOnA;
    const auto pointOnB = contact.worldSpace.pointOnB;

    auto invMassA = bodyA->invMass;
    auto invMassB = bodyB->invMass;

    auto elasticity = bodyA->elasticity * bodyB->elasticity;

    const auto invWorldInertiaA = bodyA->inverseInertialTensorWorldSpace();
    const auto invWorldInertiaB = bodyB->inverseInertialTensorWorldSpace();

    const auto& n = contact.normal;

    const auto ra = pointOnA - bodyA->centerOfMassWorldSpace();
    const auto rb = pointOnB - bodyB->centerOfMassWorldSpace();

    const auto angularJA = glm::cross(invWorldInertiaA * glm::cross(ra, n), ra);
    const auto angularJB = glm::cross(invWorldInertiaB * glm::cross(rb, n), rb);
    const auto angularFactor = glm::dot(angularJA + angularJB, n);

    // Get the world space velocity of the motion and rotation;
    const auto velA = bodyA->linearVelocity + glm::cross(bodyA->angularVelocity, ra);
    const auto velB = bodyB->linearVelocity + glm::cross(bodyB->angularVelocity, rb);


    // calculate collision impulse
    const auto relVelocity = velA - velB;
    const auto  impulseJ = (1.0f + elasticity) * dot(n, relVelocity) / (invMassA + invMassB + angularFactor);
    const auto vecImpulseJ = n * impulseJ;

    bodyA->applyImpulse(pointOnA, -vecImpulseJ);
    bodyB->applyImpulse(pointOnB, vecImpulseJ);

    // calculate the impluse caused by friction
    const auto frictionA = bodyA->friction;
    const auto frictionB = bodyB->friction;
    const auto friction = frictionA * frictionB;

    const auto normalVelocity = n * glm::dot(n, relVelocity);
    const auto tangentVelocity = relVelocity - normalVelocity;

    if(!isZero(tangentVelocity)) {
        const auto relTangentVelocity = glm::normalize(tangentVelocity);
        const auto inertiaA = glm::cross(invWorldInertiaA * glm::cross(ra, relTangentVelocity), ra);
        const auto inertiaB = glm::cross(invWorldInertiaB * glm::cross(rb, relTangentVelocity), rb);
        const auto invInertia = glm::dot(inertiaA + inertiaB, relTangentVelocity);

        // calculate tangentail impulse for friction
        const auto reducedMass = 1.0f / (invMassA + invMassB + invInertia);
        const auto impulseFriction = tangentVelocity * reducedMass * friction;

        // apply kinetic fiction
        bodyA->applyImpulse(pointOnA, -impulseFriction);
        bodyB->applyImpulse(pointOnB, impulseFriction);
    }

    if(contact.timeOfImpact == 0.0f){
        const auto ta = bodyA->invMass / (bodyA->invMass + bodyB->invMass);
        const auto tb = bodyB->invMass / (bodyA->invMass + bodyB->invMass);

        const auto ds = contact.worldSpace.pointOnB - contact.worldSpace.pointOnA;
        bodyA->position += ds * ta;
        bodyB->position -= ds * tb;
    }
}

void GameWorld::renderUI(VkCommandBuffer commandBuffer) {

    ImGui::Begin("Physics");
    ImGui::SetWindowSize("Physics", {300, 350});

    // camera
    {
        if(ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)){
            auto& cam = cameraController;
            ImGui::Checkbox("Move Camera", &moveCamera);
            ImGui::Text("Position: %s", fmt::format("{}", cam->position()).c_str());
            ImGui::Text("Velocity: %s", fmt::format("{}", cam->velocity()).c_str());
            ImGui::Text("acceleration: %s", fmt::format("{}", cam->acceleration()).c_str());
            ImGui::Text("near: %s", fmt::format("{}", cam->near()).c_str());
            ImGui::Text("far: %s", fmt::format("{}", cam->far()).c_str());
        }
    }

//    if(ImGui::TreeNodeEx("Rigid Bodies", ImGuiTreeNodeFlags_DefaultOpen)) {
//        for (auto i = 0; i < bodies.size(); i++) {
//            auto body = bodies[i];
//            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
//            if(ImGui::TreeNode((void *) (intptr_t) i, "Body %d", i)) {
//                ImGui::Text("position: %s", fmt::format("{}", body->position).c_str());
//                ImGui::Text("velocity (linear): %s", fmt::format("{}", body->linearVelocity).c_str());
//                ImGui::Text("velocity (angular): %s", fmt::format("{}", body->angularVelocity).c_str());
//                ImGui::Text("mass : %f kg", body->invMass == 0 ? 0 : 1.0/body->invMass);
//                ImGui::TreePop();
//            }
//        }
//        ImGui::TreePop();
//    }

    if(ImGui::CollapsingHeader("Simulation Stats", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Objects: %d", bodies.size());
        ImGui::Text("Collisions: %d", simStates.numCollisions);
        ImGui::Text("Physics average %.3f ms/frame", simStates.physicsTime);
    }
    m_runPhysics |= ImGui::Button("Run Physics");
    ImGui::SliderFloat("sim Speed", &timeScale, 0.001f, 1.0f, "%.3f", ImGuiSliderFlags_NoInput);
    ImGui::Text("fps: %d", framePerSecond);
    ImGui::End();

    renderObjectCreateMenu(commandBuffer);
    debugMenu(commandBuffer);
    plugin(IM_GUI_PLUGIN).draw(commandBuffer);
}

void GameWorld::renderObjectCreateMenu(VkCommandBuffer commandBuffer) {
    ImGui::Begin("Create Object");
    ImGui::SetWindowSize("Create Object", {300, 250});
    ImGui::SliderFloat("Mass (kg)", &objectCreateProps.mass, 0.0f, 20.0f);
    ImGui::SliderFloat("Elasticity", &objectCreateProps.elasticity, 0.0f, 1.0f);
    ImGui::SliderFloat("Friction", &objectCreateProps.friction, 0.0f, 1.0f);
    ImGui::SliderFloat("Velocity", &objectCreateProps.speed, 0.0f, 200.0f);
    ImGui::SliderFloat("Rotation", &objectCreateProps.rotation, 0.0f, 30.0f);

    if(ImGui::CollapsingHeader("Object Info", ImGuiTreeNodeFlags_DefaultOpen)){
        ImGui::RadioButton("Box", &objectCreateProps.type, ObjectType::BOX); ImGui::SameLine();
        ImGui::RadioButton("Sphere", &objectCreateProps.type, ObjectType::SPHERE); ImGui::SameLine();
        ImGui::RadioButton("Diamond", &objectCreateProps.type, ObjectType::DIAMOND);
        ImGui::RadioButton("Stack", &objectCreateProps.type, ObjectType::STACK); ; ImGui::SameLine();
        ImGui::RadioButton("Ragdoll", &objectCreateProps.type, ObjectType::RAG_DOLL);

        if(objectCreateProps.type == BOX){
            ImGui::DragFloat3("Size", reinterpret_cast<float*>(&objectCreateProps.size), 0.2f, 0.1f, 5.0f);
        }

        if(objectCreateProps.type == SPHERE){
            ImGui::DragFloat("Radius", &objectCreateProps.radius, 1.0f, 0.1f, 5.0f);
        }

        if(objectCreateProps.type == DIAMOND){
            ImGui::DragFloat("scale", &objectCreateProps.radius, 1.0f, 0.1f, 5.0f);
        }
        if(objectCreateProps.type == STACK){
            objectCreateProps.speed = 0;
            objectCreateProps.rotation = 0;
            ImGui::SliderInt("Height", &objectCreateProps.stack.height, 2, 10);
            ImGui::Text("Type:");
            ImGui::RadioButton("Box", &objectCreateProps.stack.type, ObjectType::BOX); ImGui::SameLine();
            ImGui::RadioButton("Sphere", &objectCreateProps.stack.type, ObjectType::SPHERE); ImGui::SameLine();
            ImGui::RadioButton("Diamond", &objectCreateProps.stack.type, ObjectType::DIAMOND);
        }

        if(objectCreateProps.type == RAG_DOLL){
            objectCreateProps.speed = 0;
            objectCreateProps.rotation = 0;
        }
    }
    ImGui::End();
}

void GameWorld::debugMenu(VkCommandBuffer commandBuffer) {
    ImGui::Begin("Debug");
    ImGui::SetWindowSize("Create Object", {300, 250});
    ImGui::Checkbox("show basis", &m_showBasis);
    ImGui::End();
}

void GameWorld::createSceneObjects() {

    auto cubeBuilder = ObjectBuilder(cubeEntity, &m_registry);

    auto door = g_halfBoxUnit;
    for(auto& p : door){
        p *= glm::vec3(5, 10, 0.2);
    }
    scene.clear();
//
//    auto entityA =
//        cubeBuilder
//            .position(0, 5, 0)
//            .mass(10)
//            .elasticity(0.8)
//            .friction(1.0)
//            .shape(std::make_shared<BoxShape>(door))
//        .build();
//
//    auto entityB =
//        cubeBuilder
//            .position(2.6, 5, 2.6)
//            .orientation(glm::angleAxis(-glm::half_pi<float>(), glm::vec3{0, 1, 0}))
//        .build();
//
//    auto& bodyA = entityA.get<Body>();
//    auto hinge = std::make_unique<ConstraintHingeQuatLimited>();
//    auto anchor = bodyA.position;
//    anchor.x += 2.6;
//    hinge->m_bodyA = &bodyA;
//    hinge->m_anchorA = bodyA.worldSpaceToBodySpace(anchor);
//
//    auto& bodyB = entityB.get<Body>();
//    hinge->m_bodyB = &bodyB;
//    hinge->m_anchorB = bodyB.worldSpaceToBodySpace(anchor);
//
//    hinge->m_axisA = {0, 1, 0};
//    hinge->m_q0 = glm::inverse(bodyA.orientation) * bodyB.orientation;
//    m_constraints.push_back(std::move(hinge));


    auto builder = ObjectBuilder(cubeEntity, &m_registry);

    auto platformEntity =
        builder
            .position(10, 5, 0)
            .shape(std::make_shared<BoxShape>(g_boxPlatform))
            .mass(0)
            .elasticity(0.1)
            .friction(0.5)
        .build();

    auto mover = std::make_unique<ConstraintMover>();
    mover->m_bodyA = &platformEntity.get<Body>();
    m_constraints.push_back(std::move(mover));

    auto boxEntity =
        builder
            .position(10, 6.3, 0)
            .shape(std::make_shared<BoxShape>(g_boxUnit))
            .mass(1)
            .elasticity(0.1)
            .friction(0.9)
        .build();

    sandBoxEntity = SandBox().build(ObjectBuilder(cubeEntity, &m_registry), m_registry);

    auto view = m_registry.view<Body>();
    for(auto entity : view){
        auto body = &view.get<Body>(entity);
        scene.expand(body->shape->bounds());
        bodies.push_back(body);
    }
    shadowMap.updateMatrix(scene, glm::vec3(1));
    simStates.numObjects = bodies.size();
    spdlog::info("scene[min: {}, max: {}]", scene.min, scene.max);
}

std::vector<Entity> GameWorld::createStack(int type, const glm::vec3& position, int height) {
    auto cubeBuilder = ObjectBuilder(cubeEntity, &m_registry);

    std::vector<Entity> entities;

    auto x = 0;
    auto z = 0;
    for(int y = 0; y < height; y++){
        auto offset = ((y & 1) == 0) ? 0.0f : 0.15f;
        auto xx = static_cast<float>(x) + offset;
        auto zz = static_cast<float>(z) + offset;
        auto delta = 0.04f;
        auto scaleHeight = 2.0f + delta;
        auto deltaHeight = 1.0f + delta;
        auto center = glm::vec3(xx * scaleHeight, deltaHeight + y * scaleHeight, zz * scaleHeight);
        center.x += position.x;
        center.z += position.z;
        auto entity =
            cubeBuilder
                .position(center)
                .shape(std::make_shared<BoxShape>(g_boxUnit))
                .mass(1)
                .elasticity(1.0f)
            .build();
        entities.push_back(entity);
    }
    return entities;
}

bool GameWorld::conservativeAdvance(Body &bodyA, Body &bodyB, float dt, Contact &contact) {
    contact.bodyA = &bodyA;
    contact.bodyB = &bodyB;

    auto toi = 0.0f;
    auto numIterations = 0;
    static constexpr int maxIterations = 10;
    // Advance the positions of the bodies until they touch or there's not time left
    while(dt > 0.0f ){
        auto didIntersect = intersect(bodyA, bodyB, contact);
        if(didIntersect){
            contact.timeOfImpact = toi;
            bodyA.update(-toi);
            bodyB.update(-toi);
            return true;
        }

        numIterations++;
        if(numIterations > maxIterations){
           break;
        }

        // Get the vector from the closest point on A to the closest point on B
        auto ab = contact.worldSpace.pointOnB - contact.worldSpace.pointOnA;
        ab = nansafe(glm::normalize(ab));

        auto relVelocity = bodyA.linearVelocity - bodyB.linearVelocity;
        auto orthoSpeed = glm::dot(relVelocity, ab);

        // Add to the orthoSpeed the maximum angular speeds of the relative shapes
        auto angularSpeedA = bodyA.shape->fastLinearSpeed(bodyA.angularVelocity, ab);
        auto angularSpeedB = bodyB.shape->fastLinearSpeed(bodyB.angularVelocity, ab);
        orthoSpeed += angularSpeedA + angularSpeedB;
        if(orthoSpeed <= 0){
            break;
        }
        auto timeToGo = contact.separationDistance / orthoSpeed;
//        spdlog::info("speed: {}, sd: {}, timeToGo: {}", orthoSpeed, contact.separationDistance, timeToGo);

        if(timeToGo > dt){
            break;
        }
        dt -= timeToGo;
        toi += timeToGo;
        bodyA.update(timeToGo);
        bodyB.update(timeToGo);
    }
    // unwind the clock
    bodyA.update(-toi);
    bodyB.update(-toi);
    return false;
}

std::shared_ptr<ConvexHullShape> GameWorld::diamondShape(float size) {
    auto& shape = diamondEntity.get<ConvexHullShape>();
    if(size == 1) {
        return std::shared_ptr<ConvexHullShape>(&shape, [](auto *) {});
    }
    return std::make_shared<ConvexHullShape>(shape, size);
}

void GameWorld::newFrame() {
// FIXME don't rely on the ordering of ECS, order changes when entities get deleted
// FIXME external references to become invalidated when entities within the ECS are deleted
//    auto elapsedTimeMs = static_cast<uint64_t>(elapsedTime * 1000);
//    if(elapsedTimeMs % 1000 == 0 && currentFrame % MAX_IN_FLIGHT_FRAMES == 0){
//        auto view = m_registry.view<Body, component::Parent>(entt::exclude<Delete>);
//        std::vector<entt::entity> removeList;
//        std::vector<Body*> removeBodyList;
//
//        for(auto entity : view){
//            auto& body = view.get<Body>(entity);
//            auto camPos = cameraController->position();
//            auto dist = glm::distance(camPos, body.position);
//            if(dist > cameraController->far()){
//                body.invMass = 0;
//                body.linearVelocity = glm::vec3(0);
//                body.angularVelocity = glm::vec3(0);
//
//                spdlog::info("deleting body: {}", body.id);
//                m_manifolds.removeContactsFor(&body);
//                removeBodyList.push_back(&body);
//                removeList.push_back(entity);
//                auto parentEntity = view.get<component::Parent>(entity).entity;
//                m_registry.get<component::Render>(parentEntity).instanceCount--;
//                spdlog::info("instance Count : {}", m_registry.get<component::Render>(parentEntity).instanceCount);
//            }
//        }
//
//        for(int i = static_cast<int>(bodies.size()) - 1; i >= 0; i--){
//            auto body = bodies[i];
//            auto itr = std::find_if(removeBodyList.begin(), removeBodyList.end(), [&](const auto pBody){
//                return pBody->id == body->id;
//            });
//            if(itr != removeBodyList.end()) {
//                bodies.erase(bodies.begin() + i);
//            }
//        }
//
//        for(auto entity : removeList){
//            m_registry.emplace<Delete>(entity);
//            m_registry.destroy(entity);
//        }
//    }
//
//    if(currentFrame % MAX_IN_FLIGHT_FRAMES == 0){
//        createObject();
//    }

}

void GameWorld::createRenderBasisPipeline() {
        auto builder = device.graphicsPipelineBuilder();
        renderBasis.pipeline =
            builder
                .shaderStage()
                    .vertexShader(load("basis.vert.spv"))
                    .fragmentShader(load("basis.frag.spv"))
                .vertexInputState()
                    .addVertexBindingDescription(0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX)
                    .addVertexAttributeDescription(0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(Vertex, position))
                    .addVertexAttributeDescription(1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, offsetOf(Vertex, color))
                .inputAssemblyState()
                    .lines()
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
                    .frontFaceCounterClockwise()
                    .polygonModeFill()
					.lineWidth(1.5)
                    .cullNone()
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
                .name("render_basis")
            .build(renderBasis.layout);
}

void GameWorld::initBasis() {
    auto vertices = basis::create();
    renderBasis.vertices = device.createDeviceLocalBuffer(vertices.data(), BYTE_SIZE(vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);

}

void GameWorld::renderObjectBasis(VkCommandBuffer commandBuffer) {
    auto view = m_registry.view<Body, Offset>();

    VkDeviceSize offset = 0;
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, renderBasis.pipeline);
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, renderBasis.vertices, &offset);
    for(auto entity : view){
        auto& body = view.get<Body>(entity);
        auto offset = view.get<Offset>(entity).value;
        auto translate = glm::translate(glm::mat4(1), body.position + offset);
        auto model =  translate * glm::mat4(body.orientation) * glm::scale(glm::mat4(1), glm::vec3(2));
        cameraController->push(commandBuffer, renderBasis.layout, model);
        vkCmdDraw(commandBuffer, renderBasis.vertices.size/sizeof(Vertex), 1, 0, 0);
    }
    cameraController->setModel(glm::mat4(1));
}

void GameWorld::createDescriptorSetLayouts() {
    render.uboDescriptorSetLayout =
        device.descriptorSetLayoutBuilder()
            .binding(0)
            .descriptorType(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)
            .descriptorCount(1)
            .shaderStages(VK_SHADER_STAGE_VERTEX_BIT)
        .createLayout();

    shadowMap.shadowMapDescriptorSetLayout =
        device.descriptorSetLayoutBuilder()
            .binding(0)
            .descriptorCount(1)
            .descriptorType(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)
            .shaderStages(VK_SHADER_STAGE_FRAGMENT_BIT)
        .createLayout();
}

void GameWorld::updateDescriptorSets() {
    updateUboDescriptorSet();
    updateShadowMapDescriptorSet();
}

void GameWorld::updateShadowMapDescriptorSet() {
    shadowMap.shadowMapDescriptorSet = descriptorPool.allocate({shadowMap.shadowMapDescriptorSetLayout}).front();
    device.setName<VK_OBJECT_TYPE_DESCRIPTOR_SET>("shadow_map", shadowMap.shadowMapDescriptorSet);
    auto writes = initializers::writeDescriptorSets<1>();

    VkDescriptorImageInfo imageInfo{ shadowMap.sampler, shadowMap.framebufferAttachment.imageView, VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL };
    writes[0].dstSet = shadowMap.shadowMapDescriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo = &imageInfo;

    device.updateDescriptorSets(writes);
}

void GameWorld::updateUboDescriptorSet(){
    render.uboDescriptorSet = descriptorPool.allocate({ render.uboDescriptorSetLayout }).front();

    auto writes = initializers::writeDescriptorSets<1>();
    writes[0].dstSet = render.uboDescriptorSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;

    VkDescriptorBufferInfo bufferInfo{ uboBuffer, 0, VK_WHOLE_SIZE};
    writes[0].pBufferInfo = &bufferInfo;

    device.updateDescriptorSets(writes);
}

void GameWorld::createUboBuffer() {
    updateUbo();
    uboBuffer = device.createCpuVisibleBuffer(&ubo, sizeof(ubo), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
}

void GameWorld::updateUbo() {
    auto& camera = cameraController->cam();
    ubo.view = camera.view;
    ubo.projection = camera.proj;
    ubo.lightSpaceMatrix = shadowMap.lightSpaceMatrix;
}

void GameWorld::initShadowMap(){
    ShadowMap::initShadowMap(device, fileManager, shadowMap, scene);
}

void GameWorld::renderEntitiesLocal(VkCommandBuffer commandBuffer, entt::registry &registry) {
    auto camView = registry.view<const component::Camera>();

    Camera* camera{nullptr};
    for(auto entity : camView){
        auto cam = camView.get<const component::Camera>(entity);
        if(cam.main){
            camera = cam.camera;
            break;
        }
    }
    if(!camera){
        spdlog::error("no camera entity set");
    }
    assert(camera);

    auto view = registry.view<component::Render, component::Transform,  component::Pipelines>();
    static std::vector<VkBuffer> buffers;
    for(auto entity : view){
//    view.each([&](const component::Render& renderComp, const auto& transform,  const auto& pipelines)
        auto& renderComp = view.get<component::Render>(entity);
        auto& transform = view.get<component::Transform>(entity);
        auto& pipelines = view.get<component::Pipelines>(entity);
        if(renderComp.instanceCount > 0) {
            auto model = transform.value;
            camera->model = model;
            std::vector<VkDeviceSize> offsets(renderComp.vertexBuffers.size(), 0);
            buffers.clear();
            for(auto& buffer : renderComp.vertexBuffers){
                buffers.push_back(buffer.buffer);
            }
            vkCmdBindVertexBuffers(commandBuffer, 0, COUNT(buffers), buffers.data(), offsets.data());
            if (renderComp.indexCount > 0) {
                vkCmdBindIndexBuffer(commandBuffer, renderComp.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
            }
            for (const auto &pipeline : pipelines) {
                vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,  (VkPipeline)pipeline.pipeline);
                vkCmdPushConstants(commandBuffer, (VkPipelineLayout)pipeline.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Camera), camera);
                if (!pipeline.descriptorSets.empty()) {
                    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, (VkPipelineLayout)pipeline.layout, 0,
                                            COUNT(pipeline.descriptorSets), (VkDescriptorSet*)pipeline.descriptorSets.data(), 0,
                                            VK_NULL_HANDLE);
                }
                for (auto primitive : renderComp.primitives) {
                    if (renderComp.indexCount > 0) {
                        primitive.drawIndexed(commandBuffer, 0, renderComp.instanceCount);
                    } else {
                        primitive.draw(commandBuffer, 0, renderComp.instanceCount);
                    }
                }
            }
        }
    }
//    );
}

void GameWorld::renderSceneObjects(VkCommandBuffer commandBuffer) {

    static std::array<VkDescriptorSet, 2> sets;
    sets[0] = render.uboDescriptorSet;
    sets[1] = shadowMap.shadowMapDescriptorSet;
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, render.pipeline);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, render.layout, 0, COUNT(sets), sets.data(), 0, nullptr);

    auto view = m_registry.view<component::Render, component::Transform>(entt::exclude<SkyBoxTag>);
    static std::vector<VkBuffer> buffers;
    for(auto entity : view) {
        auto& renderComp = view.get<component::Render>(entity);

        if(renderComp.instanceCount > 0) {
            auto& transform = view.get<component::Transform>(entity);
            vkCmdPushConstants(commandBuffer, render.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &transform.value);
            std::vector<VkDeviceSize> offsets(renderComp.vertexBuffers.size(), 0);
            buffers.clear();
            for(auto& buffer : renderComp.vertexBuffers){
                buffers.push_back(buffer.buffer);
            }
            if (renderComp.indexCount > 0) {
                vkCmdBindIndexBuffer(commandBuffer, renderComp.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
            }
            vkCmdBindVertexBuffers(commandBuffer, 0, COUNT(buffers), buffers.data(), offsets.data());
            for (auto primitive : renderComp.primitives) {
                if (renderComp.indexCount > 0) {
                    primitive.drawIndexed(commandBuffer, 0, renderComp.instanceCount);
                } else {
                    primitive.draw(commandBuffer, 0, renderComp.instanceCount);
                }
            }
        }

    }
}

void GameWorld::renderSkyBox(VkCommandBuffer commandBuffer) {
    static glm::mat4 model{1};

    uint32_t indexCount = skyBox.cube.indices.size /sizeof(int32_t);
    uint32_t instanceCount = 1;
    int32_t vertexOffset = 0;
    uint32_t firstInstance = 0;
    VkDeviceSize offset = 0;

    vkCmdBindVertexBuffers(commandBuffer, 0, 1, skyBox.cube.vertices, &offset);
    vkCmdBindIndexBuffer(commandBuffer, skyBox.cube.indices, 0, VK_INDEX_TYPE_UINT32);
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, *skyBox.pipeline);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, *skyBox.layout, 0, 1, &skyBox.descriptorSet, 0, VK_NULL_HANDLE);
    cameraController->push(commandBuffer, *skyBox.layout, model);
    vkCmdDrawIndexed(commandBuffer, indexCount, instanceCount, firstInstance, vertexOffset, firstInstance);
}

void GameWorld::initFrustum() {
    auto createFrustum = [&]{
        auto vertices = Ndc::points;
        auto worldSpaceMatrix = glm::inverse(shadowMap.lightProjection * shadowMap.lightView);
        for(auto& vertex : vertices){
            vertex = worldSpaceMatrix * vertex;
        }
        frustum.vertices = device.createDeviceLocalBuffer(vertices.data(), BYTE_SIZE(vertices), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
        frustum.indices = device.createDeviceLocalBuffer(Ndc::indices.data(), BYTE_SIZE(Ndc::indices), VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    };

    auto createFrustumPipeline = [&]{
        frustum.pipeline =
                device.graphicsPipelineBuilder()
                        .shaderStage()
                        .vertexShader(load("frustum.vert.spv"))
                        .fragmentShader(load("frustum.frag.spv"))
                        .vertexInputState()
                        .addVertexBindingDescription(0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX)
                        .addVertexAttributeDescription(0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0)
                        .inputAssemblyState()
                        .lines()
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
                        .cullNone()
                        .frontFaceCounterClockwise()
                        .polygonModeFill()
                        .lineWidth(3.0)
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
                        .name("frustum")
                        .pipelineCache(pipelineCache)
                        .build(frustum.layout);

    };

    createFrustum();
    createFrustumPipeline();
}

void GameWorld::renderFrustum(VkCommandBuffer commandBuffer) {
    static VkDeviceSize offset = 0;
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, frustum.pipeline);
    cameraController->push(commandBuffer, frustum.layout);
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, frustum.vertices, &offset);
    vkCmdBindIndexBuffer(commandBuffer, frustum.indices, 0, VK_INDEX_TYPE_UINT32);
    uint32_t indexCount = frustum.indices.size/sizeof(uint32_t);
    vkCmdDrawIndexed(commandBuffer, indexCount, 1, 0, 0, 0);
}
