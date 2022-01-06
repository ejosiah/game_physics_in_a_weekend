#include "GameWorld.hpp"
#include <vulkan_util/GraphicsPipelineBuilder.hpp>
#include <vulkan_util/DescriptorSetBuilder.hpp>
#include <vulkan_util/glm_format.h>
#include "utility.hpp"
#include <vulkan_util/ImGuiPlugin.hpp>
#include <vulkan_util/random.h>
#include "sandbox.hpp"
#include "objects.hpp"
#include "diamond.hpp"
#include "gjk.hpp"

GameWorld::GameWorld(const Settings& settings) : VulkanBaseApp("Game Physics In One Weekend", settings) {
    fileManager.addSearchPath("spv");
    fileManager.addSearchPath("shaders");
    createSphereAction = &mapToMouse(static_cast<int>(MouseEvent::Button::RIGHT), "sphere", Action::detectInitialPressOnly());
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
    createCubeEntity();
    createDiamondEntity();
//    createSphereInstance({1, 1, 1}, 0.0f, 1.0f, 1000, {0, -1000, 0});
//    createSphereInstance({1, 0, 0}, 0.0f, 1.0f, 1, {0, 1, 0});
    createSceneObjects();

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
    renderUI(commandBuffer);

    vkCmdEndRenderPass(commandBuffer);

    vkEndCommandBuffer(commandBuffer);

    return &commandBuffer;
}

void GameWorld::update(float time) {
    static float dt = 1.0f/targetFrameRate;
    static int dtMs = static_cast<int>(dt * 1000);
    int elapsedTimeMs = static_cast<int>(elapsedTime * 1000);
    if(m_runPhysics && elapsedTimeMs % dtMs == 0){
        static uint64_t physicsFrames = 1;
        auto duration = profile([&]{ fixedUpdate(dt); });
        auto frameTime = static_cast<float>(duration.count());
        simStates.physicsTime = glm::mix(simStates.physicsTime, frameTime, 1.0/static_cast<float>(physicsFrames));
        physicsFrames++;
    }
    if(moveCamera) {
        cameraController->update(time);
    }

}

void GameWorld::fixedUpdate(float dt) {
    float deltaTime = dt/float(iterations);
    for(auto i = 0; i < iterations; i++){
        updateBodies(deltaTime * timeScale);
    }
    updateTransforms();
    updateInstanceTransforms();
}

void GameWorld::checkAppInputs() {
    if(createSphereAction->isPressed()){
        const auto& cam = cameraController->cam();
        auto dir = glm::unProject(glm::vec3(mouse.position, 1), cam.view, cam.proj, glm::vec4{0, 0, width, height});
        dir = glm::normalize(dir);
        auto t = glm::length(cameraController->position());
        auto pos = cameraController->position() + dir * t;
        createSphereInstance(randomColor(), 1.0f, 0.8f, 1.0f, pos);
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

void GameWorld::createSphereInstance(glm::vec3 color, float mass, float elasticity, float radius, const glm::vec3& center) {

    auto entity =
        ObjectBuilder(sphereEntity, &registry)
            .shape(std::make_shared<SphereShape>(radius))
            .color(color)
            .position(center)
            .mass(mass)
            .friction(1)
            .elasticity(elasticity)
        .build();
    bodies.push_back(&entity.get<Body>());

}

void GameWorld::updateBodies(float dt) {
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
            contacts.push_back(contact);
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

    for(auto& constraint : m_constraints){
        constraint->solve();
    }

    for(auto& constraint : m_constraints){
        constraint->postSolve();
    }

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
    auto view = registry.view<const Body, const Offset, const component::Scale, component::Transform>();

    for(auto entity : view){
        const auto& body = view.get<const Body>(entity);
        const auto& offset = view.get<const Offset>(entity);
        auto translate = glm::translate(glm::mat4(1), body.position + offset.value);
        auto rotate = glm::mat4(body.orientation);
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
        ImGui::Text("Objects: %d", simStates.numObjects);
        ImGui::Text("Collisions: %d", simStates.numCollisions);
        ImGui::Text("Physics average %.3f ms/frame", simStates.physicsTime);
    }
    m_runPhysics |= ImGui::Button("Run Physics");
    ImGui::SliderFloat("sim Speed", &timeScale, 0.001f, 1.0f, "%.3f", ImGuiSliderFlags_NoInput);
    ImGui::Text("fps: %d", framePerSecond);
    ImGui::End();

    plugin(IM_GUI_PLUGIN).draw(commandBuffer);
}

void GameWorld::createSceneObjects() {

//    auto sphereBuilder = ObjectBuilder(sphereEntity, &registry);
//    sphereBuilder
//        .position(10, 3, 0)
//        .linearVelocity(-100, 0, 0)
//        .mass(1)
//        .elasticity(0.5)
//        .friction(0.5)
//        .shape(std::make_shared<SphereShape>(0.5f))
//    .build();
//
//    auto diamondBuilder = ObjectBuilder(diamondEntity, &registry);
//    diamondBuilder
//        .position(-10, 3, 0)
//        .linearVelocity(100, 0, 0)
//        .angularVelocity(0, 0, 10)
//        .mass(1.0)
//        .elasticity(0.5)
//        .friction(0.5)
//        .shape(diamondShape())
//    .build().add<Diamond>();

    auto cubeBuilder = ObjectBuilder(cubeEntity, &registry);

    auto entityA =
        cubeBuilder
            .position(0, 5.0, 0)
            .shape(std::make_shared<BoxShape>(g_boxSmall))
            .mass(0)
            .elasticity(1.0f)
        .build();

    auto entityB =
        cubeBuilder
            .position(1, 5, 0)
            .mass(1)
        .build();

    auto& bodyA = entityA.get<Body>();
    auto& bodyB = entityB.get<Body>();

    const auto jointWorldSpaceAnchor = bodyA.position;
    std::unique_ptr<Constraint> joint = std::make_unique<ConstraintDistance>();
    joint->m_bodyA = &bodyA;
    joint->m_anchorA = joint->m_bodyA->worldSpaceToBodySpace(jointWorldSpaceAnchor);

    joint->m_bodyB = &bodyB;
    joint->m_anchorB = joint->m_bodyB->worldSpaceToBodySpace(jointWorldSpaceAnchor);
    m_constraints.push_back(std::move(joint));

//    Objects().build(cubeBuilder, registry);

    sandBoxEntity =  SandBox().build(ObjectBuilder(cubeEntity, &registry), registry);

//    diamondEntity = createEntity("diamond");
//    Diamond().build(device, render.pipeline, render.layout, diamondEntity, registry);


    auto view = registry.view<Body>();
    for(auto entity : view){
        auto body = &view.get<Body>(entity);
        bodies.push_back(body);
    }

    simStates.numObjects = bodies.size();
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

std::shared_ptr<ConvexHullShape> GameWorld::diamondShape() {
    auto& shape = diamondEntity.get<ConvexHullShape>();
    return std::shared_ptr<ConvexHullShape>( &shape, [](auto* ){});
}
