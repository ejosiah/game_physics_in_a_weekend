#include <glm/glm.hpp>
#include <array>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "gjk.hpp"
#include "objectbuilder.hpp"
#include <entt/entt.hpp>

class GJKFixture : public ::testing::Test{
protected:
    std::unique_ptr<ObjectBuilder> m_builder;
    entt::registry m_registry;
    Entity m_cubeEntity;

    void SetUp() override {
        m_cubeEntity = createEntity("cube");
        m_builder = std::make_unique<ObjectBuilder>(m_cubeEntity, &m_registry);
    }

    Entity createEntity(const std::string& name){
        Entity entity{m_registry };
        entity.add<component::Position>();
        entity.add<component::Rotation>();
        entity.add<component::Scale>();
        entity.add<component::Transform>();
        auto& nameTag = entity.add<component::Name>();
        nameTag.value = name.empty() ? fmt::format("{}_{}", "Entity", m_registry.size()) : name;
        return entity;
    }

    ObjectBuilder& builder() const {
        return *m_builder.get();
    }
};

TEST_F(GJKFixture, TwoSphereNotInCollision){
    glm::vec3 centerA{0, 10, 0};
    glm::vec3 centerB{10, 15, 10};
    auto sphereA =
            builder()
                .position(centerA)
                .shape(std::make_shared<SphereShape>(1))
            .buildBody();

    auto sphereB =
            builder()
                .position(centerB)
                .shape(std::make_shared<SphereShape>(1))
            .buildBody();

    glm::vec3 pointOnA, pointOnB;

    auto collides = GJK::doesIntersect(&sphereA, &sphereB, 0.0f, pointOnA, pointOnB);
    ASSERT_FALSE(collides) << "spheres should not be in collision";

}

TEST_F(GJKFixture, TwoSpheresInCollision){
    glm::vec3 centerA{0, 10, 0};
    glm::vec3 centerB{0.5, 10.5, 0};
    auto sphereA =
            builder()
                    .position(centerA)
                    .shape(std::make_shared<SphereShape>(1))
                    .buildBody();

    auto sphereB =
            builder()
                    .position(centerB)
                    .shape(std::make_shared<SphereShape>(1))
                    .buildBody();

    glm::vec3 pointOnA, pointOnB;

    auto bias = 1E-3f;
    auto collides = GJK::doesIntersect(&sphereA, &sphereB, 1E-3f, pointOnA, pointOnB);
    ASSERT_TRUE(collides) << "spheres should be in collision";

    glm::vec3 contactNormal = glm::normalize(centerB - centerA);
    auto expectedPointOnA = centerA + contactNormal + contactNormal * bias;
    auto expectedPointOnB = centerB - contactNormal - contactNormal * bias;


    ASSERT_NEAR(pointOnA.x, expectedPointOnA.x, 1E-3f);
    ASSERT_NEAR(pointOnA.y, expectedPointOnA.y, 1E-3f);
    ASSERT_NEAR(pointOnA.z, expectedPointOnA.z, 1E-3f);

    ASSERT_NEAR(pointOnB.x, expectedPointOnB.x, 1E-3f);
    ASSERT_NEAR(pointOnB.y, expectedPointOnB.y, 1E-3f);
    ASSERT_NEAR(pointOnB.z, expectedPointOnB.z, 1E-3f);
}

TEST_F(GJKFixture, returnClosestPointBetweenToSpheres){
    glm::vec3 centerA{0, 10, 0};
    glm::vec3 centerB{10, 15, 10};
    auto sphereA =
            builder()
                .position(centerA)
                .shape(std::make_shared<SphereShape>(1))
            .buildBody();

    auto sphereB =
            builder()
                .position(centerB)
                .shape(std::make_shared<SphereShape>(1))
            .buildBody();

    glm::vec3 pointOnA, pointOnB;
    GJK::closestPoint(&sphereA, &sphereB, pointOnA, pointOnB);

    glm::vec3 contactNormal = glm::normalize(centerB - centerA);
    auto expectedPointOnA = centerA + contactNormal;
    auto expectedPointOnB = centerB - contactNormal;

    ASSERT_NEAR(pointOnA.x, expectedPointOnA.x, 1E-3f);
    ASSERT_NEAR(pointOnA.y, expectedPointOnA.y, 1E-3f);
    ASSERT_NEAR(pointOnA.z, expectedPointOnA.z, 1E-3f);

    ASSERT_NEAR(pointOnB.x, expectedPointOnB.x, 1E-3f);
    ASSERT_NEAR(pointOnB.y, expectedPointOnB.y, 1E-3f);
    ASSERT_NEAR(pointOnB.z, expectedPointOnB.z, 1E-3f);
}