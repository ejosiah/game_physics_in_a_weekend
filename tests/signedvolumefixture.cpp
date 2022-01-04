#include <glm/glm.hpp>
#include <array>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "signedVolume.hpp"

class SignedVolumeFixture : public ::testing::Test{
protected:
    static constexpr std::array<glm::vec3, 4> g_originPoints{
        glm::vec3{0, 0, 0},
        glm::vec3{1, 0, 0},
        glm::vec3{0, 1, 0},
        glm::vec3{0, 0, 1}
    };

    std::array<glm::vec3, 4> m_points{};
    glm::vec4 m_lambdas{};
    glm::vec3 m_v{};

    void signedVolume3D(const glm::vec3& point, const glm::vec4& expectedLambdas, const glm::vec3& expectedV){
        for(auto i = 0; i < 4; i++){
            m_points[i] = g_originPoints[i] + point;
        }
        signedVolume3D(m_points, expectedLambdas, expectedV);
    }

    void signedVolume3D(const std::array<glm::vec3, 4> points, const glm::vec4& expectedLambdas, const glm::vec3& expectedV){
        auto lambdas = SignedVolume::_3D(points[0], points[1], points[2], points[3]);
        auto v = glm::vec3(0);
        for(auto i = 0; i < 4; i++){
            v += points[i] * lambdas[i];
        }
        auto epsilon = 1E-3;
        ASSERT_NEAR(lambdas.x, expectedLambdas.x, epsilon);
        ASSERT_NEAR(lambdas.y, expectedLambdas.y, epsilon);
        ASSERT_NEAR(lambdas.z, expectedLambdas.z, epsilon);
        ASSERT_NEAR(lambdas.w, expectedLambdas.w, epsilon);

        ASSERT_NEAR(v.x, expectedV.x, epsilon);
        ASSERT_NEAR(v.y, expectedV.y, epsilon);
        ASSERT_NEAR(v.z, expectedV.z, epsilon);
    }
};

TEST_F(SignedVolumeFixture, signedVolume3DOriginAtZero){
    signedVolume3D({1, 1, 1}, {1, 0, 0, 0}, {1, 1, 1});
}

TEST_F(SignedVolumeFixture, signedVolume3DPoint025){
    signedVolume3D({-0.25, -0.25, -0.25}, {0.25, 0.25, 0.25, 0.25}, {0, 0, 0});
}

TEST_F(SignedVolumeFixture, signedVolume3DPointMinusOne){
    signedVolume3D({-1, -1, -1}, {0, 0.333, 0.333, 0.333}, {-0.667, -0.667, -0.667});
}

TEST_F(SignedVolumeFixture, signedVolume3DOriginPointX1Y1Z05){
    signedVolume3D({1,  1, -0.5}, {0.5, 0, 0, 0.5}, {1, 1, 0});
}

TEST_F(SignedVolumeFixture, signedVolume3DOriginNotAtZero){
    std::array<glm::vec3, 4> points{};
    points[0] = {51.1996613f, 26.1989613f, 1.91339576f};
    points[1] = {-51.0567360f, -26.0565681f, -0.436143428f};
    points[2] = {50.8978920f, -24.1035538f, -1.04042661f};
    points[3] = {-49.1021080f, 25.8964462f, -1.04042661f};
    signedVolume3D(points, {0.290, 0.302, 0.206, 0.202}, {0, 0, 0});
}

TEST_F(SignedVolumeFixture, returnZerosOnDegenrateSimplex2D){
    auto lambdas = SignedVolume::_2D({1, -2, -2}, {1, 2, 2}, {1, 0, 0});
    ASSERT_FLOAT_EQ(lambdas.x, 0);
    ASSERT_FLOAT_EQ(lambdas.y, 0);
    ASSERT_FLOAT_EQ(lambdas.z, 0);
}

TEST_F(SignedVolumeFixture, returnZerosOnDegenrateSimplex1D) {
    auto lambdas = SignedVolume::_1D({1, -2, -2}, {1, -2, -2});
    ASSERT_FLOAT_EQ(lambdas.x, 0);
    ASSERT_FLOAT_EQ(lambdas.y, 0);
}

TEST_F(SignedVolumeFixture, returnZerosOnDegenrateSimplex3D){
    auto lambdas = SignedVolume::_3D({1, -2, 0}, {1, 2, 0}, {1, 0, 0},  {1, 4, 0});
    ASSERT_FLOAT_EQ(lambdas.x, 0);
    ASSERT_FLOAT_EQ(lambdas.y, 0);
    ASSERT_FLOAT_EQ(lambdas.z, 1);
    ASSERT_FLOAT_EQ(lambdas.w, 0);
}

TEST_F(SignedVolumeFixture, anotherDegenerateSimplex3D){
    auto lambdas = SignedVolume::_3D(
              {50.618, -0.010, 25.044}
            , {-52.208, -0.009, -25.070}
            , {49.262, -0.030, -26.426}
            , {-50.853, 0.011, -23.600});
    
    auto error = 1E-3;
    ASSERT_NEAR(lambdas.x, 0.48629, error);
    ASSERT_NEAR(lambdas.y, 0.00827329, error);
    ASSERT_NEAR(lambdas.z, 0.0151818, error);
    ASSERT_NEAR(lambdas.w, 0.490255, error);
}