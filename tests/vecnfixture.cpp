#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <algorithm>
#include <numeric>
#include <glm/glm.hpp>
#include "vecN.hpp"

class VecNFixture : public ::testing::Test {
protected:
};

TEST_F(VecNFixture, InitializeVecNToSingleValue){
    vec<10> vector{5.0f};
    for(int i = 0; i < 10; i++){
        ASSERT_EQ(vector[i], 5.0f);
    }
}

TEST_F(VecNFixture, InitialzeVecNWithListOfValues){
    vec<5> vector{0.f, 1.f, 2.f, 3.f, 4.f};
    for(int i = 0; i < 5; i++){
        ASSERT_EQ(static_cast<float>(i), vector[i]);
    }
}

TEST_F(VecNFixture, MultiplyVectorNByScalarValue){
    vec<5> vector{1, 2, 3, 4, 5};

    vec<5> res = vector * 2.0f;
    for(int i = 0; i < 5; i++){
        auto expected = float((i+1) * 2);
        ASSERT_EQ(res[i], expected);
    }
}

TEST_F(VecNFixture, MultiplyAssignmentOnVectorN){
    vec<5> vector{1, 2, 3, 4, 5};

    vector *= 2.0f;
    for(int i = 0; i < 5; i++){
        auto expected = float((i+1) * 2);
        ASSERT_EQ(vector[i], expected);
    }
}

TEST_F(VecNFixture, AddTwoVectors){
    vec<5> v1{1, 2, 3, 4, 5};
    vec<5> v2{6, 7, 8, 9, 10};

    auto v3 = v1 + v2;

    ASSERT_EQ(v3[0], 7.0f);
    ASSERT_EQ(v3[1], 9.0f);
    ASSERT_EQ(v3[2], 11.0f);
    ASSERT_EQ(v3[3], 13.0f);
    ASSERT_EQ(v3[4], 15.0f);
}

TEST_F(VecNFixture, AddVectorAssignment){
    vec<5> v1{1, 2, 3, 4, 5};
    vec<5> v2{6, 7, 8, 9, 10};

    v1 += v2;

    ASSERT_EQ(v1[0], 7.0f);
    ASSERT_EQ(v1[1], 9.0f);
    ASSERT_EQ(v1[2], 11.0f);
    ASSERT_EQ(v1[3], 13.0f);
    ASSERT_EQ(v1[4], 15.0f);
}

TEST_F(VecNFixture, SubtractTwoVectors){
    vec<5> v1{1, 2, 3, 4, 5};
    vec<5> v2{6, 7, 8, 9, 10};

    auto v3 = v2 - v1;

    ASSERT_EQ(v3[0], 5.0f);
    ASSERT_EQ(v3[1], 5.0f);
    ASSERT_EQ(v3[2], 5.0f);
    ASSERT_EQ(v3[3], 5.0f);
    ASSERT_EQ(v3[4], 5.0f);
}


TEST_F(VecNFixture, SubtractVectorsAssignment){
    vec<5> v1{1, 2, 3, 4, 5};
    vec<5> v2{6, 7, 8, 9, 10};

    v2 -= v1;

    ASSERT_EQ(v2[0], 5.0f);
    ASSERT_EQ(v2[1], 5.0f);
    ASSERT_EQ(v2[2], 5.0f);
    ASSERT_EQ(v2[3], 5.0f);
    ASSERT_EQ(v2[4], 5.0f);
}

TEST_F(VecNFixture, DotProduct) {
    vec<5> v1{1, 2, 3, 4, 5};
    vec<5> v2{2, 1, 5, 2, 3};

    auto result = v1.dot(v2);

    ASSERT_EQ(result, 42.0);
}

TEST_F(VecNFixture, ResetVectorN){
    vec<5> v{1, 2, 3, 4, 5};

    v.clear();

    for(int i= 0; i < 5; i++){
        ASSERT_EQ(v[i], 0);
    }
}

TEST_F(VecNFixture, InitializeWithGlmVector){
    vec12 v{
        glm::vec4(0, 1, 2, 3),
        glm::vec4(4, 5, 6, 7),
        glm::vec4(8, 9, 10, 11)
    };

    for(int i = 0; i < 12; i++){
        ASSERT_EQ(v[i], i);
    }
}

TEST_F(VecNFixture, convertToGlmVectors){
    auto v0 = glm::vec4(0, 1, 2, 3);
    auto v1 = glm::vec4(4, 5, 6, 7);
    auto v2 = glm::vec4(8, 9, 10, 11);

    const vec12 v12{v0, v1, v2};

    auto v = v12.split<4>();

    ASSERT_EQ(v.size(), 3);
    ASSERT_EQ(v[0], v0);
    ASSERT_EQ(v[1], v1);
    ASSERT_EQ(v[2], v2);
}

TEST_F(VecNFixture, NegateVector){
    vec<4> v{1, 2, 3, -2};
    auto nv = -v;

    ASSERT_EQ(nv[0], -1);
    ASSERT_EQ(nv[1], -2);
    ASSERT_EQ(nv[2], -3);
    ASSERT_EQ(nv[3], 2);
}

TEST_F(VecNFixture, CopyConstructor){
    vec12 v1{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    vec12 v2(v1);

    for(auto i = 0; i < 12; i++){
        ASSERT_EQ(v1[i], v2[i]);
    }
}

TEST_F(VecNFixture, CheckCopyConstructorIsNotShallow){
    vec12 v1{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    vec12 v2(v1);

    for(int i = 0; i < 12; i++){
        v1[i] += 1;
    }

    for(auto i = 0; i < 12; i++){
        ASSERT_NE(v1[i], v2[i]);
    }
}

TEST_F(VecNFixture, MoveConstructor){
    vec12 v1{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    vec12 v2(v1);
    vec12 v3(static_cast<vec12&&>(v2));

    for(auto i = 0; i < 12; i++){
        ASSERT_EQ(v1[i], v3[i]);
    }
}

TEST_F(VecNFixture, CheckMoveConstructorIsNotShallow){
    vec12 v1{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    vec12 v2(v1);
    vec12 v3(static_cast<vec12&&>(v2));

    for(int i = 0; i < 12; i++){
        v1[i] += 1;
    }

    for(auto i = 0; i < 12; i++){
        ASSERT_NE(v1[i], v3[i]);
    }
}