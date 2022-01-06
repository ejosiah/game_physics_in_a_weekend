#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "mat.hpp"
#include <fmt/format.h>

class MatFixture : public ::testing::Test {
protected:
};

TEST_F(MatFixture, IntializeMatWithVectors){
    mat<3, 5> mat{
            {0, 1, 2, 3, 4},
            {5, 6, 7, 8, 9},
            {10, 11, 12, 13, 14}
    };

    for(int i = 0; i < 5; i++){
        ASSERT_EQ(mat[0][i], i);
    }

    for(int i = 0; i < 5; i++){
        ASSERT_EQ(mat[1][i], i + 5);
    }

    for(int i = 0; i < 5; i++){
        ASSERT_EQ(mat[2][i], i + 10);
    }
}

TEST_F(MatFixture, MultiplyByVector){
    mat<3, 5> mat{
            {0, 1, 2, 2, 1},
            {1, 1, 3, 1, 2},
            {1, 2, 3, 1, 3}
    };

    vec<5> v{1, 2, 1, 3, 1};

    vec<3> v1 = mat * v;

    ASSERT_EQ(v1[0], 11);
    ASSERT_EQ(v1[1], 11);
    ASSERT_EQ(v1[2], 14);

}

TEST_F(MatFixture, MultiplyByScalarValue){
    mat<3, 5> mat{
            {0, 1, 2, 3, 4},
            {5, 6, 7, 8, 9},
            {10, 11, 12, 13, 14}
    };

    auto m = mat * 2.0f;

    for(int i = 0; i < 5; i++){
        ASSERT_EQ(m[0][i], i * 2);
    }

    for(int i = 0; i < 5; i++){
        ASSERT_EQ(m[1][i], (i + 5) * 2);
    }

    for(int i = 0; i < 5; i++){
        ASSERT_EQ(m[2][i], (i + 10) * 2);
    }

}

TEST_F(MatFixture, TransposeMatrix){
    mat<3, 5> mat{
            {0, 1, 2, 3, 4},
            {5, 6, 7, 8, 9},
            {10, 11, 12, 13, 14}
    };

    auto m = mat.transpose();

    ASSERT_EQ(m[0][0], 0);
    ASSERT_EQ(m[0][1], 5);
    ASSERT_EQ(m[0][2], 10);

    ASSERT_EQ(m[1][0], 1);
    ASSERT_EQ(m[1][1], 6);
    ASSERT_EQ(m[1][2], 11);

    ASSERT_EQ(m[2][0], 2);
    ASSERT_EQ(m[2][1], 7);
    ASSERT_EQ(m[2][2], 12);

    ASSERT_EQ(m[3][0], 3);
    ASSERT_EQ(m[3][1], 8);
    ASSERT_EQ(m[3][2], 13);

    ASSERT_EQ(m[4][0], 4);
    ASSERT_EQ(m[4][1], 9);
    ASSERT_EQ(m[4][2], 14);
}

TEST_F(MatFixture, MatrixMatrixMultiply){
    matN<3> m1{
            {1, 2, 3},
            {2, 1, 3},
            {3, 2, 2}
    };

    matN<3> m2{
            {2, 1, 1},
            {2, 2, 1},
            {1, 1, 3}
    };

    auto m3 = m1 * m2;

    ASSERT_EQ(m3[0][0], 9);
    ASSERT_EQ(m3[0][1], 8);
    ASSERT_EQ(m3[0][2], 12);

    ASSERT_EQ(m3[1][0], 9);
    ASSERT_EQ(m3[1][1], 7);
    ASSERT_EQ(m3[1][2], 12);

    ASSERT_EQ(m3[2][0], 12);
    ASSERT_EQ(m3[2][1], 9);
    ASSERT_EQ(m3[2][2], 11);

}

TEST_F(MatFixture, ClearMaxtrixToZeros){
    matN<3> mat{
            {1, 2, 3},
            {2, 1, 3},
            {3, 2, 2}
    };

    mat.clear();

    for(int m = 0; m < 3; m++){
        for(int n = 0; n < 3; n++){
            ASSERT_EQ(mat[m][n], 0);
        }
    }
}

TEST_F(MatFixture, IntializeMatrixWithSingleScalarValue){
    matN<10> mat(1);

    for(int m = 0; m < 10; m++){
        for(int n = 0; n < 10; n++){
            if(n == m){
                ASSERT_EQ(mat[m][n], 1);
                continue;
            }
            ASSERT_EQ(mat[m][n], 0);
        }
    }
}

TEST_F(MatFixture, IntiailzeMatrixWithListOfMatrices){
    constexpr int N = 12;
    constexpr int NN = 3;
    matN<N> m{
        matN<NN>(1),
        matN<NN>(2),
        matN<NN>(3),
        matN<NN>(4),
    };

    for(int i = 0; i < N/NN; i++){
        for(int j = 0; j < NN; j++){
            for(int k = 0; k < N; k++){
                int row = i * NN + j;
                if(row == k){
                    ASSERT_EQ(m[row][k], i+1);
                }else{
                    ASSERT_EQ(m[row][k], 0);
                }
            }
        }
    }
}

TEST_F(MatFixture, IntializeMatrixWithGlmMatrix){
    glm::mat3x4 gm{
            {1, 2, 3, 1},
            {3, 1, 1, 2},
            {2, 2, 1, 3}
    };

    mat<4, 3> m(gm);

    ASSERT_EQ(m[0][0], 1);
    ASSERT_EQ(m[0][1], 3);
    ASSERT_EQ(m[0][2], 2);

    ASSERT_EQ(m[1][0], 2);
    ASSERT_EQ(m[1][1], 1);
    ASSERT_EQ(m[1][2], 2);

    ASSERT_EQ(m[2][0], 3);
    ASSERT_EQ(m[2][1], 1);
    ASSERT_EQ(m[2][2], 1);

    ASSERT_EQ(m[3][0], 1);
    ASSERT_EQ(m[3][1], 2);
    ASSERT_EQ(m[3][2], 3);
}

TEST_F(MatFixture, AssignMatrixRowsToGlmVectors){
    glm::vec3 v0{0, 1, 2};
    glm::vec3 v1{3, 4, 5};
    glm::vec3 v2{6, 7, 8};
    glm::vec3 v3{9, 10, 11};

    mat12 m;
    m[0] = vec12{v0, v1, v2, v3};

    for(auto i = 0; i < 12; i++){
        ASSERT_EQ(m[0][i], i);
    }

}

TEST_F(MatFixture, NonSquareMatrixMultiply){
    mat<1, 3> m1{{1, 2, 3}};
    mat3 m2{
        {1, 2, 3},
        {2, 1, 3},
        {3, 2, 2}
    };

    mat<1, 3> m3 = m1 * m2;

    ASSERT_EQ(m3[0][0], 14);
    ASSERT_EQ(m3[0][1], 10);
    ASSERT_EQ(m3[0][2], 15);
}