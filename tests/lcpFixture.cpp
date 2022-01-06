#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "lcp.hpp"

class LCPFixture : public ::testing::Test {
protected:
};

TEST_F(LCPFixture, SolveUsingGaussSidelMethod){
    matN<2> A{
        {3, 1},
        {2, 4}
    };

    vec<2> b{6.0f, 7.0f};

    auto x = lcp::gaussSeidel(A, b);

    ASSERT_NEAR(x[0], 1.70, 1E-2);
    ASSERT_NEAR(x[1], 0.90, 1E-2);

}