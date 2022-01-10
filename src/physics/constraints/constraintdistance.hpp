#pragma once

#include "constraintbase.hpp"

class ConstraintDistance : public ConstraintBase {
public:
    void preSolve(float dt) override;

    void solve() override;

    void postSolve() override;

private:
    mat1x12 m_Jacobian;
    vec1 m_cachedLambda{0};
    float m_baumgarte{0};
};