#pragma once

#include "constraintbase.hpp"

class ConstraintConstantVelocity : public ConstraintBase{
public:

    void preSolve(float dt) override;

    void solve() override;

    void postSolve() override;

    glm::quat m_q0{1, 0, 0, 0}; // The initial relative quaternion q1^-1 * q2
    mat2x12 m_Jacobian;
    vec2 m_cachedLambda{0};
    float m_baumgarte{0};
};