#pragma once

#include "constraintbase.hpp"

class ConstraintConstantVelocityLimited : public ConstraintBase{
public:
    void preSolve(float dt) override;

    void solve() override;

    void postSolve() override;

    glm::quat m_q0{1, 0, 0, 0}; // The initial relative quaternion q1^-1 * q2
    mat4x12 m_Jacobian;
    vec4 m_cachedLambda{0};
    float m_baumgarte{0};
    bool m_isAngleViolatedU{false};
    bool m_isAngleViolatedV{false};
    float m_angleU{0};
    float m_angleV{0};
};