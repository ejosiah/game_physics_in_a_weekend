#pragma once

#include "constraintbase.hpp"

class ConstraintOrientation : public ConstraintBase{
public:

    void preSolve(float dt) override;

    void solve() override;

    glm::quat m_q0{1, 0, 0, 0}; // The initial relative quaternion q1^-1 * q2
    mat4x12 m_Jacobian;
    float m_baumgarte{0};
};