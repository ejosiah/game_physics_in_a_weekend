#pragma once

#include "constraintbase.hpp"

class ConstraintMotor : public ConstraintBase{
public:

    void preSolve(float dt) override;

    void solve() override;

    float m_motorSpeed{0};
    glm::vec3 m_motorAxis{0, 1, 0};
    glm::quat m_q0{1, 0, 0, 0}; // The initial relative quaternion q1^-1 * q2
    mat4x12 m_Jacobian;
    glm::vec3 m_baumgarte{0};
};