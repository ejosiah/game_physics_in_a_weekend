#pragma once

#include "constraintbase.hpp"

class ConstraintPenetration : public ConstraintBase{
public:

    void preSolve(float dt) override;

    void solve() override;

    vec3 m_cachedLambda{0};
    glm::vec3 m_normal;

    mat3x12 m_Jacobian;

    float m_baumgarte{0};
    float m_friction{0};
};