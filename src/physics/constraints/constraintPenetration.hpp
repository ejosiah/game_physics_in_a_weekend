#pragma once

#include "constraintbase.hpp"

class ConstraintPenetration : public ConstraintBase{
public:

    void preSolve(const float dt) override;

    void solve() override;

    vec3 m_cachedLambda{3};
    glm::vec3 m_normal;

    mat3x12 m_Jacobian;

    float m_baumgarte{0};
    float m_friction{0};
};