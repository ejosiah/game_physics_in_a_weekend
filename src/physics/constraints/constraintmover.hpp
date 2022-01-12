#pragma once

#include "constraintbase.hpp"

class ConstraintMover : public ConstraintBase{
public:
    void preSolve(float dt) override;

    float m_time{0};
};