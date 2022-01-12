
#include "constraintmover.hpp"

void ConstraintMover::preSolve(float dt) {
    m_time += dt;
    m_bodyA->linearVelocity.z = glm::cos(m_time * 0.25f) * 4.0f;
}
