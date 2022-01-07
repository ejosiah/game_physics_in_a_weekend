#include "constraintdistance.hpp"
#include "lcp.hpp"
#include <spdlog/spdlog.h>
#include <sstream>

void ConstraintDistance::preSolve(const float dt) {
    // Get the world space position of the hinge from A's orientation
    const auto worldAnchorA = m_bodyA->bodySpaceToWorldSpace(m_anchorA);

    // Get the world space position of the hinge from B's orientation
    const auto worldAnchorB = m_bodyB->bodySpaceToWorldSpace(m_anchorB);

    const auto r = worldAnchorB - worldAnchorA;
    const auto ra = worldAnchorA - m_bodyA->centerOfMassWorldSpace();
    const auto rb = worldAnchorB - m_bodyB ->centerOfMassWorldSpace();
    const auto a = worldAnchorA;
    const auto b = worldAnchorB;

    m_Jacobian.clear();

    glm::vec3 J1 = ( a - b) * 2.0f;
    m_Jacobian.set(0, 0, J1);

    auto J2 = glm::cross(ra, J1);
    m_Jacobian.set(0, 3, J2);

    auto J3 = (b - a ) * 2.0f;
    m_Jacobian.set(0, 6, J3);

    auto J4 = glm::cross(rb, J3);
    m_Jacobian.set(0, 9, J4);

    // apply warm starting from last frame
    const auto impulses = m_Jacobian.transpose() * m_cachedLambda;
    applyImpulses(impulses);

    // Calculate the baumgarte stabilization
    auto C = glm::dot(r, r);
    C = glm::max(0.0f, C - 0.01f);
    const auto Beta = 0.05f;
    m_baumgarte = (Beta / dt) * C;

}

void ConstraintDistance::solve() {
    const auto jacobianTranspose = m_Jacobian.transpose();

    // Build the system of equations
    const auto q_dt = velocities();
    const auto invMassMatrix = inverseMassMatrix();
    const auto J_W_Jt = m_Jacobian * invMassMatrix * jacobianTranspose;

    auto rhs = m_Jacobian * -q_dt;
    rhs[0] -= m_baumgarte;

    // Solve for the Lagrange multipliers;
    const auto lambdaN = lcp::gaussSeidel(J_W_Jt, rhs);
    const auto impulses = jacobianTranspose * lambdaN;

    applyImpulses(impulses);
    m_cachedLambda += lambdaN;
}

void ConstraintDistance::postSolve() {
    if( m_cachedLambda[0] * 0.0f != m_cachedLambda[0] * 0.0f){
        m_cachedLambda[0] = 0.0f;
    }

    static constexpr float limit = 1e5f;
    if(m_cachedLambda[0] > limit){
        m_cachedLambda[0] = limit;
    }
    if(m_cachedLambda[0] < -limit){
        m_cachedLambda[0] = -limit;
    }
}
