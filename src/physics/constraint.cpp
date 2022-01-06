#include "Constraint.hpp"
#include "lcp.hpp"
#include <spdlog/spdlog.h>
#include <sstream>
mat12 Constraint::inverseMassMatrix() const {
    mat3 invMassA(m_bodyA->invMass);
    mat3 invInertiaA(m_bodyA->inverseInertialTensorWorldSpace());
    mat3 invMassB(m_bodyB->invMass);
    mat3 invInertiaB(m_bodyB->inverseInertialTensorWorldSpace());

    mat12 result{invMassA, invInertiaA, invMassB, invInertiaB};

    return result;
}

vec12 Constraint::velocities() const {
    vec12 q_dt{
        m_bodyA->linearVelocity,
        m_bodyA->angularVelocity,
        m_bodyB->linearVelocity,
        m_bodyB->angularVelocity
    };

    return q_dt;
}

void Constraint::applyImpulses(const vec12 &impulses) {
    auto vImpulses = impulses.split<3>();

    glm::vec3 forceInternalA = vImpulses[0];
    glm::vec3 torqueInternalA = vImpulses[1];
    glm::vec3 forceInternalB = vImpulses[2];
    glm::vec3 torqueInternalB = vImpulses[3];

    m_bodyA->applyImpulseLinear(forceInternalA);
    m_bodyA->applyImpulseAngular(torqueInternalA);

    m_bodyB->applyImpulseLinear(forceInternalB);
    m_bodyB->applyImpulseAngular(torqueInternalB);

}

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

    auto J1 = ( a - b) * 2.0f;
    m_Jacobian[0][0] = J1.x;
    m_Jacobian[0][1] = J1.y;
    m_Jacobian[0][2] = J1.z;

    auto J2 = glm::cross(ra, J1);
    m_Jacobian[0][3] = J2.x;
    m_Jacobian[0][4] = J2.y;
    m_Jacobian[0][5] = J2.z;

    auto J3 = (b - a ) * 2.0f;
    m_Jacobian[0][6] = J3.x;
    m_Jacobian[0][7] = J3.y;
    m_Jacobian[0][8] = J3.z;

    auto J4 = glm::cross(rb, J3);
    m_Jacobian[0][9] =  J4.x;
    m_Jacobian[0][10] = J4.y;
    m_Jacobian[0][11] = J4.z;

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
