#include "constrainthingequatlimited.hpp"

void ConstraintHingeQuatLimited::preSolve(float dt) {
    const auto worldAnchorA = m_bodyA->bodySpaceToWorldSpace(m_anchorA);
    const auto worldAnchorB = m_bodyB->bodySpaceToWorldSpace(m_anchorB);

    const auto r = worldAnchorB - worldAnchorA;
    const auto ra = worldAnchorA - m_bodyA->centerOfMassWorldSpace();
    const auto rb = worldAnchorB - m_bodyB->centerOfMassWorldSpace();
    const auto a = worldAnchorA;
    const auto b = worldAnchorB;

    const auto q1 = m_bodyA->orientation;
    const auto q2 = m_bodyB->orientation;
    const auto q0_inv = glm::inverse(m_q0);
    const auto q1_inv = glm::inverse(q1);

    glm::vec3 u, v;
    auto hingeAxis = m_axisA;
    orthonormal(hingeAxis, u, v);


    glm::mat4 P{
            {0, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    };
    auto P_T = glm::transpose(P);

    const auto MatA = P * qLeft(q1_inv) * qRight(q2 * q0_inv) * P_T * -0.5f;
    const auto MatB = P * qLeft(q1_inv) * qRight(q2 * q0_inv) * P_T * 0.5f;

    const auto qr = q1_inv * q2;
    const auto qrr = qr * q0_inv;
    auto relativeAngle = 2.0f * glm::asin(glm::dot(glm::axis(qrr), hingeAxis));
    relativeAngle = glm::degrees(relativeAngle);

    m_isAngleViolated = relativeAngle > 45 || relativeAngle < -45;
    m_relativeAngle = relativeAngle;


    // The distance constraint
    m_Jacobian.clear();

    // First row is the primary distance constraint that holds the anchor points together
    glm::vec3 J1 = ( a - b) * 2.0f;
    m_Jacobian.set(0, 0, J1);

    auto J2 = glm::cross(ra, J1);
    m_Jacobian.set(0, 3, J2);

    auto J3 = (b - a ) * 2.0f;
    m_Jacobian.set(0, 6, J3);

    auto J4 = glm::cross(rb, J3);
    m_Jacobian.set(0, 9, J4);

    // The quaternion jacobians
    const auto idx = 1;
    glm::vec4 tmp;
    {
        J1 = glm::vec3(0);
        m_Jacobian.set(1, 0, J1);

        tmp = MatA * glm::vec4(0, u.x, u.y, u.z);
        J2 = { tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(1, 3, J2);

        J3 = glm::vec3(0);
        m_Jacobian.set(1, 6, J3);

        tmp = MatB * glm::vec4(0, u.x, u.y, u.z);
        J4 = {tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(1, 9, J4);
    }
    {
        J1 = glm::vec3(0);
        m_Jacobian.set(2, 0, J1);

        tmp = MatA * glm::vec4(0, v.x, v.y, v.z);
        J2 = { tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(2, 3, J2);

        J3 = glm::vec3(0);
        m_Jacobian.set(2, 6, J3);

        tmp = MatB * glm::vec4(0, v.x, v.y, v.z);
        J4 = {tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(2, 9, J4);
    }
    if(m_isAngleViolated){
        J1 = glm::vec3(0);
        m_Jacobian.set(3, 0, J1);

        tmp = MatA * glm::vec4(0, hingeAxis);
        J2 = {tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(3, 3, J2);

        J3 = glm::vec3(0);
        m_Jacobian.set(3, 6, J3);

        tmp = MatB * glm::vec4(0, hingeAxis);
        J4 = {tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(3, 9, J4);
    }

    const auto impulses = m_Jacobian.transpose() * m_cachedLambda;
    applyImpulses(impulses);

    // Calculate the baumgarte stabilization;
    const auto Beta = 0.05f;
    auto C = glm::dot(r, r);
    C = glm::max(0.0f, C - 0.01f);
    m_baumgarte = (Beta / dt) * C;
}

void ConstraintHingeQuatLimited::solve() {
    const auto jacobianTranspose = m_Jacobian.transpose();

    // Build the system of equations
    const auto q_dt = velocities();
    const auto invMassMatrix = inverseMassMatrix();
    const auto J_W_Jt = m_Jacobian * invMassMatrix * jacobianTranspose;

    auto rhs = m_Jacobian * -q_dt;
    rhs[0] -= m_baumgarte;

    // Solve for the Lagrange multipliers;
    auto lambdaN = lcp::gaussSeidel(J_W_Jt, rhs);

    // Clamp the torque from the angle constraint.
    // We need to make sure it's a restorative torque.
    if(m_isAngleViolated){
        if(m_relativeAngle > 0.0f){
            lambdaN[3] = glm::min(0.0f, lambdaN[3]);
        }
        if(m_relativeAngle < 0.0f){
            lambdaN[3] = glm::max(0.0f, lambdaN[3]);
        }
    }

    const auto impulses = jacobianTranspose * lambdaN;

    applyImpulses(impulses);
}

void ConstraintHingeQuatLimited::postSolve() {
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
