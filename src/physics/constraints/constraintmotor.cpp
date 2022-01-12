#include "constraintmotor.hpp"

void ConstraintMotor::preSolve(float dt) {
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

    auto motorAxis = glm::rotate(m_bodyA->orientation, m_motorAxis);
    glm::vec3 motorU, motorV;
    orthonormal(motorAxis, motorU, motorV);
    const auto u = motorU;
    const auto v = motorV;
    const auto w = motorAxis;

    glm::mat4 P{
            {0, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    };
    auto P_T = glm::transpose(P);

    const auto MatA = P * qLeft(q1_inv) * qRight(q2 * q0_inv) * P_T * -0.5f;
    const auto MatB = P * qLeft(q1_inv) * qRight(q2 * q0_inv) * P_T * 0.5f;

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
    {
        J1 = glm::vec3(0);
        m_Jacobian.set(3, 0, J1);

        tmp = MatA * glm::vec4(0, w.x, w.y, w.z);
        J2 = { tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(3, 3, J2);

        J3 = glm::vec3(0);
        m_Jacobian.set(3, 6, J3);

        tmp = MatB * glm::vec4(0, w.x, w.y, w.z);
        J4 = {tmp[idx + 0], tmp[idx + 1], tmp[idx + 2]};
        m_Jacobian.set(3, 9, J4);
    }

    // Calculate the baumgarte stabilization;
    const auto Beta = 0.05f;
    auto C = glm::dot(r, r);

    const auto qr = glm::inverse(m_bodyA->orientation) * m_bodyB->orientation;
    const auto qrA = qr * q0_inv;    // Relative orientation in BodyA's space
    const auto qrAAxis = glm::vec3(qrA.x, qrA.y, qrA.z);

    // Get the world space axis for the relative rotation
    const auto axisA = glm::rotate(m_bodyA->orientation, qrAAxis);

    m_baumgarte = glm::vec3(0);

    const auto Beta_over_dt = Beta/dt;
    m_baumgarte[0] = Beta_over_dt * C;
    m_baumgarte[1] = glm::dot(motorU, axisA) * Beta_over_dt;
    m_baumgarte[2] = glm::dot(motorV, axisA) * Beta_over_dt;

}

void ConstraintMotor::solve() {
    const auto motorAxis = glm::rotate(m_bodyA->orientation, m_motorAxis);

    vec12 w_dt(0);
    w_dt[3] = motorAxis[0] * -m_motorSpeed;
    w_dt[4] = motorAxis[1] * -m_motorSpeed;
    w_dt[5] = motorAxis[2] * -m_motorSpeed;

    w_dt[ 9] = motorAxis[0] * m_motorSpeed;
    w_dt[10] = motorAxis[1] * m_motorSpeed;
    w_dt[11] = motorAxis[2] * m_motorSpeed;

    const auto jacobianTranspose = m_Jacobian.transpose();

    // Build the system of equations
    const auto q_dt = velocities() - w_dt;
    const auto invMassMatrix = inverseMassMatrix();
    const auto J_W_Jt = m_Jacobian * invMassMatrix * jacobianTranspose;

    auto rhs = m_Jacobian * -q_dt;
    rhs[0] -= m_baumgarte[0];
    rhs[1] -= m_baumgarte[1];
    rhs[2] -= m_baumgarte[2];

    // Solve for the Lagrange multipliers;
    const auto lambdaN = lcp::gaussSeidel(J_W_Jt, rhs);
    const auto impulses = jacobianTranspose * lambdaN;

    applyImpulses(impulses);
}
