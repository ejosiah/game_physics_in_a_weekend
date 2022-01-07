#include "constraintPenetration.hpp"

void ConstraintPenetration::preSolve(const float dt) {
    const auto worldAnchorA = m_bodyA->bodySpaceToWorldSpace(m_anchorA);
    const auto worldAnchorB = m_bodyB->bodySpaceToWorldSpace(m_anchorB);

    const auto ra = worldAnchorA - m_bodyA->centerOfMassWorldSpace();
    const auto rb = worldAnchorB - m_bodyB->centerOfMassWorldSpace();
    const auto a = worldAnchorA;
    const auto b = worldAnchorB;

    m_friction = m_bodyA->friction * m_bodyB->friction;

    glm::vec3 u, v;
    orthonormal(m_normal, u, v);

    // Convert tangent space from model space to world space;
    auto normal = glm::mat3(m_bodyA->orientation) * m_normal;
    u = glm::mat3(m_bodyA->orientation) * u;
    v = glm::mat3(m_bodyA->orientation) * v;

    m_Jacobian.clear();

    auto J1 = -normal;
    m_Jacobian.set(0, 0, J1);

    auto J2 = glm::cross(ra, -normal);
    m_Jacobian.set(0, 3, J2);

    auto J3 = normal;
    m_Jacobian.set(0, 6, J3);

    auto J4 = glm::cross(rb, normal);
    m_Jacobian.set(0, 9, J4);

    if(m_friction > 0.0f){
        J1 = -u;
        m_Jacobian.set(1, 0, J1);

        J2 = glm::cross(ra, -u);
        m_Jacobian.set(1, 3, J2);

        J3 = u;
        m_Jacobian.set(1, 6, J3);

        J4 = glm::cross(rb, u);
        m_Jacobian.set(1, 9, J4);

        J1 = -v;
        m_Jacobian.set(2, 0, J1);

        J2 = glm::cross(ra, -v);
        m_Jacobian.set(2, 3, J2);

        J3 = v;
        m_Jacobian.set(2, 6, J3);

        J4 = glm::cross(rb, v);
        m_Jacobian.set(2, 9, J4);
    }

    // apply warm starting from last frame;
    const auto impulses = m_Jacobian.transpose() * m_cachedLambda;
    applyImpulses(impulses);

    float C = glm::dot(b - a, normal);
    C = glm::min(0.0f, C + 0.02f);
    float Beta = 0.25f;
    m_baumgarte = Beta * C / dt;
}

void ConstraintPenetration::solve() {
    const auto jacobianTranspose = m_Jacobian.transpose();

    // build the system of equations
    const auto q_dt = velocities();
    const auto invMassMatrix = inverseMassMatrix();
    const auto J_W_Jt = m_Jacobian * invMassMatrix * jacobianTranspose;
    auto rhs = m_Jacobian * -q_dt;
    rhs[0] -= m_baumgarte;

    auto lambdaN = lcp::gaussSeidel(J_W_Jt, rhs);

    // Accumulate the impulses and clamp to within the constraint limits
    auto oldLambda = m_cachedLambda;
    m_cachedLambda += lambdaN;
    const auto lambdaLimit = 0.0f;
    if(m_cachedLambda[0] < lambdaLimit){
        m_cachedLambda[0] = lambdaLimit;
    }

    if(m_friction > 0.0f){
        const auto umg = m_friction * 10.0f * 1.0f / (m_bodyA->invMass + m_bodyB->invMass);
        const auto normalForce = glm::abs(lambdaN[0] * m_friction);
        const auto maxForce = (umg > normalForce) ? umg : normalForce;

        if(m_cachedLambda[1] > maxForce){
            m_cachedLambda[1] = maxForce;
        }
        if(m_cachedLambda[1] < -maxForce){
            m_cachedLambda[1] = -maxForce;
        }

        if(m_cachedLambda[2] > maxForce){
            m_cachedLambda[2] = maxForce;
        }
        if(m_cachedLambda[2] < -maxForce){
            m_cachedLambda[2] = -maxForce;
        }
    }
    lambdaN = m_cachedLambda - oldLambda;
    const auto impulses = jacobianTranspose * lambdaN;
    applyImpulses(impulses);
}
