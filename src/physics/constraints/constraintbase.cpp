#include "constraintbase.hpp"

mat12 ConstraintBase::inverseMassMatrix() const {
    mat3 invMassA(m_bodyA->invMass);
    mat3 invInertiaA(m_bodyA->inverseInertialTensorWorldSpace());
    mat3 invMassB(m_bodyB->invMass);
    mat3 invInertiaB(m_bodyB->inverseInertialTensorWorldSpace());

    mat12 result{invMassA, invInertiaA, invMassB, invInertiaB};

    return result;
}

vec12 ConstraintBase::velocities() const {
    vec12 q_dt{
            m_bodyA->linearVelocity,
            m_bodyA->angularVelocity,
            m_bodyB->linearVelocity,
            m_bodyB->angularVelocity
    };

    return q_dt;
}

void ConstraintBase::applyImpulses(const vec12 &impulses) {
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