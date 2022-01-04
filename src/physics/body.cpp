#include "body.hpp"
#include "utility.hpp"

glm::vec3 Body::centerOfMassWorldSpace() const {
    auto centerOfMass = shape->centerOfMass();
    auto wCenterOfMass = glm::mat4(orientation) * glm::vec4(centerOfMass, 1);
    auto pos = position + wCenterOfMass.xyz();
    return pos;
}

glm::vec3 Body::centerOfMassModelSpace() const {
    return shape->centerOfMass();
}

glm::vec3 Body::worldSpaceToBodySpace(const glm::vec3 &pt) const {
    auto temp = pt - centerOfMassModelSpace();
    auto inverseRotation = glm::inverse(glm::mat3(orientation));
    auto bodySpacePt = inverseRotation * temp;
    return bodySpacePt;
}

glm::vec3 Body::bodySpaceToWorldSpace(const glm::vec3& pt) const {
    auto rotation = glm::mat3(orientation);
    auto worldSpacePt = centerOfMassModelSpace() + rotation * pt;
    return worldSpacePt;
}

glm::mat3 Body::inverseInertialTensorBodySpace() const {
    auto tensor = shape->inertiaTensor();
    return glm::inverse(tensor) * invMass;
}

glm::mat3 Body::inverseInertialTensorWorldSpace() const {
    auto invTensor = inverseInertialTensorBodySpace();
    auto orient = glm::mat3(orientation);
    return orient * invTensor * glm::transpose(orient);
}

void Body::applyImpulseLinear(const glm::vec3 &impulse) {
    if(invMass == 0) return;

    // p = mv;  // momentum
    // dp = mdv = J  // impulse
    // => dv = j / m
    linearVelocity += impulse * invMass;
}

void Body::applyImpulseAngular(const glm::vec3 &impulse) {
    if(invMass == 0) return;

    // L = I w = r x p
    // dL = I dw = r x J
    // => dw = I^−1 * (r x J)
    angularVelocity += inverseInertialTensorWorldSpace() * impulse;

    const float maxAngularSpeed = 30.0f;
    if(dot(angularVelocity, angularVelocity) > maxAngularSpeed * maxAngularSpeed){
        angularVelocity = glm::normalize(angularVelocity);
        angularVelocity *= maxAngularSpeed;
    }
}

void Body::applyImpulse(const glm::vec3& impulsePoint, const glm::vec3 &impulse) {
    if(invMass == 0) return;

    applyImpulseLinear(impulse);

    auto com = centerOfMassWorldSpace();    // applying impulses must produce torques through the center of mass;
    auto r = impulsePoint - com;
    auto dL = glm::cross(r, impulse);
    applyImpulseAngular(dL);
}

void Body::update(float dt) {
    position += linearVelocity * dt;

    auto com = centerOfMassWorldSpace();
    auto comToPosition = position - com;

    auto orient = glm::mat3(orientation);
    auto tensor = orient * shape->inertiaTensor() * glm::transpose(orient);

    auto alpha = glm::inverse(tensor) * glm::cross(angularVelocity, tensor * angularVelocity);
    angularVelocity += alpha;

    auto dAngle = angularVelocity * dt;
    glm::quat dq = glm::angleAxis(glm::length(dAngle), glm::normalize(dAngle));
    dq = nansafe(dq);

    orientation = dq * orientation;
    orientation = glm::normalize(orientation);

    position = com + glm::mat3(orientation) * comToPosition;
}

bool Body::hasInfiniteMass() const {
    return invMass <= 0.0f;
}
