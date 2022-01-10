#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "vecN.hpp"
#include "mat.hpp"
#include "body.hpp"
#include "lcp.hpp"

class ConstraintBase{
public:
    virtual void preSolve(float dt) {}
    virtual void solve() {}
    virtual void postSolve(){}

protected:

    [[nodiscard]]
    mat12 inverseMassMatrix() const;

    [[nodiscard]]
    vec12 velocities() const;

    void applyImpulses(const vec12& impulses);

public:
    Body* m_bodyA;
    Body* m_bodyB;

    glm::vec3 m_anchorA;
    glm::vec3 m_axisA;

    glm::vec3 m_anchorB;
    glm::vec3 m_axisB;
};