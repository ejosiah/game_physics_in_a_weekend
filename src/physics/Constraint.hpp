#pragma once

#include <glm/glm.hpp>
#include "vecN.hpp"
#include "mat.hpp"
#include "body.hpp"

class Constraint{
public:
    virtual void preSolve( const float dt) {}
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

class ConstraintDistance : public Constraint {
public:
    void preSolve(float dt) override;

    void solve() override;

    void postSolve() override;

private:
    mat1x12 m_Jacobian;
    vec1 m_cachedLambda{1};
};