#pragma once

#include <memory>
#include <algorithm>
#include "shape.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct Body{
    glm::vec3 position{0};
    glm::quat orientation{1, 0, 0, 0};
    glm::vec3 linearVelocity{0};
    glm::vec3 angularVelocity{0};
    float invMass{0};
    float elasticity{1};
    float friction{0.5};
    std::shared_ptr<Shape> shape;

    [[nodiscard]]
    glm::vec3 centerOfMassWorldSpace() const;

    [[nodiscard]]
    glm::vec3 centerOfMassModelSpace() const;

    [[nodiscard]]
    glm::vec3 worldSpaceToBodySpace(const glm::vec3& pt) const;

    [[nodiscard]]
    glm::vec3 bodySpaceToWorldSpace(const glm::vec3& pt) const;

    [[nodiscard]]
    glm::mat3 inverseInertialTensorBodySpace() const;

    [[nodiscard]]
    glm::mat3 inverseInertialTensorWorldSpace() const;

    void applyImpulseLinear( const glm::vec3& impulse);

    void applyImpulseAngular(const glm::vec3& impulse);

    void applyImpulse(const glm::vec3& impulsePoint, const glm::vec3& impulse);

    void update(float dt);

    [[nodiscard]]
    bool hasInfiniteMass() const;
};

struct PsuedoBody{
    int id;
    float value;
    bool isMin;
};

struct CollisionPair{
    int a, b;

    bool operator==(const CollisionPair& rhs) const {
        return (a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a);
    }

    bool operator !=(const CollisionPair& rhs) const {
        return !( *this == rhs);
    }
};


inline void sort(const std::vector<Body*>& bodies, std::vector<PsuedoBody>& psuedoBodies, float dt){

    glm::vec3 axis{glm::normalize(glm::vec3(1))};

    auto numBodies = bodies.size();
    psuedoBodies.clear();
    psuedoBodies.reserve(numBodies * 2);
    for(auto i = 0; i < numBodies; i++){
        auto body = bodies[i];
        auto bounds = body->shape->bounds(body->position, body->orientation);

        // expand the bounds by the linear velocity;
        bounds.expand(bounds.min + body->linearVelocity * dt);
        bounds.expand(bounds.max + body->linearVelocity * dt);

        static const float epsilon = 0.01f;
        bounds.expand(bounds.min + glm::vec3(-1) * epsilon);
        bounds.expand(bounds.max + glm::vec3(1) * epsilon);

        PsuedoBody psuedoBody{ i, glm::dot(axis, bounds.max), true};
        psuedoBodies.push_back(psuedoBody);

        psuedoBody.value = glm::dot(axis, bounds.max);
        psuedoBody.isMin = false;
        psuedoBodies.push_back(psuedoBody);
    }

    std::sort(begin(psuedoBodies), end(psuedoBodies), [](const auto& a, const auto& b){ return a.value < b.value; });

}

inline void buildPairs(std::vector<CollisionPair>& collisionPair, const std::vector<PsuedoBody>& psuedoBodies){
    collisionPair.clear();
    collisionPair.reserve(psuedoBodies.size());

    auto numBodies = psuedoBodies.size();
    for(auto i = 0; i < numBodies; i++){
        const auto& a = psuedoBodies[i];
        if(!a.isMin){
            continue;
        }

        for(auto j = i + 1; j < numBodies; j++){
            const auto& b = psuedoBodies[j];

            if(b.id == a.id){
                break;  // end of element, done creating pairs
            }

            if(!b.isMin){
                continue;
            }

            collisionPair.push_back({a.id, b.id});
        }
    }
}

inline void sweepAndPrune1D(const std::vector<Body*>& bodies, std::vector<CollisionPair>& finalPairs, const float dt){
    std::vector<PsuedoBody> psuedoBodies;
    sort(bodies, psuedoBodies, dt);
    buildPairs(finalPairs, psuedoBodies);
}

inline void broadPhase(const std::vector<Body*>& bodies, std::vector<CollisionPair> finalPairs, const float dt){
    finalPairs.clear();
    sweepAndPrune1D(bodies, finalPairs, dt);
}