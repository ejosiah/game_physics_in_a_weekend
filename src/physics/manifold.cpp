#include "manifold.hpp"
#include "utility.hpp"
#include <glm/gtx/quaternion.hpp>

void Manifold::addContact(const Contact &contact) {
    auto newContact = contact;
    if(contact.bodyA != m_bodyA || contact.bodyB != m_bodyB){
        newContact.LocalSpace.pointOnA = contact.LocalSpace.pointOnB;
        newContact.LocalSpace.pointOnB = contact.LocalSpace.pointOnA;
        newContact.worldSpace.pointOnA = contact.worldSpace.pointOnB;
        newContact.worldSpace.pointOnB = contact.worldSpace.pointOnA;

        newContact.bodyA = m_bodyA;
        newContact.bodyB = m_bodyB;
    }

    // if the contact is close to one another then keep the old contact
    for(auto i = 0; i < m_numContacts; i++){
        const auto [bodyA, bodyB] = m_contacts[i].bodies();

        const auto oldA = bodyA->bodySpaceToWorldSpace(m_contacts[i].LocalSpace.pointOnA);
        const auto oldB = bodyB->bodySpaceToWorldSpace(m_contacts[i].LocalSpace.pointOnB);

        const auto newA = bodyA->bodySpaceToWorldSpace(newContact.LocalSpace.pointOnA);
        const auto newB = bodyB->bodySpaceToWorldSpace(newContact.LocalSpace.pointOnB);

        const auto aa = newA - oldA;
        const auto bb = newB - oldB;

        constexpr auto distanceThreshold = 0.02f;
        if(glm::dot(aa, aa) < distanceThreshold * distanceThreshold){
            return;
        }
        if(glm::dot(bb, bb) < distanceThreshold * distanceThreshold){
            return;
        }
    }

    // if we're all full on contacts, then keep the contacts that are furthest away from each other
    auto newSlot = m_numContacts;
    if(newSlot >= MAX_CONTACTS){
        glm::vec3 avg{0};

        avg += m_contacts[0].LocalSpace.pointOnA;
        avg += m_contacts[1].LocalSpace.pointOnA;
        avg += m_contacts[2].LocalSpace.pointOnA;
        avg += m_contacts[3].LocalSpace.pointOnA;
        avg += newContact.LocalSpace.pointOnA;
        avg *= 0.2f;

        auto diff = (avg - newContact.LocalSpace.pointOnA);
        auto minDist = glm::dot(diff, diff);
        auto newIdx = -1;
        for(auto i = 0; i < MAX_CONTACTS; i++){
            diff = (avg - m_contacts[i].LocalSpace.pointOnA);
            auto dist2  = glm::dot(diff, diff);

            if(dist2 < minDist){
                minDist = dist2;
                newIdx = i;
            }
        }

        if( newIdx != -1){
            newSlot = newIdx;
        }else{
            return;
        }
    }

    m_contacts[newSlot] = newContact;

    m_constraints[newSlot].m_bodyA = newContact.bodyA;
    m_constraints[newSlot].m_bodyB = newContact.bodyB;
    m_constraints[newSlot].m_anchorA = newContact.LocalSpace.pointOnA;
    m_constraints[newSlot].m_anchorB = newContact.LocalSpace.pointOnB;

    // Get the normal in BodyA's space
    auto normal = glm::rotate(m_bodyA->orientation, -newContact.normal);
    m_constraints[newSlot].m_normal = glm::normalize(normal);
    m_constraints[newSlot].m_cachedLambda.clear();

    if(newSlot == m_numContacts){
        m_numContacts++;
    }

}

void Manifold::removeExpiredContacts() {
    for(auto i = 0; i < m_numContacts; i++){
        auto& contact = m_contacts[i];

        auto [bodyA, bodyB] = contact.bodies();

        auto normal = glm::rotate(bodyA->orientation, m_constraints[i].m_normal);

        // Get the tangential distance of the point on A and the point on B
        const auto a = bodyA->bodySpaceToWorldSpace(contact.LocalSpace.pointOnA);
        const auto b = bodyB->bodySpaceToWorldSpace(contact.LocalSpace.pointOnB);

        // Calculate the tangential separation and penetration depth
        const auto ab = b - a;
        auto penetrationDepth = glm::dot(normal, ab);
        auto abNormal = normal * penetrationDepth;
        auto abTangent = ab - abNormal;

        // if the tangential displacement is less than a specific threshold, it's okay to keep it
        constexpr auto distanceThreshold = 0.02f;
        if(glm::dot(abTangent, abTangent) < distanceThreshold * distanceThreshold && penetrationDepth <= 0.0f){
            continue;
        }

        // This contact has moved beyond its threshold and should be removed;
        for(auto j = i; j < MAX_CONTACTS - 1; j++){
            m_constraints[j] = m_constraints[j + 1];
            if(j > m_numContacts){
                m_constraints[j].m_cachedLambda.clear();
            }
        }
        m_numContacts--;
        i--;
    }
}

void Manifold::preSolve(const float dt) {
    for(auto i = 0; i < m_numContacts; i++){
        m_constraints[i].preSolve(dt);
    }
}

void Manifold::solve() {
    for(auto i = 0; i < m_numContacts; i++){
        m_constraints[i].solve();
    }
}

void Manifold::postSolve() {
    for(auto i = 0; i < m_numContacts; i++){
        m_constraints[i].postSolve();
    }
}

const Contact &Manifold::operator[](const int idx) const {
    assert(idx < m_numContacts);
    return m_contacts[idx];
}

Contact &Manifold::operator[](const int idx) {
    assert(idx < m_numContacts);
    return m_contacts[idx];
}


void ManifoldCollector::addContact(const Contact &contact) {
    int foundIdx = -1;
    for(int i = 0; i < m_manifolds.size(); i++){
        const auto& manifold = m_manifolds[i];
        bool hasA = (manifold.m_bodyA == contact.bodyA || manifold.m_bodyB == contact.bodyA);
        bool hasB = (manifold.m_bodyA == contact.bodyB || manifold.m_bodyB == contact.bodyB);
        if(hasA && hasB){
            foundIdx = i;
            break;
        }
    }

    if(foundIdx >= 0){
        m_manifolds[foundIdx].addContact(contact);
    }else{
        Manifold manifold;
        manifold.m_bodyA = contact.bodyA;
        manifold.m_bodyB = contact.bodyB;

        manifold.addContact(contact);
        m_manifolds.push_back(manifold);
    }
}

void ManifoldCollector::removeExpired() {
    for(auto i = static_cast<int>(m_manifolds.size() - 1); i >= 0; i--){
        auto& manifold = m_manifolds[i];
        manifold.removeExpiredContacts();

        if(manifold.m_numContacts == 0){
            m_manifolds.erase(m_manifolds.begin() + i);
        }
    }
}

void ManifoldCollector::preSolve(const float dt) {
    for(auto& manifold : m_manifolds){
        manifold.preSolve(dt);
    }
}

void ManifoldCollector::solve() {
    for(auto& manifold : m_manifolds){
        manifold.solve();
    }
}

void ManifoldCollector::postSolve() {{
    for(auto& manifold : m_manifolds){
        manifold.postSolve();
    }
}}