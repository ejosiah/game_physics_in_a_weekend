#pragma once

#include "body.hpp"
#include "contact.hpp"
#include "constraintPenetration.hpp"
#include <array>
#include <vector>

class Manifold{
public:

    void addContact(const Contact& contact);

    void removeExpiredContacts();

    void removeContact(int idx);

    void removeContactFor(const Body* body);

    void preSolve(float dt);

    void solve();

    void postSolve();

    const Contact& operator[](int idx) const;

    Contact& operator[](int idx);

    [[nodiscard]]
    inline int numContacts() const {
        return m_numContacts;
    }

private:
    static constexpr int MAX_CONTACTS = 4;
    std::array<Contact, MAX_CONTACTS> m_contacts{};
    int m_numContacts{0};

    Body* m_bodyA{nullptr};
    Body* m_bodyB{nullptr};

    std::array<ConstraintPenetration, MAX_CONTACTS> m_constraints{};

    friend class ManifoldCollector;
};

class ManifoldCollector{
public:

    void addContact(const Contact& contact);

    void preSolve(float dt);

    void solve();

    void postSolve();

    void removeExpired();

    void removeContactsFor(const Body* body);

    inline void clear() {

    }

public:
    std::vector<Manifold> m_manifolds;
};