#include "shape.hpp"

Bounds SphereShape::bounds(const glm::vec3 &pos, const glm::quat &orient) const {
    Bounds bounds;
    bounds.expand(glm::vec3(-m_radius) + pos);
    bounds.expand(glm::vec3(m_radius) + pos);
    return bounds;
}

Bounds SphereShape::bounds() const {
    Bounds bounds;
    bounds.expand(glm::vec3(-m_radius));
    bounds.expand(glm::vec3(m_radius) );
    return bounds;
}

glm::vec3
SphereShape::support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient, float bias) const {
    return pos + dir * (m_radius + bias);
}


glm::mat3 BoxShape::inertiaTensor() const {
    const auto d = m_bounds.max - m_bounds.min;

    glm::mat3 tensor{0};
    tensor[0][0] = (d.y * d.y + d.z * d.z)/12.0f;
    tensor[1][1] = (d.x * d.x + d.z * d.z)/12.0f;
    tensor[2][2] = (d.x * d.x + d.y * d.y)/12.0f;

    // Now we need to use the parallel axis theorem to get the inertial tensor for a box
    // that is not centered around the origin
    auto com = (m_bounds.max + m_bounds.min) * 0.5f;
    const auto R = glm::vec3(0) - com;
    const auto R2 = glm::dot(R, R);

    glm::mat3 patTensor{
            {R2 - R.x * R.x,    R.y * R.x,          R.z * R.x},
            {R.x * R.y,         R2 - R.y * R.y,     R.z * R.y},
            {R.x * R.z,         R.y * R.z,          R2 - R.z * R.z}
    };
    tensor += patTensor;

    return tensor;
}

glm::vec3 BoxShape::centerOfMass() const {
    return m_centerOfMass;
}

Bounds BoxShape::bounds(const glm::vec3 &pos, const glm::quat &orient) const {
    std::array<glm::vec3, 8> corners{};
    const auto min = m_bounds.min;
    const auto max = m_bounds.max;
    corners[0] = min;
    corners[1] = glm::vec3(min.xy, max.z);
    corners[2] = glm::vec3(min.x, max.y, min.z);
    corners[3] = glm::vec3(max.x, min.yz);

    corners[4] = max;
    corners[5] = glm::vec3(max.xy, min.z);
    corners[6] = glm::vec3(max.x, min.y, max.z);
    corners[7] = glm::vec3(min.x, max.yz);

    Bounds cBounds{};
    auto rotate = glm::mat3(orient);
    for(auto& corner : corners){
        corner = rotate * corner;
        cBounds.expand(corner);
    }

    return cBounds;
}

Bounds BoxShape::bounds() const {
    return m_bounds;
}

void BoxShape::build(const std::vector<glm::vec3> &points) {
    for(const auto& point : points){
        m_bounds.expand(point);
    }

    const auto min = m_bounds.min;
    const auto max = m_bounds.max;
    m_points.clear();
    m_points.emplace_back(min.x, min.y, min.z);
    m_points.emplace_back(max.x, min.y, min.z);
    m_points.emplace_back(min.x, max.y, min.z);
    m_points.emplace_back(min.x, min.y, max.z);

    m_points.emplace_back(max.x, max.y, max.z);
    m_points.emplace_back(min.x, max.y, max.z);
    m_points.emplace_back(max.x, min.y, max.z);
    m_points.emplace_back(max.x, max.y, min.z);

    m_centerOfMass = (max + min) * 0.5f;
}

glm::vec3 BoxShape::support(const glm::vec3 &dir, const glm::vec3 &pos, const glm::quat &orient, float bias) const {
    auto rotation = glm::mat3(orient);
    auto maxPoint = rotation * m_points[0] + pos;
    auto maxDist = glm::dot(dir, maxPoint);
    for(auto i = 1; i < m_points.size(); i++){
        const auto pt = rotation * m_points[i] + pos;
        const auto dist = glm::dot(dir, pt);
        if(dist > maxDist){
            maxDist = dist;
            maxPoint = pt;
        }
    }
    auto n = glm::normalize(dir);
    n *= bias;
    return maxPoint + n;
}

float BoxShape::fastLinearSpeed(const glm::vec3 &angularVelocity, const glm::vec3 &dir) const {
    auto maxSpeed = std::numeric_limits<float>::min();
    for(const auto& point : m_points){
        auto r = point - m_centerOfMass;
        auto linearVelocity = glm::cross(angularVelocity, r);
        auto speed = glm::dot(dir, linearVelocity);
        maxSpeed = glm::max(speed, maxSpeed);
    }
    return maxSpeed;
}
