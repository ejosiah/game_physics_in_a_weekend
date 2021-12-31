#include "convexhullbuilder.hpp"
#include <limits>
#include <bounds.hpp>

ConvexHullBuilder::ConvexHullBuilder(const std::vector<glm::vec3> &points)
: m_points(points)
{

}


void ConvexHullBuilder::expandConvexHull(std::vector<glm::vec3> &hullPoints, std::vector<Tri> &hullTris,
                                         const std::vector<glm::vec3> &vertices) {
    auto externalVertices = vertices;
    removeInternalPoints(hullPoints, hullTris, externalVertices);

    while(!externalVertices.empty()){
        auto pointIdx = findPointFurthestInDir(externalVertices, externalVertices.front());
        auto point = externalVertices[pointIdx];
        externalVertices.erase(begin(externalVertices) + pointIdx);
        addPoint(hullPoints, hullTris, point);
        removeInternalPoints(hullPoints, hullTris, externalVertices);
    }
    removeUnreferencedVertices(hullPoints, hullTris);
}

void ConvexHullBuilder::removeInternalPoints(std::vector<glm::vec3> &hullPoints, std::vector<Tri> &hullTris,
                                             std::vector<glm::vec3> &checkPoints) {
    for(auto i = 0; i < checkPoints.size(); i++){
        const auto& point = checkPoints[i];

        bool isExternal = false;
        for(const auto& tri : hullTris){
            const auto a = hullPoints[tri.a];
            const auto b = hullPoints[tri.b];
            const auto c = hullPoints[tri.c];

            // if the poitn is in front of any triangle then it's external
            auto dist = distanceFromTriangle(a, b, c, point);
            if(dist > 0.0f){
                isExternal = true;
                break;
            }
        }

        // if it's not external, then it's inside the polyhendron and should be removed
        if(!isExternal){
            checkPoints.erase(begin(checkPoints) + i);
            i--;
        }
    }

    // Also remove any points that are just a little too close to the hull points;
    for(int i = 0; i < checkPoints.size(); i++){
        const auto point = checkPoints[i];

        bool isTooClose = false;
        for(const auto& hullPoint : hullPoints){
            auto ray = hullPoint - point;
            if(glm::dot(ray, ray) < 0.0001){    // 1cm is too close
                isTooClose = true;
                break;
            }
        }
        if(isTooClose){
            checkPoints.erase(begin(checkPoints) + i);
            i--;
        }
    }

}

void ConvexHullBuilder::addPoint(std::vector<glm::vec3> &hullPoints, std::vector<Tri> &hullTris, glm::vec3 point) {
    // This point is outside
    // Now we need to remove old triangles and build new ones

    // find all the triangles that face this point
    std::vector<int> facingTris;
    for(auto i = static_cast<int>(hullTris.size()) - 1; i >= 0; i--){
        const auto& tri = hullTris[i];

        const auto& a = hullPoints[tri.a];
        const auto& b = hullPoints[tri.b];
        const auto& c = hullPoints[tri.c];

        const auto dist = distanceFromTriangle(a, b, c, point);
        if(dist > 0.0f){
            facingTris.push_back(i);
        }
    }

    // Now find all edges that are unique to this tris, these will be the edges that form the new triangles
    std::vector<Edge> uniqueEdges;
    for(int i = 0; i < facingTris.size(); i++){
        const auto triIdx = facingTris[i];
        const auto& tri = hullTris[triIdx];

        std::array<Edge, 3> edges{};
        edges[0].a = tri.a;
        edges[0].b = tri.b;

        edges[1].a = tri.b;
        edges[1].b = tri.c;

        edges[2].a = tri.c;
        edges[2].b = tri.a;

        for(const auto& edge : edges){
            if(isEdgeUnique(hullTris, facingTris, triIdx, edge)){
                uniqueEdges.push_back(edge);
            }
        }
    }

    // now remove the old facing tris;
    for(int facingTri : facingTris){
        hullTris.erase(begin(hullTris) + facingTri);
    }

    // now add the new point
    const auto newPointIdx = static_cast<int>(hullPoints.size());
    hullPoints.push_back(point);

    for(const auto& edge : uniqueEdges){
        Tri tri{edge.a, edge.b, newPointIdx};
        hullTris.push_back(tri);
    }
}

bool
ConvexHullBuilder::isEdgeUnique(const std::vector<Tri> &tris, const std::vector<int> &facingTris,  int ignoreTri,
                                const Edge &edge) {
    for(int triIdx : facingTris){
        if(ignoreTri == triIdx){
            continue;
        }

        const auto& tri = tris[triIdx];

        std::array<Edge, 3> edges{};
        edges[0].a = tri.a;
        edges[0].b = tri.b;

        edges[1].a = tri.b;
        edges[1].b = tri.c;

        edges[2].a = tri.c;
        edges[2].b = tri.a;

        for(int e = 0; e < 3; e++){
            if(edge == edges[e]){
                return false;
            }
        }
    }
    return true;
}

void ConvexHullBuilder::removeUnreferencedVertices(std::vector<glm::vec3> &hullPoints, std::vector<Tri> &hullTris) {
    for(int i = 0; i < hullPoints.size(); i++){
        bool isUsed = false;
        for(auto & tri : hullTris){
            if(tri.a == i || tri.b == i || tri.c == i){
                isUsed = true;
                break;
            }
        }
        if(isUsed){
            continue;
        }

        for(auto & tri : hullTris){
            if(tri.a > i){
                tri.a--;
            }
            if(tri.b > i){
                tri.b--;
            }
            if(tri.c > i){
                tri.c--;
            }
            hullPoints.erase(begin(hullPoints) + i);
            i--;
        }
    }
}

void ConvexHullBuilder::build() {
    if(m_points.size() < 4) return;

    buildTetrahedron(m_points, m_hullPoints, m_hullTris);
    expandConvexHull(m_hullPoints, m_hullTris, m_points);
}

bool ConvexHullBuilder::isExternal(const std::vector<glm::vec3> &points, const std::vector<Tri> &tris,
                                   const glm::vec3 &point) {

    for(const auto& tri : tris){
        const auto& a = points[tri.a];
        const auto& b = points[tri.b];
        const auto& c = points[tri.c];
        float dist = distanceFromTriangle(a, b, c, point);
        if(dist > 0.0f){
            return true;
        }
    }
    return false;
}

glm::vec3 ConvexHullBuilder::calculateCenterOfMass() {
    const auto numSamples = 100;

    Bounds bounds;
    bounds.expand(m_hullPoints);

    glm::vec3 com{0};
    const auto d = (bounds.max - bounds.min)/static_cast<float>(numSamples);

    int sampleCount = 0;
    for(auto x = bounds.min.x; x < bounds.max.x; x += d.x){
        for(auto y = bounds.min.y; y < bounds.max.y; y += d.y){
            for(auto z = bounds.min.z; z < bounds.max.z; z += d.z){
                glm::vec3 point(x, y, z);
                if(isExternal(m_hullPoints, m_hullTris, point)){
                    continue;
                }
                com += point;
                sampleCount++;
            }
        }
    }

    com /= static_cast<float>(sampleCount);
    return com;
}

glm::mat4
ConvexHullBuilder::calculateInertiaTensor(const glm::vec3& centerOfMass) {
    const auto numSamples = 100;

    Bounds bounds;
    bounds.expand(m_hullPoints);

    glm::mat3 tensor{0};
    const auto d = (bounds.max - bounds.min)/static_cast<float>(numSamples);

    int sampleCount = 0;
    for(auto x = bounds.min.x; x < bounds.max.x; x += d.x){
        for(auto y = bounds.min.y; y < bounds.max.y; y += d.y){
            for(auto z = bounds.min.z; z < bounds.max.z; z += d.z){
                glm::vec3 point(x, y, z);
                if(isExternal(m_hullPoints, m_hullTris, point)){
                    continue;
                }
                point -= centerOfMass;

                tensor[0][0] += glm::dot(point.yz(), point.yz());
                tensor[1][1] += glm::dot(point.zx(), point.zx());
                tensor[2][2] += glm::dot(point.xy(), point.xy());

                tensor[1][0] += -1.0f * point.x * point.y;
                tensor[2][0] += -1.0f * point.x * point.z;
                tensor[2][1] += -1.0f * point.y * point.z;

                tensor[0][1] += -1.0f * point.x * point.y;
                tensor[0][2] += -1.0f * point.x * point.z;
                tensor[1][2] += -1.0f * point.y * point.z;
                sampleCount++;
            }
        }
    }

    tensor *=  1.0/static_cast<float>(sampleCount);
    return tensor;
}
