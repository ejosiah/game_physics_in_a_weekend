#include "convexhullbuilder.hpp"
#include <limits>

ConvexHullBuilder::ConvexHullBuilder(const std::vector<glm::vec3> &points)
: m_points(points)
{

}

int ConvexHullBuilder::findPointFurthestInDir(const std::vector<glm::vec3> &points, const glm::vec3 &dir) {
    int maxId = 0;
    float maxDist = glm::dot(dir, points.front());
    for(int i = 1; i < points.size(); i++){
        auto dist = glm::dot(dir, points[i]);
        if(maxDist < dist){
            maxDist = dist;
            maxId = i;
        }
    }
    return maxId;
}

float ConvexHullBuilder::distanceFromLine(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &point) {
    auto ab = glm::normalize(b - a);
    auto ray = point - a;
    auto p = ab * glm::dot(ray, ab);
    return glm::length(ray - p);
}

glm::vec3 ConvexHullBuilder::findPointFurthestFromLine(const std::vector<glm::vec3> &points, const glm::vec3 &pointA,
                                                       const glm::vec3 &pointB) {
    int maxId = 0;
    float maxDist = distanceFromLine(pointA, pointB, points.front());
    for(auto i = 1; i < points.size(); i++){
        auto dist = distanceFromLine(pointA, pointB, points[i]);
        if(dist > maxDist){
            maxDist = dist;
            maxId = i;
        }
    }
    return points[maxId];
}

float ConvexHullBuilder::distanceFromTriangle(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c,
                                              const glm::vec3 &point) {
    auto ab = b - a;
    auto ac = c - a;
    auto n = glm::normalize(glm::cross(ab, ac));

    auto ray = point - a;

    return glm::dot(ray, n);
}

glm::vec3 ConvexHullBuilder::findPointFurthestFromTriangle(const std::vector<glm::vec3> &points, const glm::vec3 &a,
                                                           const glm::vec3 &b, const glm::vec3& c) {
    int maxId = 0;
    float maxDist = distanceFromTriangle(a, b, c, points.front());
    for(int i = 1; i < points.size(); i++){
        float dist = distanceFromTriangle(a, b, c, points[i]);
        if(dist > maxDist){
            maxDist = dist;
            maxId = i;
        }
    }
    return points[maxId];
}

void ConvexHullBuilder::buildTetrahedron(const std::vector<glm::vec3>& vertices, std::vector<glm::vec3> &hullPoints,
                                         std::vector<Tri> &hullTris) {
    hullPoints.clear();
    hullTris.clear();

    std::array<glm::vec3, 4> points{};

    auto idx = findPointFurthestInDir( vertices, {1, 0, 0});
    points[0] = vertices[idx];
    idx = findPointFurthestInDir(vertices, -points[0]);
    points[1] = vertices[idx];
    points[2] = findPointFurthestFromLine(vertices, points[0], points[1]);
    points[3] = findPointFurthestFromTriangle(vertices, points[0], points[1], points[2]);

    float dist = distanceFromTriangle(points[0], points[1], points[2], points[3]);
    if(dist > 0.0f){
        std::swap(points[0], points[1]);
    }

    hullPoints.push_back(points[0]);
    hullPoints.push_back(points[1]);
    hullPoints.push_back(points[2]);
    hullPoints.push_back(points[3]);

    Tri tri{0, 1, 2};
    hullTris.push_back(tri);

    tri = Tri{0, 2, 3};
    hullTris.push_back(tri);

    tri = Tri{2, 1, 3};
    hullTris.push_back(tri);

    tri = Tri{1, 0, 3};
    hullTris.push_back(tri);
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
                                const ConvexHullBuilder::Edge &edge) {
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
