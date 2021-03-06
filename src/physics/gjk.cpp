#include "gjk.hpp"
#include "signedVolume.hpp"
#include <spdlog/spdlog.h>
#include <vulkan_util/glm_format.h>
#include <sstream>

Point GJK::support(const Body *bodyA, const Body *bodyB, glm::vec3 dir, float bias) {
    auto tmp = dir;
    dir = glm::normalize(dir);

    if(glm::any(isnan(dir))){
        assert(false);
    }

    Point point{};

    // Find the point furthest in direction dir
    point.pointA = bodyA->shape->support(dir, bodyA->position, bodyA->orientation, bias);
    dir *= -1;

    // Find the point furthest in opposite direction
    point.pointB = bodyB->shape->support(dir, bodyB->position, bodyB->orientation, bias);

    // Return the point, in the minkowski sum, furthest in the direction
    point.xyz = point.pointA - point.pointB;
    return point;
}

bool GJK::simplexSignedVolumes(Point *points, const int num, glm::vec3 &newDir, glm::vec4 &lambdasOut) {
    const float epsilonf = 1E-8;
    lambdasOut = glm::vec4(0);

    auto isZero = [](auto v){
        return glm::all(glm::equal(v, decltype(v)(0)));
    };

    bool doesIntersect = false;
    switch (num) {
        default:
        case 2 : {
            auto lambdas = SignedVolume::_1D(points[0].xyz, points[1].xyz);
            glm::vec3 v(0);
            v += points[0].xyz * lambdas[0];
            v += points[1].xyz * lambdas[1];
            newDir = v * -1.0f;
            if(isZero(lambdas)){
                for(int i = 0; i < num; i++){
                    auto point = points[i];
                }
                assert(!isZero(lambdas));
            }
            doesIntersect = (glm::dot(v, v) < epsilonf);
            lambdasOut[0] = lambdas[0];
            lambdasOut[1] = lambdas[1];
        } break;
        case 3: {
            auto lambdas = SignedVolume::_2D(points[0].xyz, points[1].xyz, points[2].xyz);
            glm::vec3 v(0);
            v += points[0].xyz * lambdas[0];
            v += points[1].xyz * lambdas[1];
            v += points[2].xyz * lambdas[2];
            newDir = v * -1.0f;
            if(isZero(lambdas)){
                for(int i = 0; i < num; i++){
                    auto point = points[i];
                    spdlog::info("cso: {}", point.xyz);
                }
                assert(!isZero(lambdas));
            }
            doesIntersect = (glm::dot(v, v) < epsilonf);
            lambdasOut[0] = lambdas[0];
            lambdasOut[1] = lambdas[1];
            lambdasOut[2] = lambdas[2];
        } break;
        case 4: {
            auto lambdas = SignedVolume::_3D(points[0].xyz, points[1].xyz, points[2].xyz, points[3].xyz);
            glm::vec3 v(0);
            v += points[0].xyz * lambdas[0];
            v += points[1].xyz * lambdas[1];
            v += points[2].xyz * lambdas[2];
            v += points[3].xyz * lambdas[3];
            newDir = v * -1.0f;
            if(isZero(lambdas)){
                for(int i = 0; i < num; i++){
                    auto point = points[i];
                }
                assert(!isZero(lambdas));
            }
            doesIntersect = (glm::dot(v, v) < epsilonf);
            lambdasOut[0] = lambdas[0];
            lambdasOut[1] = lambdas[1];
            lambdasOut[2] = lambdas[2];
            lambdasOut[3] = lambdas[3];
        } break;
    }
    return doesIntersect;
}

bool GJK::hasPoint(const std::array<Point, 4>& simplexPoints, const Point &newPoint) {
    constexpr float precision = 1E-6f;
    for(int i = 0; i < 4; i++){
        auto delta = simplexPoints[i].xyz - newPoint.xyz;
        if(glm::dot(delta, delta) < precision * precision){
            return true;
        }
    }
    return false;
}

void GJK::sortValids(std::array<Point, 4> &simplexPoints, glm::vec4 &lambdas) {
    std::array<bool, 4> valids{};
    for(int i = 0; i < 4; i++){
        valids[i] = true;
        if(lambdas[i] == 0){
            valids[i] = false;
        }
    }

    glm::vec4 validLambdas(0);
    int validCount = 0;
    std::array<Point, 4> validPoints{};

    for(int i = 0; i < 4; i++){
        if(valids[i]){
            validPoints[validCount] = simplexPoints[i];
            validLambdas[validCount] = lambdas[i];
            validCount++;
        }
    }

    for(int i = 0; i < 4; i++){
        simplexPoints[i] = validPoints[i];
        lambdas[i] = validLambdas[i];
    }
}

int GJK::numValids(const glm::vec4 &lambdas) {
    int num = 0;
    for(int i = 0; i < 4; i++){
        if(lambdas[i] != 0){
            num++;
        }
    }
    return num;
}

bool GJK::doesIntersect(const Body *bodyA, const Body *bodyB, float bias, glm::vec3& pointOnA, glm::vec3& pointOnB) {
    const glm::vec3 origin(0);

    int numPoints = 1;
    std::array<Point, 4> simplexPoints{};
    simplexPoints[0] = support(bodyA, bodyB, glm::vec3(1));

    float closestDist = 1E10f;
    bool doesContainOrigin = false;
    auto newDir = -simplexPoints[0].xyz;
    static int noIntersect = 0;
    do{
        noIntersect++;
        // Get the new point to check on
        auto newPoint = support(bodyA, bodyB, newDir);

        // if the new point is the same as a previous point, then we can't expand any further
        if(hasPoint(simplexPoints, newPoint)){
            break;
        }

        simplexPoints[numPoints] = newPoint;
        numPoints++;

        // if this new point hasn't moved passed the origin, then the
        // origin cannot be in the set. And therefore there is no collision
        if(glm::dot(newDir, newPoint.xyz - origin) < 0.0f){
            break;
        }

        glm::vec4 lambdas;
        doesContainOrigin = simplexSignedVolumes(simplexPoints.data(), numPoints, newDir, lambdas);
        if(doesContainOrigin){
            break;
        }

        // Check that the new projectin of the origin onto the simplex is closer than the previous
        auto dist = glm::dot(newDir, newDir);
        if(dist >= closestDist){
            break;
        }
        closestDist = dist;

        // use the lambdas that support the new search direction, and invalidate any points that don't support it
        sortValids(simplexPoints, lambdas);
        numPoints = numValids(lambdas);
        doesContainOrigin = (numPoints == 4);

    } while (!doesContainOrigin);

    if(!doesContainOrigin){
        return false;
    }

    // check that we have a 3-simplex (EPA expects a tetrahedron
    if(numPoints == 1){
        auto searchDir = simplexPoints[0].xyz * -1.0f;
        auto newPoint = support(bodyA, bodyB, searchDir);
        simplexPoints[numPoints] = newPoint;
        numPoints++;
    }
    if(numPoints == 2){
        auto ab = simplexPoints[1].xyz - simplexPoints[0].xyz;
        glm::vec3 u, v;
        orthonormal(ab, u, v);
        auto newDir = u;
        auto newPoint = support(bodyA, bodyB, newDir);
        simplexPoints[numPoints] = newPoint;
        numPoints++;
    }
    if(numPoints == 3){
        auto ab = simplexPoints[1].xyz - simplexPoints[0].xyz;
        auto ac = simplexPoints[2].xyz - simplexPoints[0].xyz;
        auto normal = glm::cross(ab, ac);

        auto newDir = normal;
        auto newPoint = support(bodyA, bodyB, newDir);
        simplexPoints[numPoints] = newPoint;
        numPoints++;
    }

    // Expand the simplex by the bias amount

    // Get the center point of the simplex
    glm::vec3 avg{0};
    for(const auto& point : simplexPoints){
        avg += point.xyz;
    }
    avg *= 0.25;

    // Now expand the simplex by the bias amount
    for(auto i = 0; i < numPoints; i++){
        auto& pt = simplexPoints[i];

        auto dir = pt.xyz - avg; // ray from "center" to witness point
        dir = glm::normalize(dir);
        pt.pointA += dir * bias;
        pt.pointB += dir * bias;
        pt.xyz = pt.pointA - pt.pointB;
    }

//    spdlog::info("no intersect: {}", noIntersect);
    EPA::expand(bodyA, bodyB, bias, simplexPoints, pointOnA, pointOnB);

    return true;
}

void GJK::closestPoint(const Body *bodyA, const Body *bodyB, glm::vec3 &pointOnA, glm::vec3& pointOnB) {
    const glm::vec3 origin{0};
    auto closestDist = 1E10f;

    int numPoints = 1;
    std::array<Point, 4> simplexPoints{};
    simplexPoints[0] = support(bodyA, bodyB, glm::vec3(1));

    glm::vec4 lambdas{1, 0, 0, 0};
    auto newDir = -simplexPoints[0].xyz;

    do{
        auto newPoint = support(bodyA, bodyB, newDir);

        // if the new point is the same as the previous point, then we can't expand any further;
        if(hasPoint(simplexPoints, newPoint)){
            break;
        }

        // Add point and get new search direction
        simplexPoints[numPoints] = newPoint;
        numPoints++;

        simplexSignedVolumes(simplexPoints.data(), numPoints, newDir, lambdas);
        sortValids(simplexPoints, lambdas);
        numPoints = numValids(lambdas);

        // Check that the new projection of the origin onto the simplex is closer than the previous
        auto dist = glm::dot(newDir, newDir);
        if(dist >= closestDist){
            break;
        }
        closestDist = dist;
    } while (numPoints < 4);

    pointOnA = glm::vec3{0};
    pointOnB = glm::vec3{0};
    for(auto i = 0; i < 4; i++){
        pointOnA += simplexPoints[i].pointA * lambdas[i];
        pointOnB += simplexPoints[i].pointB * lambdas[i];
    }
}

glm::vec3 EPA::normalDirection(const Tri &tri, const std::vector<Point> &points) {
    const auto& a = points[tri.a].xyz;
    const auto& b = points[tri.b].xyz;
    const auto& c = points[tri.c].xyz;

    auto ab = b - a;
    auto ac = c - a;
    auto normal = glm::normalize(glm::cross(ab, ac));
    return normal;
}

float EPA::signedDistanceToTriangle(const Tri &tri, const glm::vec3 &point, const std::vector<Point> &points) {
    const auto normal = normalDirection(tri, points);
    const auto& a = points[tri.a].xyz;
    const auto a2pt = point - a;
    const auto dist = glm::dot(normal, a2pt);
    return dist;
}

int EPA::closestTriangle(const std::vector<Tri> &triangles, const std::vector<Point> &points) {
    auto minDistSqr = 1e10;

    int idx = -1;
    for(int i = 0; i < triangles.size(); i++){
        const auto& tri = triangles[i];
        float dist = signedDistanceToTriangle(tri, glm::vec3(0), points);
        float distSqr = dist * dist;
        if(distSqr < minDistSqr){
            idx = i;
            minDistSqr = distSqr;
        }
    }
    return idx;
}

bool EPA::hasPoint(const glm::vec3 &w, const std::vector<Tri> &triangles, const std::vector<Point> &points) {
    const auto epsilon = 1E-8;
    glm::vec3 delta{0};

    for(auto tri : triangles){
        delta = w - points[tri.a].xyz;
        if(glm::dot(delta, delta) < epsilon){
            return true;
        }

        delta = w - points[tri.b].xyz;
        if(glm::dot(delta, delta) < epsilon){
            return true;
        }

        delta = w - points[tri.c].xyz;
        if(glm::dot(delta, delta) < epsilon){
            return true;
        }
    }
    return false;
}

int
EPA::removeTrianglesFacingPoint(const glm::vec3 &point, std::vector<Tri> &triangles, const std::vector<Point> &points) {
    int numRemoved = 0;
    for(auto i = 0; i < triangles.size(); i++){
        const auto& tri = triangles[i];
        auto dist = signedDistanceToTriangle(tri, point, points);
        if(dist > 0.0f){
            triangles.erase(begin(triangles) + i);
            i--;
            numRemoved++;
        }
    }
    return numRemoved;
}

void EPA::findDanglingEdges(std::vector<Edge> &danglingEdges, const std::vector<Tri> &triangles) {
    danglingEdges.clear();

    for(auto i = 0; i < triangles.size(); i++){
        const auto& tri = triangles[i];

        std::array<Edge, 3> edges{};
        edges[0].a = tri.a;
        edges[0].b = tri.b;

        edges[1].a = tri.b;
        edges[1].b = tri.c;

        edges[2].a = tri.c;
        edges[2].b = tri.a;

        std::array<int, 3> count{0};

        for(auto j = 0; j < triangles.size(); j++){
            if(j == i){
                continue;
            }

            const auto& tri2 = triangles[j];

            std::array<Edge, 3> edges2{};
            edges2[0].a = tri2.a;
            edges2[0].b = tri2.b;

            edges2[1].a = tri2.b;
            edges2[1].b = tri2.c;

            edges2[2].a = tri2.c;
            edges2[2].b = tri2.a;

            for(auto k = 0; k < 3; k++){
                if(edges[k] == edges2[0]){
                    count[k]++;
                }
                if(edges[k] == edges2[1]){
                    count[k]++;
                }
                if(edges[k] == edges2[2]){
                    count[k]++;
                }
            }
        }

        // An edge that isn't shared is dangling
        for(int k = 0; k < 3; k++){
            if(count[k] == 0){
                danglingEdges.push_back(edges[k]);
            }
        }
    }
}

float
EPA::expand(const Body *bodyA, const Body *bodyB, float bias, const std::array<Point, 4> &simplexPoints, glm::vec3 &pointOnA,
            glm::vec3 &pointOnB) {

    std::vector<Point> points;
    std::vector<Tri> triangles;
    std::vector<Edge> danglingEdges;

    glm::vec3 center(0);
    for(int i = 0; i < 4; i++){
        points.push_back(simplexPoints[i]);
        center += simplexPoints[i].xyz;
    }
    center *= 0.25f;

    // build the triangles
    bool inverted = false;
    for(int i = 0; i < 4; i++){
        int j = (i + 1) % 4;
        int k = (i + 2) % 4;
        Tri tri{};
        tri.a = i;
        tri.b = j;
        tri.c = k;

        int unusedPoint = (i + 3) % 4;
        auto dist = signedDistanceToTriangle(tri, points[unusedPoint].xyz, points);

        // The unused point is always on the negative/inside of the triangle .. make sure the normal points away
        if(dist > 0.0f){
            inverted = true;
            std::swap(tri.a, tri.b);
        }

        triangles.push_back(tri);
    }

    if(!inverted){
        std::swap(triangles[0].a, triangles[0].b);
    }

    // expand the simplex to find the closest face of the CSO to the origin'
    while(true){
        const auto idx = closestTriangle(triangles, points);
        auto normal = normalDirection(triangles[idx], points);

        const auto newPoint = GJK::support(bodyA, bodyB, normal, bias);

        // if w already exists, then just stop, because it means we can't expand any further
        if(hasPoint(newPoint.xyz, triangles, points)){
            break;
        }

        float dist = signedDistanceToTriangle(triangles[idx], newPoint.xyz, points);
        if(dist <= 0){
            break; // can't expand
        }

        const auto newIdx = static_cast<int>(points.size());
        points.push_back(newPoint);

        // remove triangles that face this point
//        spdlog::info("newPoint: {}", newPoint.xyz);
        int numRemoved = removeTrianglesFacingPoint(newPoint.xyz, triangles, points);
        if(numRemoved == 0){
            break;
        }

        // find dangling edges
        danglingEdges.clear();
        findDanglingEdges(danglingEdges, triangles);
        if(danglingEdges.empty()){
            break;
        }

        // In theory the edges should be a proper CCW order
        // so we only need to add the new point as 'a' in order
        // to create new triangles that face away from origin
        for(auto & edge : danglingEdges){
            Tri triangle{newIdx, edge.b, edge.a};

            dist = signedDistanceToTriangle(triangle, center, points);
            if(dist > 0.0f){
                std::swap(triangle.b, triangle.c);
            }
            triangles.push_back(triangle);
        }

    }

    // Get the projection of the origin on the closest triangle
    const auto idx = closestTriangle(triangles, points);
    const auto& tri = triangles[idx];
    auto ptA_w = points[tri.a].xyz;
    auto ptB_w = points[tri.b].xyz;
    auto ptC_w = points[tri.c].xyz;
    auto lambdas = barycentricCoordinates(ptA_w, ptB_w, ptC_w, glm::vec3(0));

    // GEt the point on shape A
    auto ptA_a = points[tri.a].pointA;
    auto ptB_a = points[tri.b].pointA;
    auto ptC_a = points[tri.c].pointA;
    pointOnA = ptA_a * lambdas[0] + ptB_a * lambdas[1] + ptC_a * lambdas[2];

    // Get the point on B
    auto ptA_b = points[tri.a].pointB;
    auto ptB_b = points[tri.b].pointB;
    auto ptC_b = points[tri.c].pointB;
    pointOnB = ptA_b * lambdas[0] + ptB_b * lambdas[1] + ptC_b * lambdas[2];

    // return the penetration distance
    auto delta = pointOnB - pointOnA;

    return glm::length(delta);
}


