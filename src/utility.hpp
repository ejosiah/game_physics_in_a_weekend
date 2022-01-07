#pragma once

#include <algorithm>
#include <glm/glm.hpp>


template<glm::length_t L, typename T, glm::qualifier Q = glm::defaultp>
inline bool isZero(const glm::vec<L, T, Q>& vector){
    return glm::all(glm::equal(glm::vec<L, T, Q>(0), vector));
}

template<glm::length_t L, typename T, glm::qualifier Q = glm::defaultp>
inline bool isClose(const glm::vec<L, T, Q>& vector, const glm::vec<L, T, Q>& target){
    return false;   // TODO
}

template<glm::length_t L, typename T, glm::qualifier Q = glm::defaultp>
inline bool isCloseToZero(const glm::vec<L, T, Q>& vector){
    return isClose(vector, glm::vec<L, T, Q>{0});
}


template<glm::length_t L, typename T, glm::qualifier Q = glm::defaultp>
inline bool isNan(const glm::vec<L, T, Q>& vector){
    return glm::any(glm::isnan(vector));
}

template<glm::length_t L, typename T, glm::qualifier Q = glm::defaultp>
inline bool isInf(const glm::vec<L, T, Q>& vector){
    return glm::any(glm::isinf(vector));
}


template<glm::length_t L, typename T, glm::qualifier Q = glm::defaultp>
void swap(const glm::vec<L, T, Q>& a, const glm::vec<L, T, Q>& b){
    for(auto i = 0; i < L; i++){
        std::swap(a[i], b[i]);
    }
}
