#pragma once
#include <algorithm>
#include <initializer_list>
#include <array>

template<size_t N>
struct vec{
    std::array<float, N> data{};

    vec() = default;

    constexpr explicit vec(float value){
        std::fill_n(begin(data), N, value);
    }

    constexpr vec(std::initializer_list<float> values){
        assert(values.size() == N);
        auto itr = begin(values);
        for(int i = 0; i < N; i++){
            data[i] = *itr;
            std::advance(itr, 1);
        }
    }

    const float& operator[](const int idx) const {
        return data[idx];
    }

    float& operator[](const int idx) {
        return data[idx];
    }

    vec<N>& operator*=(float rhs) {
        for(int i = 0; i < N; i++){
            data[i] *= rhs;
        }
        return *this;
    }

    constexpr vec<N> operator*(float rhs) const {
        vec<N> v;
        for(int i = 0; i < N; i++){
            v[i] = data[i] * rhs;
        }
        return v;
    }

    template<size_t M>
    constexpr vec<N> operator+(const vec<M>& rhs) const {
        static_assert(M == N);
        vec<N> v;
        for(int i = 0; i < N; i++){
            v[i] = data[i] + rhs.data[i];
        }
        return v;
    }

    template<size_t M>
    constexpr vec<N> operator-(const vec<M>& rhs) const {
        static_assert(M == N);
        vec<N> v;
        for(int i = 0; i < N; i++){
            v[i] = data[i] - rhs[i];
        }
        return v;
    }

    template<size_t M>
    constexpr vec<N>& operator -=(const vec<M>& rhs) {
        static_assert(M == N);
        for(int i = 0; i < N; i++){
            data[i] -= rhs[i];
        }
        return *this;
    }

    template<size_t M>
    vec<N>& operator +=(const vec<M>& rhs) {
        static_assert(M == N);
        for(int i = 0; i < N; i++){
            data[i] += rhs[i];
        }
        return *this;
    }

    template<size_t M>
    constexpr float dot(const vec<M>& rhs) const {
        static_assert(M == N);
        float result = 0;
        for(int i = 0; i < N; i++){
            result += data[i] * rhs[i];
        }
        return result;
    }

    void clear() {
        std::fill_n(begin(data), N, 0);
    }

};