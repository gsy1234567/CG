#pragma once

#include "core.cuh"
#include <type_traits>
#include <numeric>
#include <cmath>

namespace gsy {
    
    template<typename T, typename... Others>
    __host__ __device__ T min(T x, Others... others) {
        if constexpr (sizeof...(others) == 0) {
            return x;
        } else {
            T other_min = min(others...);
            return x < other_min ? x : other_min;
        }
    }  

    template<typename T, typename... Others>
    __host__ __device__ T max(T x, Others... others) {
        if constexpr (sizeof...(others) == 0) {
            return x;
        } else {
            T other_max = max(others...);
            return x > other_max ? x : other_max;
        }
    }

    template<typename T>
    __host__ __device__ T average(const T& x, const T& y) {
        return (x + y) / static_cast<T>(2);
    }

    template<typename T>
    __host__ __device__ std::enable_if_t<std::is_floating_point_v<T>, bool>
    almost_equal(T x, T y, u32 ulp = 2) {
        return std::fabs(x - y) < std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
            std::fabs(x - y) < std::numeric_limits<T>::min();
    }

    inline __host__ __device__ float fast_abs(float x) {
        u32* p = reinterpret_cast<u32*>(&x);
        *p &= 0x7FFF'FFFFU;
        return x;
    }

    inline __host__ __device__ double fast_abs(double x) {
        u64* p = reinterpret_cast<u64*>(&x);
        *p &= 0x7FFF'FFFF'FFFF'FFFFULL;
        return x;
    }

}
