#pragma once

#include "core.cuh"

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

}
