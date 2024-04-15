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

    inline __host__ __device__ float get_n_ulp_magnitude(float x, u32 n = 1U) {
        u32* p = reinterpret_cast<u32*>(&x);
        *p &= 0x7F80'0000U;
        *p |= n;
        return x;
    }

    inline __host__ __device__ double get_n_ulp_magnitude(double x, u64 n = 1ULL) {
        u64* p = reinterpret_cast<u64*>(&x);
        *p &= 0x7FF0'0000'0000'0000ULL;
        *p |= n;
        return x;
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
