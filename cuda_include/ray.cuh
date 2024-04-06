#pragma once 

#include "core.cuh"

namespace gsy {
    struct Ray {
        vec3f ori;
        vec3f dir;
        Float tMin = 1e-7;
        Float tMax = Inf;

        __host__ __device__ Ray(const vec3f& o, const vec3f& d) : 
            ori{o}, dir{d.normalized()} {}

        __host__ __device__ vec3f operator()(Float t) const {
            return ori + dir * t;
        }
    };
}
