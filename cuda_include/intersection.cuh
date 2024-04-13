#pragma once

#include "core.cuh"
#include "brdf.cuh"
#include "ray.cuh"
#include "AABB.cuh"

namespace gsy {

    struct Interaction {
        enum struct Type : u8 {
            None, 
            Geometry, 
            Light
        } type = Type::None;

        Float t = -1, u = -1, v = -1;
        BRDF* brdf = nullptr;
    };

    /**
     * @brief Given a ray `ray` and an Axis Aligned Bounding Box `aabb`, judge whether the ray intersects with the aabb. If they intersect, calculate `lambdaIn` and `lambdaOut`.
     * @return Whether the ray intersects with the aabb.
     * @param[in] ray The input light ray.
     * @param[in] aabb The input Axis Aligned Bounding Box.
     * @param[out] lambdaIn The output lambda in, it is valid only if this function returns true.
     * @param[out] lambdaOut The output lambda out, it is valid only if this function returns true.
    */
    __host__ __device__ bool intersect(const Ray& ray, const AABB& aabb, float& lambdaIn, float& lambdaOut);
}