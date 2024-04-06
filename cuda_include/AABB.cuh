#pragma once

#include "core.cuh"
#include "utils.cuh"
#include <type_traits>

namespace gsy {
    struct AABB {
        vec3f min = vec3f::Constant(Inf);
        vec3f max = vec3f::Constant(-Inf);

        __host__ __device__ AABB() = default;
        __host__ __device__ AABB(const vec3f& min, const vec3f& max) : min(min), max(max) {}
        
        __host__ __device__ AABB expand(const AABB& other) {
            min = min.cwiseMin(other.min);
            max = max.cwiseMax(other.max);
            return *this;
        }

        __host__ __device__ bool has_intersection(const AABB& other) const {
            return  max.x() > other.min.x() &&
                    max.y() > other.min.y() &&
                    max.z() > other.min.z() &&
                    min.x() < other.max.x() &&
                    min.y() < other.max.y() &&
                    min.z() < other.max.z();
        }

        template<typename X, typename Y, typename Z>
        __host__ __device__ AABB get_eighth() const {
            AABB ret;
            if constexpr (std::is_same_v<X, Negative>) {
                ret.min.x() = min.x();
                ret.max.x() = average(min.x(), max.x());
            } else {
                ret.min.x() = average(min.x(), max.x());
                ret.max.x() = max.x();
            }

            if constexpr (std::is_same_v<Y, Negative>) {
                ret.min.y() = min.y();
                ret.max.y() = average(min.y(), max.y());
            } else {
                ret.min.y() = average(min.y(), max.y());
                ret.max.y() = max.y();
            }

            if constexpr (std::is_same_v<Z, Negative>) {
                ret.min.z() = min.z();
                ret.max.z() = average(min.z(), max.z());
            } else {
                ret.min.z() = average(min.z(), max.z());
                ret.max.z() = max.z();
            }

            return ret;
        }
    };
}