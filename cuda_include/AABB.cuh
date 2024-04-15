#pragma once

#include "core.cuh"
#include "utils.cuh"
#include "ray.cuh"
#include <type_traits>

namespace gsy {

    struct AABB {
        vec3f min = vec3f::Constant(1e10);
        vec3f max = vec3f::Constant(-1e10);

        __host__ __device__ AABB() = default;
        __host__ __device__ AABB(const vec3f& min, const vec3f& max) : min(min), max(max) {}
        __host__ __device__ AABB(const AABB&) = default;
        __host__ __device__ AABB operator=(const AABB& other) {
            min = other.min;
            max = other.max;
            return *this;
        }

        __host__ __device__ vec3f mid() const { return (min + max) * static_cast<Float>(0.5); }
        
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

        __host__ __device__ Direction get_direction(const vec3f& point, u32 n = 1) const {
            if(fast_abs(point.x() - min.x()) < get_n_ulp_magnitude(min.x(), 2U)) {
                return Direction::left;
            }
            if(fast_abs(point.x() - max.x()) < get_n_ulp_magnitude(max.x(), 2U)) {
                return Direction::right;
            }
            if(fast_abs(point.y() - min.y()) < get_n_ulp_magnitude(min.y(), 2U)) {
                return Direction::back;
            }
            if(fast_abs(point.y() - max.y()) < get_n_ulp_magnitude(max.y(), 2U)) {
                return Direction::front;
            }
            if(fast_abs(point.z() - min.z()) < get_n_ulp_magnitude(min.z(), 2U)) {
                return Direction::bottom;
            }
            if(fast_abs(point.z() - max.z()) < get_n_ulp_magnitude(max.z(), 2U)) {
                return Direction::top;
            }
            return Direction::none;
        }

        __host__ __device__ bool intersect(Ray& ray) const {
            auto x1 = (min.x() - ray.ori.x()) / ray.dir.x();
            auto x2 = (max.x() - ray.ori.x()) / ray.dir.x();
            auto xmin = gsy::min(x1, x2);
            auto xmax = gsy::max(x1, x2);
            auto y1 = (min.y() - ray.ori.y()) / ray.dir.y();
            auto y2 = (max.y() - ray.ori.y()) / ray.dir.y();
            auto ymin = gsy::min(y1, y2);
            auto ymax = gsy::max(y1, y2);
            auto z1 = (min.z() - ray.ori.z()) / ray.dir.z();
            auto z2 = (max.z() - ray.ori.z()) / ray.dir.z();
            auto zmin = gsy::min(z1, z2);
            auto zmax = gsy::max(z1, z2);

            auto tmpMin = gsy::max(ray.tMin, xmin, ymin, zmin);
            auto tmpMax = gsy::min(ray.tMax, xmax, ymax, zmax);

            if(tmpMin < tmpMax) {
                ray.tMin = tmpMin;
                ray.tMax = tmpMax;
                return true;
            } else {
                return false;
            }
        }

        /**
         * @brief We assert the ray intersects with the AABB, then we set lambdaOut to ray.tMin
        */
        __host__ __device__ void ray_exit(Ray& ray) const {
            auto x1 = (min.x() - ray.ori.x()) / ray.dir.x();
            auto x2 = (max.x() - ray.ori.x()) / ray.dir.x();
            auto xmax = gsy::max(x1, x2);
            auto y1 = (min.y() - ray.ori.y()) / ray.dir.y();
            auto y2 = (max.y() - ray.ori.y()) / ray.dir.y();
            auto ymax = gsy::max(y1, y2);
            auto z1 = (min.z() - ray.ori.z()) / ray.dir.z();
            auto z2 = (max.z() - ray.ori.z()) / ray.dir.z();
            auto zmax = gsy::max(z1, z2);

            ray.tMin = gsy::min(xmax, ymax, zmax);
        }
    };
}