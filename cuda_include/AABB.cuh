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

        __host__ __device__ Direction get_direction(const vec3f& point) const {
            Float v = std::numeric_limits<Float>::max();
            Direction ret = Direction::none;
            Float test;
            if((test = std::fabs((point.x() - min.x()) / min.x())) < v) {
                v = test;
                ret = Direction::left;
            }
            if((test = std::fabs((point.x() - max.x()) / max.x())) < v) {
                v = test;
                ret = Direction::right;
            }
            if((test = std::fabs((point.y() - min.y()) / min.y())) < v) {
                v = test;
                ret = Direction::back;
            }
            if((test = std::fabs((point.y() - max.y()) / max.y())) < v) {
                v = test;
                ret = Direction::front;
            }
            if((test = std::fabs((point.z() - min.z()) / min.z())) < v) {
                v = test;
                ret = Direction::bottom;
            }
            if((test = std::fabs((point.z() - max.z()) / max.z())) < v) {
                v = test;
                ret = Direction::top;
            }
            return ret;
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
        __host__ __device__ Direction ray_exit(Ray& ray) const {
            auto x1 = (min.x() - ray.ori.x()) / ray.dir.x();
            auto x2 = (max.x() - ray.ori.x()) / ray.dir.x();
            Direction dirx;
            Float xmax;
            if(x1 > x2) {
                dirx = Direction::left;
                xmax = x1;
            } else {
                dirx = Direction::right;
                xmax = x2;
            }

            auto y1 = (min.y() - ray.ori.y()) / ray.dir.y();
            auto y2 = (max.y() - ray.ori.y()) / ray.dir.y();
            Direction diry;
            Float ymax;
            if(y1 > y2) {
                diry = Direction::back;
                ymax = y1;
            } else {
                diry = Direction::front;
                ymax = y2;
            }

            auto z1 = (min.z() - ray.ori.z()) / ray.dir.z();
            auto z2 = (max.z() - ray.ori.z()) / ray.dir.z();
            Direction dirz;
            Float zmax;
            if(z1 > z2) {
                dirz = Direction::bottom;
                zmax = z1;
            } else {
                dirz = Direction::top;
                zmax = z2;
            }

            Direction dirxy;
            Float xymin;
            if(xmax <= ymax) {
                xymin = xmax;
                dirxy = dirx;
            } else {
                xymin = ymax;
                dirxy = diry;
            }

            if(xymin <= zmax) {
                ray.tMin = xymin;
                return dirxy;
            } else {
                ray.tMin = zmax;
                return dirz;
            }
        }
    };
}