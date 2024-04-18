#pragma once

#include "core.cuh"
#include "ray.cuh"

namespace gsy {

    struct Camera {
        vec3f pos;
        vec3f right; //x+, normalized
        vec3f front; //y+, normalized
        vec3f up;    //z+, normalized
        Float vFov;
        Float focalLen;
        vec2u resolution;
        __host__ __device__ Camera(const vec3f& pos, Float vFov, Float focalLen, const vec2u& resolution) : 
            pos(pos), vFov(vFov),focalLen(focalLen), resolution(resolution) {}

        __host__ __device__ void look_at(const vec3f& lookAt, const vec3f& refUp) {
            front = (lookAt - pos).normalized();
            right = front.cross(refUp).normalized();
            up = right.cross(front).normalized();
        }
        __host__ __device__ Ray generat_ray(Float dx, Float dy) const {
            assert(dx >= 0 && dx < resolution.x());
            assert(dy >= 0 && dy < resolution.y());
            //dx->[-1, 1)
            dx = static_cast<Float>(2) * (dx / resolution.x()) - static_cast<Float>(1);
            //dy->[-1, 1)
            dy = static_cast<Float>(2) * (dy / resolution.y()) - static_cast<Float>(1);
            Float xmax = focalLen * arctan<Float>(vFov / static_cast<Float>(2));
            Float ymax = xmax / resolution.x() * resolution.y();
            return Ray{pos, front * focalLen + right * dx * xmax + up * dy * ymax};
        }
    };
}