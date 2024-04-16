#pragma once

#include "core.cuh"
#include "utils.cuh"
#include "brdf.cuh"
#include "intersection.cuh"
#include "AABB.cuh"
#include "ray.cuh"


namespace gsy {

    struct Geometry {
        BRDF* brdf = nullptr;

        __host__ __device__ Geometry() = default;
        __host__ __device__ virtual ~Geometry() = default;
        __host__ __device__ virtual bool intersect(Interaction& interaction, Ray& ray) const  = 0;
        __host__ __device__ virtual AABB get_aabb() const = 0;
        __host__ __device__ virtual Float x_min() const = 0;
        __host__ __device__ virtual Float x_max() const = 0;
        __host__ __device__ virtual Float y_min() const = 0;
        __host__ __device__ virtual Float y_max() const = 0;
        __host__ __device__ virtual Float z_min() const = 0;
        __host__ __device__ virtual Float z_max() const = 0;
        __host__ __device__ virtual vec3f get_normal(Float u, Float v) const = 0;
    };

    struct Triangle : public Geometry {
        Point3f p0, p1, p2;
        vec3f n0, n1, n2;

        __host__ __device__ Triangle() = default;
        __host__ __device__ Triangle(vec3f p0, vec3f p1, vec3f p2, vec3f n0, vec3f n1, vec3f n2) : 
            p0(p0), p1(p1), p2(p2), n0(n0), n1(n1), n2(n2) {}
        __host__ __device__ ~Triangle() = default;

        __host__ __device__ bool intersect(Interaction& interaction, Ray& ray) const override {
            vec3f e0 = p0 - p1;
            vec3f e1 = p0 - p2;
            vec3f e2 = p0 - ray.ori;
            vec3f cross0 = ray.dir.cross(e0);
            vec3f cross1 = e2.cross(e1);

            Float det_inv = static_cast<Float>(1) / e1.dot(cross0);
            Float t = -e0.dot(cross1) * det_inv;
            if(t > interaction.t + Eps && t > ray.tMin && t < ray.tMax) {
                Float u = ray.dir.dot(cross1) * det_inv;
                Float v = e2.dot(cross0) * det_inv;
                if(
                    u >= static_cast<Float>(0) && 
                    v >= static_cast<Float>(0) && 
                    (static_cast<Float>(1) - u - v) >= static_cast<Float>(0)
                ) {
                    interaction.t = t;
                    interaction.u = u;
                    interaction.v = v;
                    interaction.brdf = brdf;
                    interaction.type = Interaction::Type::Geometry;
                    ray.tMax = t;
                    return true;
                }
            }
            return false;
        }

        __host__ __device__ AABB get_aabb() const override {
            AABB aabb {p0, p0};
            aabb.min = aabb.min.cwiseMin(p1.cwiseMin(p2));
            aabb.max = aabb.max.cwiseMax(p1.cwiseMax(p2));
            return aabb;
        }

        __host__ __device__ Float x_min() const override {
            return min(p0.x(), p1.x(), p2.x());
        }

        __host__ __device__ Float x_max() const override {
            return max(p0.x(), p1.x(), p2.x());
        }

        __host__ __device__ Float y_min() const override {
            return min(p0.y(), p1.y(), p2.y());
        }

        __host__ __device__ Float y_max() const override {
            return max(p0.y(), p1.y(), p2.y());
        }

        __host__ __device__ Float z_min() const override {
            return min(p0.z(), p1.z(), p2.z());
        }

        __host__ __device__ Float z_max() const override {
            return max(p0.z(), p1.z(), p2.z());
        }

        __host__ __device__ vec3f get_normal(Float u, Float v) const override {
            return (n0 * (static_cast<Float>(1) - u - v) + n1 * u + n2 * v).normalized();
        } 
    };
}