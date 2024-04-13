#include "intersection.cuh"
#include "utils.cuh"

namespace gsy {
    
    __host__ __device__ bool intersect(const Ray& ray, const AABB& aabb, float& lambdaIn, float& lambdaOut) {
        auto x1 = (aabb.min.x() - ray.ori.x()) / ray.dir.x();
        auto x2 = (aabb.max.x() - ray.ori.x()) / ray.dir.x();
        auto xmin = min(x1, x2);
        auto xmax = max(x1, x2);
        auto y1 = (aabb.min.y() - ray.ori.y()) / ray.dir.y();
        auto y2 = (aabb.max.y() - ray.ori.y()) / ray.dir.y();
        auto ymin = min(y1, y2);
        auto ymax = max(y1, y2);
        auto z1 = (aabb.min.z() - ray.ori.z()) / ray.dir.z();
        auto z2 = (aabb.max.z() - ray.ori.z()) / ray.dir.z();
        auto zmin = min(z1, z2);
        auto zmax = max(z1, z2);
        lambdaIn = max(ray.tMin, xmin, ymin, zmin);
        lambdaOut = min(ray.tMax, xmax, ymax, zmax);
        return lambdaIn < lambdaOut;
    }

}