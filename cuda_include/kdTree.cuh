#pragma once

#include "core.cuh"
#include "geometry.cuh"
#include "AABB.cuh"
#include <type_traits>
#include <vector>
#include <algorithm>
#include <numeric>
#include <queue>

namespace gsy {
    template<u32 BranchFactor, u32 LeafCapa, typename Location>
    class kdTree;

    template<u32 BranchFactor>
    struct Rope;

    template<>
    struct Rope<8> {
        using idx_type = typename std::conditional_t<std::is_same_v<Float, float>, u32, u64>;
        static constexpr idx_type invalid = std::numeric_limits<idx_type>::max();

        union {
            Float xSplit;
            idx_type isInner;
        };

        union {
            Float ySplit;
            idx_type begin;
        };

        union {
            Float zSplit;
            idx_type end;
        };

        union {
            //in the order of x-, x+, y-, y+, z-, z+
            std::array<idx_type, 6> neighbors;
            std::array<idx_type, 8> children;
        };

        __host__ __device__ constexpr bool is_inner_rope() const { return isInner != invalid; }
        __host__ __device__ constexpr bool is_leaf_rope() const { return isInner == invalid; }
        __host__ __device__ constexpr void set_leaf_rope() { isInner = invalid; }
    };

    struct BuildTask {
        using idx_type = typename std::conditional_t<std::is_same_v<Float, float>, u32, u64>;
        static constexpr idx_type invalid = std::numeric_limits<idx_type>::max();

        AABB aabb;
        std::vector<Geometry> geometries;
        //in the order of x-, x+, y-, y+, z-, z+
        std::array<idx_type, 6> neighbors = {invalid, invalid, invalid, invalid, invalid, invalid};
        idx_type parent = invalid, which = invalid;

        BuildTask() = default;
        BuildTask(AABB aabb, std::vector<Geometry> geometries) : 
            aabb(std::move(aabb)), geometries(std::move(geometries)) {}
    };

    template<u32 LeafCapa>
    class kdTree<8, LeafCapa, Host> {
        private:
            AABB _aabb;
            std::vector<Geometry> _geometries;
            std::vector<Rope<8>>     _ropes;

        public:
            __host__ kdTree(const std::vector<Geometry>& geometries);
    };
}