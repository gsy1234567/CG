#pragma once

#include "core.cuh"
#include <vector>
#include <array>
#include <utility>
#include <iostream>

namespace gsy {
    #pragma pack (1)
    struct LeafNode {
        u32 neighbors[6];
        u32 begin;
        u16 size;
    };
    #pragma pack ()

    #pragma pack (1)
    struct InnerNode {
        Float splitPos[3];
        u32 children[4];
        Axis split0 : 2;
        Axis split10 : 2;
        Axis split11 : 2;
    };
    #pragma pack ()

    #pragma pack (1)
    struct Rope{
        union {
            LeafNode leafNode;
            InnerNode innerNode;
        };

        u16 isLeaf;
    };
    #pragma pack ()

    struct BuildTask {

        //points[0]:ObjPoint in x axis.
        //points[1]:ObjPoint in y axis.
        //points[2]:ObjPoint in z axis.
        //For each axis, ObjPoint should be in ascending order.
        std::array<std::vector<ObjPoint<Float, 1>>, 3> points;

        //neighbors[0]:The neighbor in x- direction.
        //neighbors[1]:The neighbor in x+ direction.
        //neighbors[2]:The neighbor in y- direction.
        //neighbors[3]:The neighbor in y+ direction.
        //neighbors[4]:The neighbor in z- direction.
        //neighbors[5]:The neighbor in z+ direction.
        u32 neighbors[6] {
            static_cast<u32>(-1), 
            static_cast<u32>(-1), 
            static_cast<u32>(-1), 
            static_cast<u32>(-1), 
            static_cast<u32>(-1), 
            static_cast<u32>(-1)
        };
    };

    /**
     * @brief Choose the best axis and the best split position of `task`.
     * @param[in] task Is the input task.
     * @param[out] subTask0 Is the output task which contains the `ObjPoints` which has `value` smaller than the split position.
     * @param[out] subTask1 Is the output task which contains the `ObjPoints` which has `value` larger than the split position.
    */
    __host__ std::pair<Axis, Float> choose_split_axis(
        const BuildTask& task, 
        BuildTask& subTask0, 
        BuildTask& subTask1
    );

    /**
     * @param[in] parentNeighbors The parent's neighbors, in the order of x-, x+, y-, y+, z-, z+.
     * @param[out] child0Neighbors The child's neighbors, in the order of x-, x+, y-, y+, z-, z+. The `ObjPoint.value` of this child is smaller.
     * @param[out] child1Neighbors The child's neighbors, in the order of x-, x+, y-, y+, z-, z+. The `ObjPoint.value` of this child is larger.
     * @param[in] splitAxis The split Axis.
     * @param[in] parentIndex The index of parent rope node in all rope nodes.
    */
    __host__ void first_find_neighbors(
        const u32 parentNeighbors[6], 
        u32 child0Neighbors[6], 
        u32 child1Neighbors[6], 
        Axis splitAxis, 
        u32 parentIndex
    );

    /**
     * @param[in] parentNeighbors The parent's neighbors, in the order of x-, x+, y-, y+, z-, z+.
     * @param[out] child0Neighbors The child's neighbors, in the order of x-, x+, y-, y+, z-, z+. The `ObjPoint.value` of this child is smaller.
     * @param[out] child1Neighbors The child's neighbors, in the order of x-, x+, y-, y+, z-, z+. The `ObjPoint.value` of this child is larger.
     * @param[in] splitAxis The split Axis.
     * @param[in] child0Index The index of chuild0 rope node in all rope nodes.
    */
    __host__ void second_find_neighbors(
        const u32 parentNeighbors[6], 
        u32 child0Neighbors[6], 
        u32 child1Neighbors[6], 
        Axis splitAxis, 
        u32 child0Index
    );

    /**
     * @brief Given a `buildTask`, build a kdTree in the form of `ropes`. Each leaf-node of this kdTree has no more than `leafCapa` geometries. At the same time, this function reorders the geometries, the results are saved in `indices`.
     * @param[out] ropes The output `ropes`, which saves the new built kdTree, any origin data in it will be cleared.
     * @param[in] buildTask The input `buildTask`.
     * @param[in] leafCapa The input `leafCapa`.
     * @param[out] indices The output `indices`.
    */
    __host__ void build(
        std::vector<Rope>& ropes, 
        BuildTask buildTask, 
        const u16 leafCapa, 
        std::vector<u32>& indices
    );

    template<typename GeometryType, typename Location>
    class CompressedkdTree;

    template<typename GeometryType>
    class CompressedkdTree<GeometryType, Host> {
        private:
            std::vector<GeometryType> _geometries;
            std::vector<Rope> _ropes;
            const u16 _leafCapa;
        public:
            __host__ CompressedkdTree(std::vector<GeometryType> geometries, u16 leafCapa) : 
                _leafCapa(leafCapa) {
                //construct the original build task
                BuildTask buildTask;
                for(u32 i = 0 ; i < geometries.size() ; ++i) {
                    buildTask.points[0].emplace_back(geometries[i].x_min(), i);
                    buildTask.points[0].emplace_back(geometries[i].x_max(), i);
                    buildTask.points[1].emplace_back(geometries[i].y_min(), i);
                    buildTask.points[1].emplace_back(geometries[i].y_max(), i);
                    buildTask.points[2].emplace_back(geometries[i].z_min(), i);
                    buildTask.points[2].emplace_back(geometries[i].z_max(), i);
                }

                std::sort(buildTask.points[0].begin(), buildTask.points[0].end());
                std::sort(buildTask.points[1].begin(), buildTask.points[1].end());
                std::sort(buildTask.points[2].begin(), buildTask.points[2].end());

                //construct a vector to save the new indices
                std::vector<u32> newIndices;

                //build the kdTree
                build(_ropes, std::move(buildTask), _leafCapa, newIndices);

                //reorder the geometries according to `newIndices`
                for(u32 currIndex : newIndices) {
                    _geometries.emplace_back(std::move(geometries[currIndex]));
                }

                std::cout << _ropes.size() << std::endl;
            }
    };
}