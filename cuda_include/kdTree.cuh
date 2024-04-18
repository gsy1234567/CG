#pragma once

#include "core.cuh"
#include "AABB.cuh"
#include "intersection.cuh"
#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <numeric>
#include <assert.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>


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
            AABB _aabb;

        /**
         * @brief Find the leafNode of the kdTree according to `lambdaIn`, `lambdaOut`.
         * @param[in] lambdaIn The least distance between the ray.ori and the AABB of this kdTree.
         * @param[in] ray The input ray.
         * @param[inout] rope As the input, it is the innerNode of this kdTree. As the output, it is the leafNode of this kdTree.
        */
        __host__ void down_traversal(
            Float lambdaIn, 
            const Ray& ray, 
            Rope& rope
        ) const {
            auto pEntry = ray(lambdaIn);

            while(!rope.isLeaf) {
                switch(rope.innerNode.split0) {
                    case Axis::X:
                        if(pEntry.x() <= rope.innerNode.splitPos[0]) goto Step1_0;
                        else goto Step1_1;
                    case Axis::Y:
                        if(pEntry.y() <= rope.innerNode.splitPos[0]) goto Step1_0;
                        else goto Step1_1;
                    case Axis::Z:
                        if(pEntry.z() <= rope.innerNode.splitPos[0]) goto Step1_0;
                        else goto Step1_1;
                    default:
                        std::cerr << "Invalid split0" << std::endl;
                        exit(1);
                }

                Step1_0:
                switch(rope.innerNode.split10) {
                    case Axis::X:
                        if(pEntry.x() <= rope.innerNode.splitPos[1]) goto Step2_0;
                        else goto Step2_1;
                    case Axis::Y:
                        if(pEntry.y() <= rope.innerNode.splitPos[1]) goto Step2_0;
                        else goto Step2_1;
                    case Axis::Z:
                        if(pEntry.y() <= rope.innerNode.splitPos[1]) goto Step2_0;
                        else goto Step2_1;
                    default:
                        std::cout << "Invalid split10" << std::endl;
                        exit(1);
                }

                Step1_1:
                switch(rope.innerNode.split10) {
                    case Axis::X:
                        if(pEntry.x() <= rope.innerNode.splitPos[2]) goto Step2_2;
                        else goto Step2_3;
                    case Axis::Y:
                        if(pEntry.y() <= rope.innerNode.splitPos[2]) goto Step2_2;
                        else goto Step2_3;
                    case Axis::Z:
                        if(pEntry.y() <= rope.innerNode.splitPos[2]) goto Step2_2;
                        else goto Step2_3;
                    default:
                        std::cout << "Invalid split11" << std::endl;
                        exit(1);
                }

                Step2_0:
                rope = _ropes[rope.innerNode.children[0]];
                continue;
                Step2_1:
                rope = _ropes[rope.innerNode.children[1]];
                continue;
                Step2_2:
                rope = _ropes[rope.innerNode.children[2]];
                continue;
                Step2_3:
                rope = _ropes[rope.innerNode.children[3]];
            }
        }

        public:
            __host__ CompressedkdTree<GeometryType, Host>::CompressedkdTree(std::vector<GeometryType> geometries, u16 leafCapa) : 
                _leafCapa(leafCapa) {

                for(auto geometry : geometries) {
                    _aabb.expand(geometry.get_aabb());
                }

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
            }
        
            __host__ bool intersect(Interaction& interaction, Ray ray) const {
                Rope currRope = _ropes[0];
                Float lambdaIn, lambdaOut;
                if(!ray_aabb_intersect(ray, _aabb, lambdaIn, lambdaOut))
                    return false;

                while(lambdaIn < lambdaOut) {
                    down_traversal(lambdaIn, ray, currRope);

                    
                }
                return true;
            }
    };

    template<typename T, typename Location>
    class KDTree;

    template<typename T>
    class KDTree<T, Host> {
        private:
            struct Index {
                i32 index;
                static constexpr i32 invalid = std::numeric_limits<i32>::min();
                __host__ Index() : index{invalid} {}
                __host__ Index(u32 idx, bool isInnerNode) : index{isInnerNode ? static_cast<i32>(idx) : -static_cast<i32>(idx)-1} {}
                __host__ bool is_leaf_node() const { 
                    assert(index != invalid);
                    return index < 0; 
                }
                __host__ bool is_valid() const { return index != invalid; }
                __host__ bool is_inner_node() const { return !is_leaf_node(); }
                __host__ u32 get_row_index() const { return is_leaf_node() ? -index-1 : index; }
            };

            struct InnerNode {
                Axis splitAxis;
                Float splitPos;
                std::array<Index, 2> children;
                __host__ InnerNode(Axis axis, Float pos, Index left, Index right) : 
                    splitAxis(axis), 
                    splitPos(pos), 
                    children{left, right} {}
    
                __host__ InnerNode() : splitAxis(Axis::None) {}
            };

            struct LeafNode {
                static constexpr u32 nil = static_cast<u32>(-1);
                std::array<Index, 6> neighbors;
                AABB aabb;
                u32 begin;
                u32 size;
            };

            struct BuildTask {
                AABB aabb;
                std::array<std::vector<ObjPoint<Float, 1>>, 3> points;
                std::array<Index, 6> neighbors;
                u32 parent;
                u32 which;
            };

            std::vector<InnerNode> _innerNodes;
            std::vector<LeafNode>  _leafNodes;
            std::vector<T>         _geometries;
            AABB                   _aabb;
            const u32              _leafCapa;

            __host__ static inline bool is_left(Direction dir) { return dir % 2 == 0; }
            __host__ static inline bool is_right(Direction dir) { return dir % 2 != 0; }

            __host__ inline void make_LeafNode(LeafNode& newLeafNode, std::vector<u32>& indices, const BuildTask& buildTask) {
                newLeafNode.neighbors = buildTask.neighbors;
                newLeafNode.aabb = buildTask.aabb;
                newLeafNode.begin = indices.size();
                std::unordered_set<u32> vis;
                for(auto objPoint : buildTask.points[0]) {
                    vis.insert(objPoint.index);
                }
                for(auto index : vis) {
                    indices.emplace_back(index);
                }
                newLeafNode.size = vis.size();
            }

            __host__ inline void make_InnerNode(
                InnerNode& newInnerNode, 
                BuildTask task, 
                BuildTask& subTask0, 
                BuildTask& subTask1, 
                u32& innerNodeCnt, 
                u32& leafNodeCnt
            ) {
                Axis optAxis = Axis::None;
                Float optPos = nan<Float>();
                u32 optElements = std::numeric_limits<u32>::max();
                std::unordered_map<u32, u8> optVis;
                for(
                    u8 currAxis = static_cast<u8>(Axis::X) ; 
                    currAxis < static_cast<u8>(Axis::None) ;
                    ++currAxis
                ) {
                    const std::vector<ObjPoint<Float, 1>>& currPoints = task.points[currAxis];
                    u32 currLeft = 1;
                    assert(task.points[currAxis].size() % 2 == 0);
                    u32 currRight = task.points[currAxis].size() / 2;
                    Float currSplitPos = average(currPoints[0].value, currPoints[1].value);
                    std::unordered_map<u32, u8> vis;
                    vis[currPoints[0].index] = 1;

                    for(u32 i = 2 ; currLeft < currRight ; ++i) {
                        currSplitPos = average(currPoints[i-1].value, currPoints[i].value);
                        u32 currIndex = currPoints[i-1].index;
                        auto cnt = vis[currIndex] += 1;
                        assert(cnt == 1 || cnt == 2);
                        if(cnt == 1) {
                            ++currLeft;
                        } else {
                            --currRight;
                        }
                    }

                    u32 currElements = currLeft + currRight;

                    if(currElements < optElements) {
                        optAxis = static_cast<Axis>(currAxis);
                        optPos = currSplitPos;
                        optElements = currElements;
                        optVis = std::move(vis);
                    }
                }

                //update the points of subTasks
                for(int i = 0 ; i < 3 ; ++i) {
                    subTask0.points[i].clear();
                    subTask1.points[i].clear();

                    for(auto point : task.points[i]) {
                        auto iter = optVis.find(point.index);
                        if(iter == optVis.end()) {
                            subTask1.points[i].push_back(point);
                        } else {
                            assert(iter->second == 1 || iter->second == 2);
                            if(iter->second == 1) {
                                subTask1.points[i].push_back(point);
                            }
                            subTask0.points[i].push_back(point);
                        }
                    }
                }

                //update the aabb of subTasks
                //update the neighbors of subTasks
                subTask0.aabb = subTask1.aabb = task.aabb;
                subTask0.neighbors = subTask1.neighbors = task.neighbors;

                assert(subTask0.points[0].size() % 2 == 0);
                bool isSubTask0Leaf = (subTask0.points[0].size() / 2) <= _leafCapa;
                bool isSubTask1Leaf = (subTask1.points[0].size() / 2) <= _leafCapa;
                assert(isSubTask0Leaf == isSubTask1Leaf);
                
                Index idx0 {isSubTask0Leaf ? leafNodeCnt++ : innerNodeCnt++, !isSubTask0Leaf };
                Index idx1 {isSubTask1Leaf ? leafNodeCnt++ : innerNodeCnt++, !isSubTask1Leaf };

                switch(optAxis) {
                    case Axis::X:
                        subTask0.aabb.max.x() = subTask1.aabb.min.x() = optPos;
                        subTask0.neighbors[Direction::right] = idx1;
                        subTask1.neighbors[Direction::left] = idx0;
                        break;
                    case Axis::Y:
                        subTask0.aabb.max.y() = subTask1.aabb.min.y() = optPos;
                        subTask0.neighbors[Direction::front] = idx1;
                        subTask1.neighbors[Direction::back] = idx0;
                        break;
                    case Axis::Z:
                        subTask0.aabb.max.z() = subTask1.aabb.min.z() = optPos;
                        subTask0.neighbors[Direction::top] = idx1;
                        subTask1.neighbors[Direction::bottom] = idx0;
                        break;
                }
                
                //update newInnerNode
                newInnerNode.splitPos = optPos;
                newInnerNode.splitAxis = optAxis;
                newInnerNode.children[0] = idx0;
                newInnerNode.children[1] = idx1;
            }
            

            void build(BuildTask rootBuildTask, std::vector<u32>& indices) {
                _innerNodes.clear();
                _leafNodes.clear();
                _geometries.clear();

                std::queue<BuildTask> unFinishedTasks;
                unFinishedTasks.emplace(std::move(rootBuildTask));
                u32 leafCnt = 0;
                u32 innerCnt = 1;

                while(!unFinishedTasks.empty()) {
                    BuildTask currBuildTask = std::move(unFinishedTasks.front());
                    unFinishedTasks.pop();

                    assert(
                        currBuildTask.points[0].size() == currBuildTask.points[1].size() &&
                        currBuildTask.points[1].size() == currBuildTask.points[2].size() && 
                        currBuildTask.points[0].size() % 2 == 0
                    );

                    const u32 segmentNum = currBuildTask.points[0].size() / 2;

                    if(segmentNum <= _leafCapa) {
                        if(currBuildTask.parent != std::numeric_limits<u32>::max())
                            _innerNodes[currBuildTask.parent].children[currBuildTask.which] = Index{static_cast<u32>(_leafNodes.size()), false};
                        _leafNodes.emplace_back();
                        make_LeafNode(_leafNodes.back(), indices, currBuildTask);
                    } else {
                        std::array<BuildTask, 2> newBuildTasks;
                        newBuildTasks[0].parent = _innerNodes.size();
                        newBuildTasks[1].parent = _innerNodes.size();
                        newBuildTasks[0].which = 0;
                        newBuildTasks[1].which = 1;
                        if(currBuildTask.parent != std::numeric_limits<u32>::max())
                            _innerNodes[currBuildTask.parent].children[currBuildTask.which] = Index{static_cast<u32>(_innerNodes.size()), true};
                        _innerNodes.emplace_back();
                        make_InnerNode(_innerNodes.back(), std::move(currBuildTask), newBuildTasks[0], newBuildTasks[1], innerCnt, leafCnt);
                        unFinishedTasks.emplace(std::move(newBuildTasks[0]));
                        unFinishedTasks.emplace(std::move(newBuildTasks[1]));
                    }
                }
            }

            __host__ inline Index get_root() const {
                return Index{0, _innerNodes.size() > 0};
            }

        public:
            KDTree(std::vector<T> geometries, u32 leafCapa) : _leafCapa(leafCapa) {
                std::cout << "Build kdTree Begin" << std::endl;
                std::cout << "# of geometries: " << geometries.size() << std::endl;
                BuildTask rootBuildTask;
                std::vector<u32> indices;

                for(u32 i = 0 ; i < geometries.size() ; ++i) {
                    rootBuildTask.aabb.expand(geometries[i].get_aabb());
                    rootBuildTask.points[0].emplace_back(geometries[i].x_min(), i);
                    rootBuildTask.points[0].emplace_back(geometries[i].x_max(), i);
                    rootBuildTask.points[1].emplace_back(geometries[i].y_min(), i);
                    rootBuildTask.points[1].emplace_back(geometries[i].y_max(), i);
                    rootBuildTask.points[2].emplace_back(geometries[i].z_min(), i);
                    rootBuildTask.points[2].emplace_back(geometries[i].z_max(), i);
                }

                _aabb = rootBuildTask.aabb;

                rootBuildTask.parent = std::numeric_limits<u32>::max();

                std::sort(rootBuildTask.points[0].begin(), rootBuildTask.points[0].end());
                std::sort(rootBuildTask.points[1].begin(), rootBuildTask.points[1].end());
                std::sort(rootBuildTask.points[2].begin(), rootBuildTask.points[2].end());

                build(std::move(rootBuildTask), indices);

                for(auto index : indices) {
                    _geometries.push_back(geometries[index]);
                }

                std::cout << "Build kdTree End" << std::endl;
                std::cout << "# of inner nodes: " << _innerNodes.size() << std::endl;
                std::cout << "# of leaf  nodes: " << _leafNodes.size() << std::endl;
            }

            void optimize_leafNode() {
                for(LeafNode& leafNode : _leafNodes) {
                    for(u8 dir = Direction::left ; dir != Direction::none ; ++dir) {
                        Index& idx = leafNode.neighbors[dir];
                        if(!idx.is_valid())
                            continue;
                        while(!idx.is_leaf_node()) {
                            InnerNode& innerNode = _innerNodes[idx.get_row_index()];
                            if(is_parallel(innerNode.splitAxis, static_cast<Direction>(dir))) {
                                idx = innerNode.children[leafNode.aabb.mid()[get_axis(static_cast<Direction>(dir))] >= innerNode.splitPos];
                            } else {
                                break;
                            }
                        }
                    }
                }
            }

            bool traversal(Ray ray, Interaction& interaction) const {
                if(!_aabb.intersect(ray)) {
                    return false;
                }

                Index idx = get_root();

                while(ray.tMin + Eps < ray.tMax) {
                    vec3f pEntry = ray(ray.tMin);

                    //down traversal
                    while(!idx.is_leaf_node()) {
                        const InnerNode& currInnerNode = _innerNodes[idx.get_row_index()];
                        idx = currInnerNode.children[pEntry[currInnerNode.splitAxis] >= currInnerNode.splitPos];
                    }

                    //At a leaf.
                    //check for intersection with contained triangles.
                    const LeafNode& currLeafNode = _leafNodes[idx.get_row_index()];
                    for(u32 geometryOffset = 0 ; geometryOffset < currLeafNode.size ; ++geometryOffset) {
                        _geometries[geometryOffset + currLeafNode.begin].intersect(interaction, ray);
                    }

                    if(interaction.type != Interaction::Type::None) {
                        return true;
                    }

                    auto dir = currLeafNode.aabb.ray_exit(ray);
                    
                    idx = currLeafNode.neighbors[dir];

                }
                return false;
            }
    };
}