#include "kdTree.cuh"
#include "utils.cuh"
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <queue>

namespace gsy {

    __host__ std::pair<Axis, Float> choose_split_axis(
        const BuildTask& task, 
        BuildTask& subTask0, 
        BuildTask& subTask1
    ) {
        Axis optAxis = Axis::None;
        Float optPos = std::nanf("");
        u32 optElement = std::numeric_limits<u32>::max();
        std::unordered_set<u32> optVis;

        for(
            u8 currAxis = static_cast<u8>(Axis::X) ; 
            currAxis < static_cast<u8>(Axis::None) ; 
            ++currAxis
        ) {
            const std::vector<ObjPoint<Float, 1>>&currPoints = task.points[currAxis];
            u32 currLeft = 1;
            assert(task.points[currAxis].size() % 2 == 0);
            u32 currRight = task.points[currAxis].size() / 2;
            Float currSplitPos = average(currPoints[0].value, currPoints[1].value);
            std::unordered_set<u32> vis;
            vis.insert(currPoints[0].index);

            for(u32 i = 2 ; currLeft < currRight ; ++i) {
                currSplitPos = average(currPoints[i-1].value, currPoints[i].value);
                u32 currIndex = currPoints[i-1].index;
                if(vis.find(currIndex) != vis.end()) {
                    --currRight;
                } else {
                    vis.insert(currIndex);
                    ++currLeft;
                }
            }

            u32 currElement = currLeft + currRight;

            if(currElement < optElement) {
                optAxis = static_cast<Axis>(currAxis);
                optPos = currSplitPos;
                optElement = currElement;
                optVis = std::move(vis);
            } 
        }

        for(int i = 0 ; i < 3 ; ++i) {
            subTask0.points[i].clear();
            subTask1.points[i].clear();

            for(auto point : task.points[i]) {
                if(optVis.find(point.index) != optVis.end()) {
                    subTask0.points[i].push_back(point);
                } else {
                    subTask1.points[i].push_back(point);
                }
            }
        }

        return std::pair<Axis, float>{optAxis, optPos};
    }

    __host__ void first_find_neighbors(
        const u32 parentNeighbors[6], 
        u32 child0Neighbors[6], 
        u32 child1Neighbors[6], 
        Axis splitAxis, 
        u32 parentIndex
    ) {
        std::copy_n(parentNeighbors, 6, child0Neighbors);
        std::copy_n(parentNeighbors, 6, child1Neighbors);

        switch(splitAxis) {
            case Axis::X:
                child0Neighbors[1] = child1Neighbors[0] = parentIndex;
                break;
            case Axis::Y:
                child0Neighbors[3] = child1Neighbors[2] = parentIndex;
                break;
            case Axis::Z:
                child0Neighbors[5] = child1Neighbors[4] = parentIndex;
                break;
            default:
                std::cerr << "Invaild axis is given in first_find_neighbors" << std::endl;
                exit(1);
        } 
    }

    __host__ void second_find_neighbors(
        const u32 parentNeighbors[6], 
        u32 child0Neighbors[6], 
        u32 child1Neighbors[6], 
        Axis splitAxis, 
        u32 child0Index
    )  {
        std::copy_n(parentNeighbors, 6, child0Neighbors);
        std::copy_n(parentNeighbors, 6, child1Neighbors);

        switch(splitAxis) {
            case Axis::X:
                child0Neighbors[1] = child0Index + 1;
                child1Neighbors[0] = child0Index;
                break;
            case Axis::Y:
                child0Neighbors[3] = child0Index + 1;
                child1Neighbors[2] = child0Index;
                break;
            case Axis::Z:
                child0Neighbors[5] = child0Index + 1;
                child1Neighbors[4] = child0Index;
                break;
            default:
                std::cerr << "Invalid axis is given in second_find_neighbors" << std::endl;
                exit(1);
        }
    }

    __host__ void build(
        std::vector<Rope>& ropes, 
        BuildTask buildTask, 
        const u16 leafCapa, 
        std::vector<u32>& indices
    ) {
        ropes.clear();
        indices.clear();

        std::queue<BuildTask> unFinishedTasks;
        unFinishedTasks.emplace(std::move(buildTask));

        while(!unFinishedTasks.empty()) {
            BuildTask currBuildTask = std::move(unFinishedTasks.front());
            unFinishedTasks.pop();

            Rope newRope;

            assert(
                currBuildTask.points[0].size() == currBuildTask.points[1].size() &&
                currBuildTask.points[1].size() == currBuildTask.points[2].size() &&
                currBuildTask.points[0].size() % 2 == 0
            );

            const unsigned segmentNum = currBuildTask.points[0].size() / 2;

            if(segmentNum < leafCapa) {
                newRope.isLeaf = true;
                LeafNode& leafNode = newRope.leafNode;
                std::copy_n(&currBuildTask.neighbors[0], 6, &leafNode.neighbors[0]);
                leafNode.begin = indices.size();
                leafNode.size = segmentNum;

                std::unordered_set<unsigned int> vis;
                for(const auto&[_, index] : currBuildTask.points[0]) {
                    vis.insert(index);
                }

                for(auto i : vis) {
                    indices.emplace_back(i);
                }

            } else {
                newRope.isLeaf = false;
                InnerNode& innerNode = newRope.innerNode;
                std::unordered_set<unsigned> vis;
                Axis splitAxis = Axis::None;
                float splitPos = std::nanf("");
                std::array<BuildTask, 6> newBuildTasks;

                std::pair<Axis, float> firstSplitRes = choose_split_axis(currBuildTask, newBuildTasks[0], newBuildTasks[1]);
                splitAxis = firstSplitRes.first;
                splitPos  = firstSplitRes.second;
                first_find_neighbors(currBuildTask.neighbors, newBuildTasks[0].neighbors, newBuildTasks[1].neighbors, splitAxis, ropes.size());
                innerNode.splitPos[0] = splitPos;
                innerNode.split0 = splitAxis;
                
                std::pair<Axis, float> secSplitRes0 = choose_split_axis(newBuildTasks[0], newBuildTasks[2], newBuildTasks[3]);
                splitAxis = secSplitRes0.first;
                splitPos  = secSplitRes0.second;
                second_find_neighbors(newBuildTasks[0].neighbors, newBuildTasks[2].neighbors, newBuildTasks[3].neighbors, splitAxis, ropes.size() + 1 + unFinishedTasks.size());
                innerNode.splitPos[1] = splitPos;
                innerNode.split10 = splitAxis;

                std::pair<Axis, float> secSplitRes1 = choose_split_axis(newBuildTasks[1], newBuildTasks[4], newBuildTasks[5]);
                splitAxis = secSplitRes1.first;
                splitPos  = secSplitRes1.second;
                second_find_neighbors(newBuildTasks[1].neighbors, newBuildTasks[4].neighbors, newBuildTasks[5].neighbors, splitAxis, ropes.size() + 3 + unFinishedTasks.size());
                innerNode.splitPos[2] = splitPos;
                innerNode.split11 = splitAxis;

                for(int i = 0 ; i < 4 ; ++i) {
                    innerNode.children[i] = ropes.size() + 1 + unFinishedTasks.size();
                    unFinishedTasks.emplace(std::move(newBuildTasks[i + 2]));
                }
            }
            ropes.emplace_back(newRope);
        }
    }

}
