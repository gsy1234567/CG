#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <assert.h>
#include <utility>
#include <array>
#include <algorithm>

enum struct Axis : unsigned char {
    X = 0b00, 
    Y = 0b01, 
    Z = 0b10, 
    None = 0b11
};

#pragma pack (1)
struct LeafNode {
    unsigned int neighbors[6];
    unsigned int begin;
    unsigned short size;
};
#pragma pack ()

#pragma pack (1)
struct InnerNode {
    float splitPos[3];
    unsigned int children[4];
    Axis split0 : 2;
    Axis split10 : 2;
    Axis split11 : 2;
};
#pragma pack ()

#pragma pack (1)
struct Rope {
    union {
        LeafNode leafNode;
        InnerNode innerNode;
    };
    unsigned short isLeaf;
};
#pragma pack ()

template<typename T>
struct Point {
    T value;
    unsigned int index;
    inline bool operator<(const Point& other) const {
        return value < other.value;
    }
    inline bool operator==(const Point& other) const {
        return value == other.value;
    }
};

struct BuildTask {
    std::array<std::vector<Point<float>>, 3> points;
    unsigned int neighbors[6];
};

/**
 * @param buildTask
 * we suppose buildTask.points[0:2] are sorted in ascending order
 * @param[out] optVis
 * @param[out] task0
 * @param[out] task1
*/
std::pair<Axis, float> choose_split_axis(
    const BuildTask& buildTask, 
    BuildTask& task0,
    BuildTask& task1
) {
    Axis optAxis = Axis::None;
    float optPos = std::nanf("");
    unsigned optElement = std::numeric_limits<unsigned>::max();
    std::unordered_set<unsigned> optVis;

    for(unsigned char currAxis = static_cast<unsigned char>(Axis::X) ; 
        currAxis < static_cast<unsigned char>(Axis::None) ; 
        ++currAxis) {
        const std::vector<Point<float>>&currPoints = buildTask.points[currAxis];
        unsigned currLeft = 1;
        assert(buildTask.points[currAxis].size() % 2 == 0);
        unsigned currRight = buildTask.points[currAxis].size() / 2;
        float currSplitPos = (currPoints[0].value + currPoints[1].value) / 2.f;
        std::unordered_set<unsigned> vis;
        vis.insert(currPoints[0].index);

        for(unsigned i = 2 ; currLeft < currRight ; ++i) {
            currSplitPos = (currPoints[i-1].value + currPoints[i].value) / 2.f;
            unsigned currIndex = currPoints[i-1].index;
            if(vis.find(currIndex) != vis.end()) {
                --currRight;
            } else {
                vis.insert(currIndex);
                ++currLeft;
            }
        }

        unsigned currElement = currLeft + currRight;

        if(currElement < optElement) {
            optAxis = static_cast<Axis>(currAxis);
            optPos = currSplitPos;
            optElement = currElement;
            optVis = std::move(vis);
        } 
    }

    for(int i = 0 ; i < 3 ; ++i) {
        task0.points[i].clear();
        task1.points[i].clear();

        for(auto point : buildTask.points[i]) {
            if(optVis.find(point.index) != optVis.end()) {
                task0.points[i].push_back(point);
            } else {
                task1.points[i].push_back(point);
            }
        }
    }

    return std::pair<Axis, float>{optAxis, optPos};
}

void first_find_neighbors(
    const unsigned parentNeighbors[6], 
    unsigned child0Neighbors[6], 
    unsigned child1Neighbors[6], 
    Axis splitAxis, 
    unsigned parentIndex
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

void second_find_neighbors(
    const unsigned parentNeighbors[6], 
    unsigned child0Neighbors[6], 
    unsigned child1Neighbors[6], 
    Axis splitAxis, 
    unsigned child0Index
) {
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

void build( std::vector<Rope>& ropes, 
            BuildTask buildTask, 
            const unsigned short leafCapa, 
            std::vector<unsigned int>& indices ) {

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

void make_test_point_3d(float x, float y, float z, unsigned index, 
    std::vector<Point<float>>& xs, 
    std::vector<Point<float>>& ys, 
    std::vector<Point<float>>& zs) {
    xs.emplace_back(x, index);
    ys.emplace_back(y, index);
    zs.emplace_back(z, index);
}

int main() {
    BuildTask buildTask;
    std::fill_n(buildTask.neighbors, 6, static_cast<unsigned>(-1));
    std::vector<Point<float>>& xs = buildTask.points[0];
    std::vector<Point<float>>& ys = buildTask.points[1];
    std::vector<Point<float>>& zs = buildTask.points[2];
    make_test_point_3d(1, 1, 0, 0, xs, ys, zs);
    make_test_point_3d(2, 2, 0, 0, xs, ys, zs);
    make_test_point_3d(1, -1, 0, 1, xs, ys, zs);
    make_test_point_3d(2, -2, 0, 1, xs, ys, zs);
    make_test_point_3d(-1, -1, 0, 2, xs, ys, zs);
    make_test_point_3d(-2, -2, 0, 2, xs, ys, zs);
    make_test_point_3d(-1, 1, 0, 3, xs, ys, zs);
    make_test_point_3d(-2, 2, 0, 3, xs, ys, zs);
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());
    std::vector<Rope> ropes;
    std::vector<unsigned> indices;
    build(ropes, std::move(buildTask), 3, indices);
    return 0;
}