#include "kdTree.cuh"

namespace gsy {

    template<u32 LeafCapa>
    __host__ kdTree<8, LeafCapa, Host>::kdTree(const std::vector<Geometry>& geometries) {

        std::queue<BuildTask> buildTasks;

        AABB aabb;
        std::for_each(geometries.begin(), geometries.end(), [&aabb](const Geometry& x) -> void {
            aabb.expand(x.get_aabb());
        });
        _aabb = aabb;
        buildTasks.emplace(std::move(BuildTask(std::move(aabb), std::move(geometries))));

        while(!buildTasks.empty()) {
            BuildTask currTask = std::move(buildTasks.front());
            buildTasks.pop();

            Rope<8> currNode;

            if(currTask.geometries.size() <= LeafCapa) {
                currNode.neighbors = currTask.neighbors;
                currNode.set_leaf_rope();
                currNode.begin = _ropes.size();
                currNode.end   = _ropes.size() + currTask.geometries.size();
                _geometries.insert(_geometries.end(), currTask.geometries.begin(), currTask.geometries.end());
            } else {
                std::array<BuildTask, 8> newBuildTasks;
                //There may be better split strategies.
                currNode.xSplit = average(currTask.aabb.min.x(), currTask.aabb.max.x());
                currNode.ySplit = average(currTask.aabb.min.y(), currTask.aabb.max.y());
                currNode.zSplit = average(currTask.aabb.min.z(), currTask.aabb.max.z());

                u32 currBaseIdx = _ropes.size() + buildTasks.size();

                newBuildTasks[0].aabb = currTask.aabb.get_eighth<Negative, Negative, Negative>();
                newBuildTasks[0].neighbors[0] = currTask.neighbors[0];
                newBuildTasks[0].neighbors[1] = currBaseIdx + 1;
                newBuildTasks[0].neighbors[2] = currTask.neighbors[2];
                newBuildTasks[0].neighbors[3] = currBaseIdx + 2;
                newBuildTasks[0].neighbors[4] = currTask.neighbors[4];
                newBuildTasks[0].neighbors[5] = currBaseIdx + 4;
                newBuildTasks[0].parent = _ropes.size();
                newBuildTasks[0].which = 0;
                
                newBuildTasks[1].aabb = currTask.aabb.get_eighth<Positive, Negative, Negative>();
                newBuildTasks[1].neighbors[0] = currBaseIdx;
                newBuildTasks[1].neighbors[1] = currTask.neighbors[1];
                newBuildTasks[1].neighbors[2] = currTask.neighbors[2];
                newBuildTasks[1].neighbors[3] = currBaseIdx + 3;
                newBuildTasks[1].neighbors[4] = currTask.neighbors[4];
                newBuildTasks[1].neighbors[5] = currBaseIdx + 4;
                newBuildTasks[1].parent = _ropes.size();
                newBuildTasks[1].which = 1;

                newBuildTasks[2].aabb = currTask.aabb.get_eighth<Negative, Positive, Negative>();
                newBuildTasks[2].neighbors[0] = currTask.neighbors[0];
                newBuildTasks[2].neighbors[1] = currBaseIdx + 3;
                newBuildTasks[2].neighbors[2] = currBaseIdx;
                newBuildTasks[2].neighbors[3] = currTask.neighbors[3];
                newBuildTasks[2].neighbors[4] = currTask.neighbors[4];
                newBuildTasks[2].neighbors[5] = currBaseIdx + 6;
                newBuildTasks[2].parent = _ropes.size();
                newBuildTasks[2].which = 2;

                newBuildTasks[3].aabb = currTask.aabb.get_eighth<Positive, Positive, Negative>();
                newBuildTasks[3].neighbors[0] = currBaseIdx + 2;
                newBuildTasks[3].neighbors[1] = currTask.neighbors[1];
                newBuildTasks[3].neighbors[2] = currBaseIdx + 1;
                newBuildTasks[3].neighbors[3] = currTask.neighbors[3];
                newBuildTasks[3].neighbors[4] = currTask.neighbors[4];
                newBuildTasks[3].neighbors[5] = currBaseIdx + 7;
                newBuildTasks[3].parent = _ropes.size();
                newBuildTasks[3].which = 3;

                newBuildTasks[4].aabb = currTask.aabb.get_eighth<Negative, Negative, Positive>();
                newBuildTasks[4].neighbors[0] = currTask.neighbors[0];
                newBuildTasks[4].neighbors[1] = currBaseIdx + 5;
                newBuildTasks[4].neighbors[2] = currTask.neighbors[2];
                newBuildTasks[4].neighbors[3] = currBaseIdx + 6;
                newBuildTasks[4].neighbors[4] = currBaseIdx;
                newBuildTasks[4].neighbors[5] = currTask.neighbors[5];
                newBuildTasks[4].parent = _ropes.size();
                newBuildTasks[4].which = 4;

                newBuildTasks[5].aabb = currTask.aabb.get_eighth<Positive, Negative, Positive>();
                newBuildTasks[5].neighbors[0] = currBaseIdx + 4;
                newBuildTasks[5].neighbors[1] = currTask.neighbors[1];
                newBuildTasks[5].neighbors[2] = currTask.neighbors[2];
                newBuildTasks[5].neighbors[3] = currBaseIdx + 7;
                newBuildTasks[5].neighbors[4] = currBaseIdx + 1;
                newBuildTasks[5].neighbors[5] = currTask.neighbors[5];
                newBuildTasks[5].parent = _ropes.size();
                newBuildTasks[5].which = 5;

                newBuildTasks[6].aabb = currTask.aabb.get_eighth<Negative, Positive, Positive>();
                newBuildTasks[6].neighbors[0] = currTask.neighbors[0];
                newBuildTasks[6].neighbors[1] = currBaseIdx + 7;
                newBuildTasks[6].neighbors[2] = currBaseIdx + 4;
                newBuildTasks[6].neighbors[3] = currTask.neighbors[3];
                newBuildTasks[6].neighbors[4] = currBaseIdx + 2;
                newBuildTasks[6].neighbors[5] = currTask.neighbors[5];
                newBuildTasks[6].parent = _ropes.size();
                newBuildTasks[6].which = 6;

                newBuildTasks[7].aabb = currTask.aabb.get_eighth<Positive, Positive, Positive>();
                newBuildTasks[7].neighbors[0] = currBaseIdx + 6;
                newBuildTasks[7].neighbors[1] = currTask.neighbors[1];
                newBuildTasks[7].neighbors[2] = currBaseIdx + 5;
                newBuildTasks[7].neighbors[3] = currTask.neighbors[3];
                newBuildTasks[7].neighbors[4] = currBaseIdx + 3;
                newBuildTasks[7].neighbors[5] = currTask.neighbors[5];
                newBuildTasks[7].parent = _ropes.size();
                newBuildTasks[7].which = 7;

                std::for_each(currTask.geometries.begin(), currTask.geometries.end(), [&newBuildTasks,& currTask](const Geometry& currGeometry){
                    std::for_each(newBuildTasks.begin(), newBuildTasks.end(), [& currGeometry](BuildTask& currNewBuildTask){
                        if(currGeometry.get_aabb().has_intersection(currNewBuildTask.aabb)) {
                            currNewBuildTask.geometries.push_back(currGeometry);
                        }
                    });
                });

                std::for_each(newBuildTasks.begin(), newBuildTasks.end(), [&buildTasks](BuildTask& currNewBuildTask)->void{
                    buildTasks.emplace(std::move(currNewBuildTask));
                });
            }
            if(currTask.parent != BuildTask::invalid) {
                _ropes[currTask.parent].children[currTask.which] = _ropes.size();
            }
            _ropes.push_back(currNode);
        }
    }

}