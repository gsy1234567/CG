#include "kdTree.cuh"
#include "obj_loader.cuh"

using namespace gsy;

int main(int argc, char *argv[]) {
    std::vector<Triangle> geometries;
    load_obj<Triangle>("assets/stanford_bunny.obj", geometries);
    KDTree<Triangle, Host> hostKDTree {std::move(geometries), 16};
    hostKDTree.optimize_leafNode();
    Ray ray{vec3f{0,0,0}, vec3f{0, 0.1, 0}};
    Interaction interaction;
    hostKDTree.traversal(ray, interaction);
    return 0;
}