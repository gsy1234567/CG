#include "kdTree.cuh"
#include "obj_loader.cuh"

using namespace gsy;

int main(int argc, char *argv[]) {
    std::vector<Triangle> geometries;
    load_obj<Triangle>("assets/stanford_bunny.obj", geometries);
    KDTree<Triangle, Host> hostKDTree {std::move(geometries), 16};
    hostKDTree.optimize_leafNode();
    return 0;
}