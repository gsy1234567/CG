#include "kdTree.cuh"

using namespace gsy;

int main(int argc, char *argv[]) {
    std::vector<Triangle> tris;
    load_obj<Triangle>("assets/stanford_bunny.obj", tris);
    return 0;
}