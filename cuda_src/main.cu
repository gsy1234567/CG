#include "kdTree.cuh"
#include "obj_loader.cuh"
#include "intersection.cuh"

using namespace gsy;

int main(int argc, char *argv[]) {
    // std::vector<Triangle> geometries;
    // load_obj<Triangle>("assets/stanford_dragon.obj", geometries);
    // CompressedkdTree<Triangle, Host> kdTree(geometries, 16);

    vec3f o = {-1, -1, -1};
    vec3f d = {1, 1, 1};
    Ray ray {o, d};
    AABB aabb {vec3f{0,0,0}, vec3f{1,1,1}};
    float in, out;
    assert(intersect(ray, aabb, in, out));
    std::cout << "in = " << in << std::endl;
    std::cout << "out = " << out << std::endl;
    return 0;
}