#include "kdTree.cuh"
#include "obj_loader.cuh"
#include "camera.cuh"
#include "image_writer.cuh"
#include <chrono>
#include <omp.h>

using namespace gsy;

int main(int argc, char *argv[]) {
    std::vector<Triangle> geometries;
    load_obj<Triangle>("assets/stanford_bunny.obj", geometries);
    KDTree<Triangle, Host> hostKDTree {std::move(geometries), 16};
    hostKDTree.optimize_leafNode();
    Ray ray{vec3f{0,0,0}, vec3f{0, 0.1, 0}};

    const vec2u res {1024, 1024};
    Camera camera {vec3f{1, 1, 1}, to_radian<Float>(30), 1, res};
    camera.look_at(vec3f{-0.01678,0.110089,-0.0016}, vec3f{0,0,1});
    ImageWriter imageWriter {res};
    auto t_start = std::chrono::high_resolution_clock::now();

    omp_set_num_threads(16);
    std::cout << omp_get_num_threads() << std::endl;
    #pragma omp parallel for 
    for(auto y = 0U ; y < res.y() ; ++y) {
        for(auto x = 0U ; x < res.x() ; ++x) {
            Ray r = camera.generat_ray(static_cast<Float>(x), static_cast<Float>(y));
            Interaction interaction;
            bool intersect = hostKDTree.traversal(r, interaction);
            switch(interaction.type) {
                case Interaction::Type::Geometry:
                    imageWriter.set_pixel(x, y, vec3f{1, 0, 0});
                    break;
                case Interaction::Type::None:
                    imageWriter.set_pixel(x, y, vec3f{0,0,0});
                    break;
                case Interaction::Type::Light:
                    imageWriter.set_pixel(x, y, vec3f{1,1,1});
                    break;
            }
        }
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << "ms" << std::endl;
    imageWriter.write("cuda_main.png");
    return 0;
}