#pragma once

#include "core.cuh"
#include "geometry.cuh"
#include <vector>
#include <iostream>
#include <string>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

namespace gsy {

    template<typename T>
    __host__ bool load_obj(const std::string& path, std::vector<T>& geometries);

    template<>
    __host__ bool load_obj<Triangle>(const std::string& path, std::vector<Triangle>& tris) {
        std::cout << "-- Loading Model --" << path << std::endl;

        tinyobj::ObjReaderConfig readerConfig;
        readerConfig.mtl_search_path = "./";

        tinyobj::ObjReader reader;
        std::string input = path;
        for(int dep = 0 ; dep != 5 ; ++dep) {
            if(!reader.ParseFromFile(input, readerConfig)) {
                input = "../" + input;
                if(dep < 4) {
                    continue;
                }
                if(!reader.Error().empty()) {
                    std::cerr << "TinyObjReader: " << reader.Error() << std::endl;
                }
                exit(1);
            }
        }

        if(!reader.Warning().empty()) {
            std::cerr << "TinyObjReader: " << reader.Warning() << std::endl;
        }

        auto& attrib = reader.GetAttrib();
        auto& shapes = reader.GetShapes();
        auto& materials = reader.GetMaterials();

        std::vector<vec3f> vertices(attrib.vertices.size() / 3);
        std::vector<vec2f> texCoords(attrib.vertices.size() / 3);
        std::vector<vec3f> normals(attrib.vertices.size() / 3);
        std::vector<std::size_t> index;

        for(auto i = 0ULL ; i < attrib.vertices.size() / 3; ++i) {
            vertices[i] = vec3f{attrib.vertices[3*i], attrib.vertices[3*i + 1], attrib.vertices[3 * i + 2]};
        }
        for(auto s = 0ULL ; s < shapes.size() ; ++s) {
            auto index_offset = 0ULL;
            for(auto f = 0 ; f < shapes[s].mesh.num_face_vertices.size() ; ++f) {
                std::size_t fv = shapes[s].mesh.num_face_vertices[f];

                for(auto v = 0ULL ; v < fv ; ++v) {
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                    index.push_back(idx.vertex_index);
                    if(!attrib.texcoords.empty()) {
                        texCoords[idx.vertex_index] = 
                            vec2f{
                                attrib.texcoords[idx.texcoord_index * 2], 
                                attrib.texcoords[idx.texcoord_index * 2 + 1]
                            };
                    }
                    normals[idx.vertex_index] = 
                        vec3f{
                            attrib.normals[idx.normal_index * 3],
                            attrib.normals[idx.normal_index * 3 + 1], 
                            attrib.normals[idx.normal_index * 3 + 2]
                        }.normalized();
                }
                index_offset += fv;
            }
        }

        assert(index.size() % 3 == 0);
        for(auto i = 0ULL ; i < index.size() ; i += 3) {
            tris.emplace_back(vertices[index[i]], vertices[index[i+1]], vertices[index[i+2]], normals[index[i]], normals[index[i+1]], normals[index[i+2]]);
        }

        std::cout << "Number of vertices: " << attrib.vertices.size() << std::endl;
        std::cout << "Number of faces: " << index.size() / 3 << std::endl;
        return true;
    }
}

#undef TINYOBJLOADER_IMPLEMENTATION

