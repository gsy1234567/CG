#pragma once

#include "core.cuh"
#include <vector>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace gsy {
    struct ImageWriter {
        const vec2u resolution;
        std::vector<vec3f> pixels;

        __host__ explicit ImageWriter(const vec2u& resolution) : resolution(resolution) {
            pixels.resize(resolution.x() * resolution.y());
        }

        __host__ inline Float get_aspect_ratio() const {
            return static_cast<Float>(resolution.x()) / static_cast<Float>(resolution.y());
        }

        __host__ inline void set_pixel(u32 x, u32 y, const vec3f& color) {
            pixels[x + y * resolution.x()] = color;
        }
        
        __host__ void write(const std::string& filePath) const {
            std::vector<u8> data;
            data.reserve(resolution.x() * resolution.y() * 3);

            auto gammaCorrection = [](Float x) {
                if(x > static_cast<Float>(1)) x = static_cast<Float>(1);
                if(x < static_cast<Float>(0)) x = static_cast<Float>(0);
                return static_cast<u8>(pow<Float>(x, 0.454545455) * 255);
            };

            for(const auto& pixel : pixels) {
                data.push_back(gammaCorrection(pixel.x()));
                data.push_back(gammaCorrection(pixel.y()));
                data.push_back(gammaCorrection(pixel.z()));
            }

            stbi_flip_vertically_on_write(true);
            stbi_write_png(filePath.c_str(), resolution.x(), resolution.y(), 3,
                 data.data(), 0);
        }
    };
}

#undef STB_IMAGE_WRITE_IMPLEMENTATION