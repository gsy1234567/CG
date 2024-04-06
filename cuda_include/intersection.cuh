#pragma once

#include "core.cuh"
#include "brdf.cuh"

namespace gsy {

    struct Interaction {
        enum struct Type : u8 {
            None, 
            Geometry, 
            Light
        } type = Type::None;

        Float t = -1, u = -1, v = -1;
        BRDF* brdf = nullptr;
    };

}