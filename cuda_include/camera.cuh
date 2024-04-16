#pragma once

#include "core.cuh"

namespace gsy {

    struct Camera {
        vec3f pos;
        vec3f right = vec3f::Constant(0); //x+, normalized
        vec3f front; //y+, normalized
        vec3f up;    //z+, normalized
        Float vFov;
        Float focalLen;

    };
}