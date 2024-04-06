#pragma once

#include <Eigen/Dense>
#include <numeric>
#include <numbers>
#include <cstdint>

namespace gsy {
    
    #ifdef FLOAT_AS_DOUBLE
    using Float = double;
    #else
    using Float = float;
    #endif

    constexpr Float Inf = std::numeric_limits<Float>::max();
    constexpr Float Pi  = 3.141592653589793238462643383279502884;
    constexpr Float Eps = 1e-6;

    using vec2f = Eigen::Matrix<Float, 2, 1>;
    using vec3f = Eigen::Matrix<Float, 3, 1>;
    using u8    = std::uint8_t;
    using u32   = std::uint32_t;
    using u64   = std::uint64_t;
    using i32   = std::int32_t;
    using vec3u = Eigen::Matrix<u32, 3, 1>;
    using vec4u = Eigen::Matrix<u32, 4, 1>;

    struct Host {};
    struct Device {};
    struct Positive {};
    struct Negative {};
}



