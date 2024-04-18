#pragma once

#include <Eigen/Dense>
#include <numeric>
#include <numbers>
#include <cstdint>
#include "math.h"

namespace gsy {
    
    #ifdef FLOAT_AS_DOUBLE
    using Float = double;
    #else
    using Float = float;
    #endif

    constexpr Float Inf = std::numeric_limits<Float>::max();
    constexpr Float Pi  = 3.141592653589793238462643383279502884;
    constexpr Float Eps = 1e-6;

    template<typename T>
    __host__ __device__ constexpr T inv_180() {
        return static_cast<T>(1) / static_cast<T>(180);
    }
    
    using vec2f = Eigen::Matrix<Float, 2, 1>;
    using vec3f = Eigen::Matrix<Float, 3, 1>;
    using u8    = std::uint8_t;
    using u16   = std::uint16_t;
    using u32   = std::uint32_t;
    using u64   = std::uint64_t;
    using i32   = std::int32_t;
    using vec2u = Eigen::Matrix<u32, 2, 1>;
    using vec3u = Eigen::Matrix<u32, 3, 1>;
    using vec4u = Eigen::Matrix<u32, 4, 1>;

    template<typename T, u8 Dim>
    using Point = Eigen::Matrix<T, Dim, 1>;

    using Point3f = Point<Float, 3>;
    using Point2f = Point<Float, 2>;
    using Point1f = Point<Float, 1>;

    template<typename T, u8 Dim>
    struct ObjPoint {
        Point<T, Dim> point;
        u32 index;
    };

    template<typename T>
    struct ObjPoint<T, 1> {
        T value;
        u32 index;
        ObjPoint(T v, u32 i) : value{v}, index{i} {}
    };

    template<typename T>
    inline bool operator<(const ObjPoint<T, 1>& p0, const ObjPoint<T, 1>& p1) {
        return p0.value < p1.value;
    }

    template<typename T>
    inline bool operator==(const ObjPoint<T, 1>& p0, const ObjPoint<T, 1>& p1) {
        return p0.value == p1.value; 
    }

    struct Host {};
    struct Device {};
    struct Positive {};
    struct Negative {};

    enum Axis : u8 {
        X    = 0b00, 
        Y    = 0b01, 
        Z    = 0b10, 
        None = 0b11
    };

    enum Direction : u8 {
        left = 0, 
        right = 1, 
        back = 2, 
        front = 3,
        bottom = 4, 
        top = 5, 
        none = 6
    };

    //left, right are parallel to X
    //back, front are parallel to Y
    //bottom, top are parallel to Z
    __host__ __device__ inline bool is_parallel(Axis axis, Direction dir) {
        return (static_cast<u8>(dir) >> 1) == static_cast<u8>(axis); 
    }

    __host__ __device__ inline Axis get_axis(Direction dir) {
        return static_cast<Axis>(static_cast<u8>(dir) >> 1);
    }

    template<typename T>
    inline T nan();

    template<>
    inline float nan<float>() {
        return std::nanf("");
    }

    template<>
    inline double nan<double>() {
        return std::nan("");
    }

    template<typename T>
    inline __host__ __device__ T arctan(T x);

    template<>
    inline __host__ __device__ float arctan<float>(float x) {
        return ::atanf(x);
    }

    template<>
    inline __host__ __device__ double arctan<double>(double x) {
        return ::atan(x);
    }

    template<typename T>
    inline __host__ __device__ T pow(T base, T exp);

    template<>
    inline __host__ __device__ float pow(float base, float exp) {
        return ::powf(base, exp);
    }

    template<>
    inline __host__ __device__ double pow(double base, double exp) {
        return ::pow(base, exp);
    }

    template<typename T>
    inline __host__ __device__ T to_radian(T angle) {
        return angle * inv_180<T>() * static_cast<T>(3.141592653589793238462643383279502884);
    }



}



