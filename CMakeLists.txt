cmake_minimum_required(VERSION 3.16)
project(CS171-hw4 LANGUAGES C CXX CUDA)
set(CMAKE_CXX_STANDARD 20)
find_package(OpenMP)

add_subdirectory(libs)
add_subdirectory(src)
add_subdirectory(cuda_src)