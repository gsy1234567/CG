project(CUDA_VERSION)

file(GLOB CUDA_SRC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/*.cu)

list(REMOVE_ITEM ${CUDA_SRC_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/main.cu)

add_library(cuda_render STATIC ${CUDA_SRC_FILE})

target_include_directories(cuda_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../cuda_include)

target_link_libraries(cuda_render PUBLIC stb Eigen::Eigen tinyobjloader)

add_executable(cuda_main main.cu)

target_link_libraries(cuda_main PRIVATE cuda_render)
