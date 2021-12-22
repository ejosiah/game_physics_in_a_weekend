cmake_minimum_required(VERSION 3.19)
project(game_physics_in_one_weekend)

find_program(GLSLC glslc REQUIRED)
include(cmake/utilities.cmake)

find_package(Vulkan)

add_definitions(
        -DGLM_FORCE_RADIANS
        -DGLFW_INCLUDE_VULKAN
        -DGLM_FORCE_SWIZZLE
)

set(RESOURCES_DIR "${CMAKE_CURRENT_LIST_DIR}/resources")
set(GLSL_SOURCE_DIR "${RESOURCES_DIR}/shaders")
set(SPV_DIR "${RESOURCES_DIR}/spv")
compile_glsl_directory(SRC_DIR "${GLSL_SOURCE_DIR}" OUT_DIR "${SPV_DIR}" INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/data/shaders")


set(CMAKE_CXX_STANDARD 17)
message(STATUS "setting up conan")
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

include_directories(src)
include_directories(src/physics)

file(GLOB_RECURSE HPP_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.hpp)
file(GLOB_RECURSE CPP_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

message(STATUS "FILES: ${HPP_FILES}")

add_executable(game_physics_in_one_weekend ${HPP_FILES} ${CPP_FILES})
target_link_libraries(game_physics_in_one_weekend VulkanBase Vulkan::Vulkan ${CONAN_LIBS})