cmake_minimum_required(VERSION 3.10)
project(render)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 ")

include_directories(/usr/local/include ./include)

add_executable(render main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(render ${OpenCV_LIBRARIES})
#target_compile_options(render PUBLIC -Wall -Wextra -pedantic)