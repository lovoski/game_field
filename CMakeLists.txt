cmake_minimum_required(VERSION 3.20)

project(engine_test)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory("toolkit")
add_subdirectory("onnx_neural")
# add_subdirectory("manga_viewer")

file(GLOB_RECURSE SCRIPTS_SRC "scripts/*.cpp")

add_executable(engine main.cpp ${SCRIPTS_SRC})
target_link_libraries(engine PRIVATE toolkit)