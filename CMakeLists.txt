cmake_minimum_required(VERSION 3.20)

project(vk_test)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory("toolkit")
# add_subdirectory("network")
# add_subdirectory("manga_viewer")
add_subdirectory("examples")

file(GLOB_RECURSE "scripts/*.cpp" SCRIPTS_SRC)

add_executable(test_tk test_tk.cpp)
target_link_libraries(test_tk PRIVATE toolkit)

add_executable(test_gl test_gl.cpp ${SCRIPTS_SRC})
target_link_libraries(test_gl PRIVATE toolkit)

add_executable(test_cmd test_cmd.cpp)
target_link_libraries(test_cmd PRIVATE toolkit)