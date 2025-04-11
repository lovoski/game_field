cmake_minimum_required(VERSION 3.20)

project(toolkit_deps)

add_subdirectory("cnpy")
add_subdirectory("tinyfd")
add_subdirectory("ufbx")
add_subdirectory("zlib_embed")
add_subdirectory("glad")
add_subdirectory("glfw")

file(GLOB IMGUI_SRC_FILES "imgui/*.cpp")
list(APPEND IMGUI_SRC_FILES
  "imgui/backends/imgui_impl_glfw.cpp"
  "imgui/backends/imgui_impl_opengl3.cpp"
)

add_library(${PROJECT_NAME} ${IMGUI_SRC_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC cnpy zlib ufbx tinyfd glad glfw)
target_include_directories(${PROJECT_NAME} PUBLIC "headers" "imgui" "imgui/backends")
