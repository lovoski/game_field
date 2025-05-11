cmake_minimum_required(VERSION 3.20)

project(imgui)

file(GLOB IMGUI_SRC_FILES "*.cpp")
list(APPEND IMGUI_SRC_FILES
  "backends/imgui_impl_glfw.cpp"
  "backends/imgui_impl_opengl3.cpp"
)

add_library(${PROJECT_NAME} ${IMGUI_SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "backends" "${PROJECT_SOURCE_DIR}")
target_link_libraries(${PROJECT_NAME} PRIVATE glfw glad)