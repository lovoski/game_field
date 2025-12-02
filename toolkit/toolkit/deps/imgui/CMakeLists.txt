cmake_minimum_required(VERSION 3.20)

project(imgui)

find_package(glad CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)

file(GLOB IMGUI_SRC_FILES "*.cpp")
list(APPEND IMGUI_SRC_FILES
  "backends/imgui_impl_opengl3.cpp"
  "backends/imgui_impl_sdl2.cpp"
  "backends/imgui_impl_sdlrenderer2.cpp"
)

add_library(${PROJECT_NAME} ${IMGUI_SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC "backends" "${PROJECT_SOURCE_DIR}")
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE 
  glad::glad
  $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
  $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
)