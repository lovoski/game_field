cmake_minimum_required(VERSION 3.20)

project(engine_test)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory("toolkit")
add_subdirectory("manga_viewer")

include(cmake/find_onnxrt.cmake)

add_executable(engine3d main3d.cpp)
target_link_libraries(engine3d PRIVATE toolkit)
target_include_directories(engine3d PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(engine2d main2d.cpp)
target_link_libraries(engine2d PRIVATE toolkit)
target_include_directories(engine2d PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  craft
  craft/main.cpp
  craft/craft.cpp
)
target_link_libraries(craft PRIVATE toolkit)
target_include_directories(craft PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  motion_matching
  motion_matching/main.cpp
  motion_matching/motion_matching.cpp
)
target_link_libraries(motion_matching PRIVATE toolkit)
target_include_directories(motion_matching PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  camdmpp
  camdmpp/main.cpp
  camdmpp/camdmpp.cpp
  camdmpp/camdmpp_render.cpp
  camdmpp/diffusion.cpp
  camdmpp/onnx_utils.cpp
  camdmpp/ziggurat.cpp
)
target_link_libraries(camdmpp PRIVATE toolkit ${ONNXRUNTIME_LIBRARY})
target_include_directories(
  camdmpp
  PRIVATE
  ${PROJECT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}
  ${ONNXRUNTIME_INCLUDE_DIR}
)