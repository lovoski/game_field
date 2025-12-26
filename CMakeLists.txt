cmake_minimum_required(VERSION 3.20)

project(engine_test)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory("toolkit")

# set(ONNX_MODEL_BASE_PATH "${PROJECT_SOURCE_DIR}/scripts/onnxruntime/models")
# configure_file("${PROJECT_SOURCE_DIR}/scripts/onnxruntime/config.h.in" "${PROJECT_BINARY_DIR}/onnx_model_configs.h")

# # search for onnxruntime library from given path
# find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h
#   PATHS 
#     $ENV{ONNXRUNTIME_ROOT}
#     "D:/repo/onnxruntime-win-x64-1.17.3"
#     "~/lib/onnxruntime-linux-x64-1.17.3"
#   PATH_SUFFIXES include
#   NO_DEFAULT_PATH)

# # search for onnxruntime library from given path
# find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
#   PATHS 
#     $ENV{ONNXRUNTIME_ROOT}
#     "D:/repo/onnxruntime-win-x64-1.17.3"
#     "~/lib/onnxruntime-linux-x64-1.17.3"
#   PATH_SUFFIXES lib
#   NO_DEFAULT_PATH)

# if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
#   message(FATAL_ERROR "ONNX Runtime not found. Please set ONNXRUNTIME_ROOT.")
# else()
#   message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
#   message(STATUS "Found ONNX Runtime headers: ${ONNXRUNTIME_INCLUDE_DIR}")
# endif()

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