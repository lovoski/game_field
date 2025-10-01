cmake_minimum_required(VERSION 3.20)
project(network_test)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(ONNX_MODEL_BASE_PATH "${PROJECT_SOURCE_DIR}/onnx")
configure_file("${PROJECT_SOURCE_DIR}/config.h.in" "${PROJECT_BINARY_DIR}/onnx_config.h")

# search for onnxruntime library from given path
find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h
  PATHS 
    $ENV{ONNXRUNTIME_ROOT}
    "D:/repo/onnxruntime-win-x64-1.17.3"
  PATH_SUFFIXES include
  NO_DEFAULT_PATH)

# search for onnxruntime library from given path
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  PATHS 
    $ENV{ONNXRUNTIME_ROOT}
    "D:/repo/onnxruntime-win-x64-1.17.3"
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime not found. Please set ONNXRUNTIME_ROOT.")
else()
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
  message(STATUS "Found ONNX Runtime headers: ${ONNXRUNTIME_INCLUDE_DIR}")
endif()

add_executable(img_onnx img.cpp)
target_include_directories(img_onnx PRIVATE ${PROJECT_BINARY_DIR} ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(img_onnx PRIVATE toolkit ${ONNXRUNTIME_LIBRARY})
