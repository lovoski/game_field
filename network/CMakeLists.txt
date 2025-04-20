cmake_minimum_required(VERSION 3.20)

project(network_test)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(ONNX_MODEL_BASE_PATH "${PROJECT_SOURCE_DIR}/onnx")
configure_file("${PROJECT_SOURCE_DIR}/config.h.in" "${PROJECT_BINARY_DIR}/onnx_config.h")

include_directories(
  "${PROJECT_BINARY_DIR}"
  "D:\\repo\\onnxruntime-win-x64-1.17.3\\include"
)
link_directories("D:\\repo\\onnxruntime-win-x64-1.17.3\\lib")

add_executable(img_onnx img.cpp)
target_link_libraries(img_onnx PRIVATE toolkit onnxruntime)
