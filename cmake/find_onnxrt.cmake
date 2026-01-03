# search for onnxruntime library from given path
find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h
  PATHS 
    $ENV{ONNXRUNTIME_ROOT}
    "D:/repo/onnxruntime-win-x64-1.17.3"
    "~/lib/onnxruntime-linux-x64-1.17.3"
  PATH_SUFFIXES include
  NO_DEFAULT_PATH)

# search for onnxruntime library from given path
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  PATHS 
    $ENV{ONNXRUNTIME_ROOT}
    "D:/repo/onnxruntime-win-x64-1.17.3"
    "~/lib/onnxruntime-linux-x64-1.17.3"
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime not found. Please set ONNXRUNTIME_ROOT.")
else()
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
  message(STATUS "Found ONNX Runtime headers: ${ONNXRUNTIME_INCLUDE_DIR}")
endif()