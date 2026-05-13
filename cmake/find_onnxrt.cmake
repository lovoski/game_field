set(_onnxruntime_default_root "D:/repo/onnxruntime-win-x64-gpu-1.23.0")
if(NOT EXISTS "${_onnxruntime_default_root}")
  set(_onnxruntime_default_root "D:/repo/onnxruntime-win-x64-1.24.4")
endif()

set(ONNXRUNTIME_ROOT
    "${_onnxruntime_default_root}"
    CACHE PATH "Path to the ONNX Runtime package root")

# Re-evaluate the resolved package on each configure so switching roots does not
# require a manual cache clear.
unset(ONNXRUNTIME_INCLUDE_DIR CACHE)
unset(ONNXRUNTIME_LIBRARY CACHE)

find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h
  PATHS
    ${ONNXRUNTIME_ROOT}
    $ENV{ONNXRUNTIME_ROOT}
  PATH_SUFFIXES include
  NO_DEFAULT_PATH)

find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  PATHS
    ${ONNXRUNTIME_ROOT}
    $ENV{ONNXRUNTIME_ROOT}
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH)

set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib")
set(ONNXRUNTIME_RUNTIME_DLLS
    "${ONNXRUNTIME_LIB_DIR}/onnxruntime.dll"
    "${ONNXRUNTIME_LIB_DIR}/onnxruntime_providers_shared.dll")

set(ONNXRUNTIME_CUDA_PROVIDER_AVAILABLE FALSE)
if(EXISTS "${ONNXRUNTIME_LIB_DIR}/onnxruntime_providers_cuda.dll")
  list(APPEND ONNXRUNTIME_RUNTIME_DLLS
       "${ONNXRUNTIME_LIB_DIR}/onnxruntime_providers_cuda.dll")
  set(ONNXRUNTIME_CUDA_PROVIDER_AVAILABLE TRUE)
endif()

list(FILTER ONNXRUNTIME_RUNTIME_DLLS INCLUDE REGEX ".*\\.dll$")

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime not found. Please set ONNXRUNTIME_ROOT.")
else()
  message(STATUS "Found ONNX Runtime root: ${ONNXRUNTIME_ROOT}")
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARY}")
  message(STATUS "Found ONNX Runtime headers: ${ONNXRUNTIME_INCLUDE_DIR}")
  if(ONNXRUNTIME_CUDA_PROVIDER_AVAILABLE)
    message(STATUS "ONNX Runtime CUDA provider detected")
  else()
    message(STATUS "ONNX Runtime CUDA provider not found; camdmpp will fall back to CPU execution")
  endif()
endif()