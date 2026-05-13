cmake_minimum_required(VERSION 3.20)

project(engine3d)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory("toolkit")

include(cmake/find_onnxrt.cmake)

function(copy_onnxruntime_runtime target_name)
  if(ONNXRUNTIME_RUNTIME_DLLS)
    add_custom_command(
      TARGET ${target_name}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${ONNXRUNTIME_RUNTIME_DLLS}
              $<TARGET_FILE_DIR:${target_name}>
      COMMAND_EXPAND_LISTS)
  endif()
endfunction()

add_executable(editor3d editor3d.cpp)
target_link_libraries(editor3d PRIVATE toolkit)
target_include_directories(editor3d PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  craft
  craft/main.cpp
  craft/craft.cpp
  craft/voxel.cpp
  craft/voxel_render.cpp
)
target_link_libraries(craft PRIVATE toolkit)
target_include_directories(craft PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  motion_matching_loco
  motion_matching/main_loco.cpp
  motion_matching/motion_matching.cpp
)
target_link_libraries(motion_matching_loco PRIVATE toolkit)
target_include_directories(motion_matching_loco PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  controller
  controller/main.cpp
  controller/controller.cpp
  controller/controller_render.cpp
)
target_link_libraries(controller PRIVATE toolkit)
target_include_directories(controller PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  camdmpp
  camdmpp/main.cpp
  camdmpp/camdmpp.cpp
  camdmpp/camdmpp_sys.cpp
  camdmpp/camdmpp_render.cpp
  camdmpp/camdmpp_terrain.cpp
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
if(ONNXRUNTIME_CUDA_PROVIDER_AVAILABLE)
  target_compile_definitions(camdmpp PRIVATE ONNXRUNTIME_USE_CUDA_EP=1)
endif()
copy_onnxruntime_runtime(camdmpp)

add_executable(
  animator
  animator/main.cpp
  animator/animator.cpp
  animator/animator_render.cpp
)
target_link_libraries(animator PRIVATE toolkit)
target_include_directories(animator PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})

add_executable(
  retarget
  retarget/main.cpp
  retarget/retarget_app.cpp
  retarget/retarget_app_render.cpp
  retarget/retarget_core.cpp
  retarget/retarget_io.cpp
)
target_link_libraries(retarget PRIVATE toolkit)
target_include_directories(retarget PRIVATE ${PROJECT_BINARY_DIR} ${PROJECT_SOURCE_DIR})