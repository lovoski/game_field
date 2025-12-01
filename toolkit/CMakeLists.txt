cmake_minimum_required(VERSION 3.20)

project(toolkit)
set(CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 20)

find_package(assimp CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED)
add_subdirectory("toolkit/deps/cnpy")
add_subdirectory("toolkit/deps/tinyfd")
add_subdirectory("toolkit/deps/ufbx")
add_subdirectory("toolkit/deps/sun_sky")
add_subdirectory("toolkit/deps/imgui")

set(
  SRC_DIR_LIST 
  "toolkit/loaders" 
  "toolkit/opengl" 
  "toolkit/anim" 
  "toolkit/sim" 
  "toolkit/bullet" 
  "toolkit/assets" 
  "toolkit/common"
)
set(LIB_LIST )
file(GLOB SRC_FILES "toolkit/*.cpp")
foreach(DIR ${SRC_DIR_LIST})
  file(GLOB_RECURSE CPP_FILES "${DIR}/*.cpp")
  list(APPEND SRC_FILES ${CPP_FILES})
endforeach()

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC 
  cnpy 
  imgui
  ZLIB::ZLIB 
  ufbx 
  tinyfd 
  glad::glad 
  glfw 
  spdlog::spdlog 
  imgui 
  sun_sky 
  assimp::assimp 
  CGAL::CGAL
  BulletSoftBody
  BulletDynamics
  BulletInverseDynamics
  BulletCollision
  Bullet3Common
  LinearMath
)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR} "toolkit/deps/headers")
if (MSVC)
  target_compile_options(${PROJECT_NAME} PUBLIC /Zc:preprocessor /bigobj /utf-8)
endif()