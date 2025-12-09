cmake_minimum_required(VERSION 3.20)

project(toolkit)
set(CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 20)

find_package(assimp CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(Bullet CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2_image CONFIG REQUIRED)
find_package(SDL2_mixer CONFIG REQUIRED)
find_package(SDL2_ttf CONFIG REQUIRED)
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
  "toolkit/sdl2d"
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
  imgui 
  sun_sky 
  assimp::assimp 
  CGAL::CGAL
  $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
  $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
  $<IF:$<TARGET_EXISTS:SDL2_image::SDL2_image>,SDL2_image::SDL2_image,SDL2_image::SDL2_image-static>
  $<IF:$<TARGET_EXISTS:SDL2_mixer::SDL2_mixer>,SDL2_mixer::SDL2_mixer,SDL2_mixer::SDL2_mixer-static>
  $<IF:$<TARGET_EXISTS:SDL2_ttf::SDL2_ttf>,SDL2_ttf::SDL2_ttf,SDL2_ttf::SDL2_ttf-static>
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