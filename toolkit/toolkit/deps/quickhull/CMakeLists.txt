cmake_minimum_required(VERSION 3.11)
project(quickhull)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

option(BUILD_QUICKHULL_TESTS "Build test cases for quickhull library" OFF)

add_library(${PROJECT_NAME} QuickHull.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})

if (${BUILD_QUICKHULL_TESTS})
  add_executable(quickhull_test test.cpp)
  target_link_libraries(quickhull_test PRIVATE quickhull)
endif()