cmake_minimum_required(VERSION 3.20)

project(toolkit)
set(CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 20)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /utf-8 /Zc:preprocessor /bigobj" CACHE STRING "" FORCE)
endif()

add_subdirectory("toolkit/deps")

set(SRC_DIR_LIST "toolkit/loaders" "toolkit/opengl")
set(LIB_LIST )
file(GLOB SRC_FILES "toolkit/*.cpp")
foreach(DIR ${SRC_DIR_LIST})
  file(GLOB_RECURSE CPP_FILES "${DIR}/*.cpp")
  list(APPEND SRC_FILES ${CPP_FILES})
endforeach()

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC toolkit_deps)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
