cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(CNPY)

add_library(cnpy "cnpy.cpp")
target_link_libraries(cnpy PUBLIC zlib)
target_include_directories(cnpy PUBLIC ${PROJECT_SOURCE_DIR})
