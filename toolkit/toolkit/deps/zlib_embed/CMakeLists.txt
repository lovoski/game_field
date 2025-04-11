cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(zlib LANGUAGES C)

file(GLOB ZLIB_SOURCE_FILES "*.c")

add_library(${PROJECT_NAME} ${ZLIB_SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
