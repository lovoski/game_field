cmake_minimum_required(VERSION 3.20)

project(sun_sky)

add_library(${PROJECT_NAME} SunSky.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
