cmake_minimum_required(VERSION 3.20)

project(tinyfd)

add_library(tinyfd tinyfiledialogs.c)
target_include_directories(tinyfd PUBLIC ${PROJECT_SOURCE_DIR})
