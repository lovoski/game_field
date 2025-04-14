cmake_minimum_required(VERSION 3.20)

project(manga_viewer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories("D:\\Code\\mupdf\\include")
link_directories("D:\\Code\\mupdf\\platform\\win32\\x64\\Debug")

file(GLOB MANGA_VIEWER_SRC_FILES *.cpp)
add_executable(${PROJECT_NAME} ${MANGA_VIEWER_SRC_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE toolkit libmupdf)
