cmake_minimum_required(VERSION 3.20)

project(manga_viewer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if (WIN32)
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SUBSYSTEM:CONSOLE")
endif()

include_directories("D:\\repo\\mupdf\\include")
link_directories("D:\\repo\\mupdf\\platform\\win32\\x64\\Release")

file(GLOB MANGA_VIEWER_SRC_FILES *.cpp)
add_executable(${PROJECT_NAME} ${MANGA_VIEWER_SRC_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE toolkit libmupdf)
