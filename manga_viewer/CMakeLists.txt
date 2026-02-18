cmake_minimum_required(VERSION 3.20)

project(manga_viewer LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if (WIN32)
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SUBSYSTEM:CONSOLE")
endif()

set(MUPDF_ROOT "D:/repo/mupdf" CACHE PATH "Local MuPDF root directory")
set(MUPDF_INCLUDE_DIR "${MUPDF_ROOT}/include" CACHE PATH "MuPDF include directory")
set(MUPDF_LIB_DIR_DEBUG "${MUPDF_ROOT}/platform/win32/x64/Debug" CACHE PATH "MuPDF Debug library directory")
set(MUPDF_LIB_DIR_RELEASE "${MUPDF_ROOT}/platform/win32/x64/Release" CACHE PATH "MuPDF Release library directory")

include_directories("${MUPDF_INCLUDE_DIR}")

find_package(spdlog CONFIG REQUIRED)

file(GLOB MANGA_VIEWER_SRC_FILES *.cpp)
add_executable(${PROJECT_NAME} ${MANGA_VIEWER_SRC_FILES})
target_link_directories(
  ${PROJECT_NAME}
  PRIVATE
  $<$<CONFIG:Debug>:${MUPDF_LIB_DIR_DEBUG}>
  $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:${MUPDF_LIB_DIR_RELEASE}>
)
target_link_libraries(${PROJECT_NAME} PRIVATE toolkit libmupdf spdlog::spdlog)
