cmake_minimum_required(VERSION 3.20)
project(yoloxr CXX)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ncnn built with -DNCNN_VULKAN=ON into third_party/ncnn-install (see README)
set(ncnn_DIR "${CMAKE_SOURCE_DIR}/../third_party/ncnn-install/lib/cmake/ncnn" CACHE PATH "")
find_package(ncnn REQUIRED)
find_package(Vulkan REQUIRED)
# SteamVR's shared loader (/usr/lib only has a static libopenxr_loader.a that doesn't link here)
find_library(OPENXR_LOADER NAMES libopenxr_loader.so PATHS /opt/steamvr/bin/linuxarm64 NO_DEFAULT_PATH REQUIRED)

foreach(stage vert frag)
  add_custom_command(
    OUTPUT box.${stage}.inc
    COMMAND glslc -mfmt=num -o box.${stage}.inc ${CMAKE_SOURCE_DIR}/box.${stage}
    DEPENDS box.${stage})
endforeach()

add_executable(yoloxr yoloxr.cpp box.vert.inc box.frag.inc)
target_include_directories(yoloxr PRIVATE ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR})
target_link_libraries(yoloxr PRIVATE ncnn Vulkan::Vulkan ${OPENXR_LOADER} pthread)
get_filename_component(OPENXR_DIR ${OPENXR_LOADER} DIRECTORY)
set_target_properties(yoloxr PROPERTIES BUILD_RPATH ${OPENXR_DIR})

add_executable(camserver camserver.cpp)

# OpenVR overlay over any VR app; Steam's arm64 libopenvr_api, the headers SteamVR ships with hellovr
find_library(OPENVR_API NAMES libopenvr_api.so PATHS $ENV{HOME}/.local/share/Steam/steamrtarm64 NO_DEFAULT_PATH REQUIRED)
add_executable(guardian guardian.cpp)
target_include_directories(guardian PRIVATE /opt/steamvr/tools/hellovr_vulkan_linux/src/openvr/headers)
find_package(Freetype REQUIRED)
target_link_libraries(guardian PRIVATE ncnn ${OPENVR_API} Freetype::Freetype Vulkan::Vulkan pthread)
get_filename_component(OPENVR_DIR ${OPENVR_API} DIRECTORY)
set_target_properties(guardian PROPERTIES BUILD_RPATH ${OPENVR_DIR})
