cmake_minimum_required(VERSION 3.20)
project(guardian CXX)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ncnn built with -DNCNN_VULKAN=ON into third_party/ncnn-install (see README)
set(ncnn_DIR "${CMAKE_SOURCE_DIR}/../third_party/ncnn-install/lib/cmake/ncnn" CACHE PATH "")
find_package(ncnn REQUIRED)
find_package(Vulkan REQUIRED)

add_executable(camserver camserver.cpp)

# OpenVR overlay over any VR app; Steam's arm64 libopenvr_api, the headers SteamVR ships with hellovr
find_library(OPENVR_API NAMES libopenvr_api.so PATHS $ENV{HOME}/.local/share/Steam/steamrtarm64 NO_DEFAULT_PATH REQUIRED)
add_executable(guardian guardian.cpp)
target_include_directories(guardian PRIVATE /opt/steamvr/tools/hellovr_vulkan_linux/src/openvr/headers)
find_package(Freetype REQUIRED)
target_link_libraries(guardian PRIVATE ncnn ${OPENVR_API} Freetype::Freetype Vulkan::Vulkan pthread)
get_filename_component(OPENVR_DIR ${OPENVR_API} DIRECTORY)
set_target_properties(guardian PROPERTIES BUILD_RPATH ${OPENVR_DIR})
