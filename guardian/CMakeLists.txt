cmake_minimum_required(VERSION 3.20)
project(guardian CXX)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Built in the Steam Runtime 4 arm64 SDK (deploy.yml); `cmake --install` lays out the app as Steam installs it:
#   guardian.py calib.py NotoSans-Medium.ttf models/  bin/{guardian,libopenvr_api.so}
#   driver/{driver.vrdrivermanifest,bin/linuxarm64/driver_guardian.so}

# ncnn built with -DNCNN_VULKAN=ON (deploy.yml); OpenVR SDK headers + libopenvr_api.so (deploy.yml downloads them)
find_package(ncnn REQUIRED)
find_package(Vulkan REQUIRED)
find_package(Freetype REQUIRED)
set(OPENVR_SDK "${CMAKE_SOURCE_DIR}/../third_party/openvr" CACHE PATH "")
set(OPENVR_API "${OPENVR_SDK}/bin/linuxarm64/libopenvr_api.so")

# SteamVR driver: camserver.cpp on a thread inside vrserver. Static libstdc++: it's loaded into the host's vrserver,
# outside the runtime.
add_library(driver_guardian MODULE driver.cpp camserver.cpp)
target_include_directories(driver_guardian PRIVATE ${OPENVR_SDK}/headers)
target_link_libraries(driver_guardian PRIVATE pthread -static-libstdc++ -static-libgcc)
set_target_properties(driver_guardian PROPERTIES PREFIX "" CXX_VISIBILITY_PRESET hidden)

# OpenVR overlay over any VR app
add_executable(guardian guardian.cpp)
target_include_directories(guardian PRIVATE ${OPENVR_SDK}/headers)
target_link_libraries(guardian PRIVATE ncnn ${OPENVR_API} Freetype::Freetype Vulkan::Vulkan pthread)
set_target_properties(guardian PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS guardian DESTINATION bin)
install(FILES ${OPENVR_API} DESTINATION bin)
install(TARGETS driver_guardian DESTINATION driver/bin/linuxarm64)
install(FILES driver.vrdrivermanifest DESTINATION driver)
install(PROGRAMS guardian.py DESTINATION .)
install(FILES calib.py ${CMAKE_SOURCE_DIR}/../third_party/fonts/NotoSans-Medium.ttf DESTINATION .)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../models/ DESTINATION models FILES_MATCHING PATTERN "*_gray.ncnn.*")
