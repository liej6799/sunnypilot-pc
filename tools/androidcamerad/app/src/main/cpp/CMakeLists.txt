cmake_minimum_required(VERSION 3.22.1)
project(android_camerad)

add_library(android_camerad SHARED android_camerad.cpp)

target_include_directories(android_camerad PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../openpilot/system/camerad)

target_compile_features(android_camerad PRIVATE cxx_std_20)
target_compile_options(android_camerad PRIVATE -Wall -Wextra -Werror)
target_link_options(android_camerad PRIVATE "-Wl,-z,max-page-size=16384")

target_link_libraries(android_camerad
  android
  camera2ndk
  log
  mediandk)
