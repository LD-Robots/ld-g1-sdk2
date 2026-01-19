cmake_minimum_required(VERSION 3.10)
project(g1_audio_control_sdk LANGUAGES CXX)

if(NOT DEFINED ENV{UNITREE_SDK2})
  message(FATAL_ERROR "Please export UNITREE_SDK2=/path/to/unitree_sdk2")
endif()

set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2}")
list(APPEND CMAKE_PREFIX_PATH "${UNITREE_SDK2_ROOT}/install/unitree_sdk2")

find_package(unitree_sdk2 REQUIRED)

add_subdirectory(audio_control)
add_subdirectory(led_control)
