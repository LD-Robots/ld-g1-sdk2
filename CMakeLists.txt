cmake_minimum_required(VERSION 3.10)
project(g1_audio_control_sdk LANGUAGES CXX)

if(NOT DEFINED ENV{UNITREE_SDK2})
  message(FATAL_ERROR "Please export UNITREE_SDK2=/path/to/unitree_sdk2")
endif()

set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2}")
list(APPEND CMAKE_PREFIX_PATH "${UNITREE_SDK2_ROOT}/install/unitree_sdk2")

find_package(unitree_sdk2 REQUIRED)

set(WHISPER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(WHISPER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(WHISPER_BUILD_SERVER OFF CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/whisper.cpp EXCLUDE_FROM_ALL)

add_custom_target(rnnoise_build
  COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/thirdparty/rnnoise ./autogen.sh
  COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/thirdparty/rnnoise ./configure --disable-examples --disable-shared
  COMMAND ${CMAKE_COMMAND} -E chdir ${CMAKE_SOURCE_DIR}/thirdparty/rnnoise make
  BYPRODUCTS ${CMAKE_SOURCE_DIR}/thirdparty/rnnoise/.libs/librnnoise.a
)

add_library(rnnoise STATIC IMPORTED)
set_target_properties(rnnoise PROPERTIES
  IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/thirdparty/rnnoise/.libs/librnnoise.a
)
add_dependencies(rnnoise rnnoise_build)

add_subdirectory(audio_control)
add_subdirectory(led_control)
