cmake_minimum_required(VERSION 3.16)
project(rio_replay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Pull in the shared estimator library. Both the firmware and this binary
# build from the same lib/rio-lib/src/rio_eskf.cpp -- no source drift.
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../lib/rio-lib rio_lib_build)

add_executable(rio_replay
  src/replay.cpp
  src/log_reader.cpp
)

target_link_libraries(rio_replay PRIVATE rio::rio_eskf_cpp)

# Read the firmware's src/config.h so replay and firmware share one source
# of truth for aiding flags (BARO_AIDING_ENABLED, BARO_AIDING_DIFFERENTIAL,
# RADAR_AIDING_ENABLED, ...). Replay-only macros and firmware-only macros
# coexist there; the firmware-only ones (SD logging, USB print, pins) are
# simply unused by replay.
target_include_directories(rio_replay PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../../src)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rio_replay PRIVATE -Wall -Wextra -Wpedantic)
endif()
