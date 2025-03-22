cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

enable_testing()

###
### Select target
###

option(BUILD_MOUSE_V2   "Build for MicroMouse (v2)" OFF)
option(BUILD_SIMULATION "Build for Desktop Simulation" OFF)
option(BUILD_TESTS      "Build for Desktop Testing" OFF)

if(NOT (BUILD_MOUSE_V2 OR BUILD_SIMULATION OR BUILD_TESTS))
  set(BUILD_MOUSE_V2 ON)
endif()

if(BUILD_SIMULATION OR BUILD_TESTS)
  set(BUILD_DESKTOP ON)
endif()

if(BUILD_MOUSE_V2 AND BUILD_DESKTOP)
  message(FATAL_ERROR "Multiple build platforms selected.")
endif()

if(BUILD_MOUSE_V2)
  message(STATUS "Build platform: STM32")
else()
  message(STATUS "Build platform: Desktop")
endif()

###
### Configure project settings
###

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Default to debug build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Project name
set(CMAKE_PROJECT_NAME firmware)

# ARM Toolchain
if(BUILD_MOUSE_V2)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/platform/mouse_v2/cmake")
  include(gcc-arm-none-eabi)
endif()

# Enable compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Languages
enable_language(C)
enable_language(CXX)

###
### Build core library
###

# Make core library
add_library(micromouse)

# Sources
file(GLOB_RECURSE LIB_SOURCES lib/*.c lib/*.cpp)
target_sources(micromouse PRIVATE ${LIB_SOURCES})

# Includes
target_include_directories(micromouse PUBLIC include)

# Enable all warnings
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  target_compile_options(micromouse PRIVATE -Wall -Wextra -Wpedantic)
endif()

###
### Build targets
###

if(BUILD_SIMULATION)
  add_subdirectory(platform/simulation)
  set_target_properties(simulation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

endif()

if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(BUILD_MOUSE_V2)
  add_subdirectory(platform/mouse_v2)
  set_target_properties(mouse-v2-firmware PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

