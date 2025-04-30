cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

###
### Configure project settings
###

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Default to debug build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  list(APPEND CMAKE_CXX_FLAGS "-g -O0")
  list(APPEND CMAKE_C_FLAGS "-g -O0")
else()
  list(APPEND CMAKE_CXX_FLAGS "-O3")
  list(APPEND CMAKE_C_FLAGS "-O3")
endif()

# Project name
set(CMAKE_PROJECT_NAME micromouse-cli)

# Enable compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Languages
enable_language(C)
enable_language(CXX)

###
### Build executable
###

# Make executable
add_executable(${CMAKE_PROJECT_NAME})

# Sources
file(GLOB_RECURSE SOURCES src/*.c src/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${SOURCES})

# Includes
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC include)

# Enable all warnings
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Set executable filename
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "mm")