cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
  message(FATAL_ERROR "Windows is not currently supported! Please use WSL or a Unix-like system.")
endif()

###
### Select target
###

option(BUILD_WITH_BLE "Include BLE support for communicating wirelessly with the physical MicroMoue" ON)
option(BUILD_WITH_ROS2 "Include ROS2 support for interfacing with the simulation" ON)
option(BUILD_WITH_SERIAL "Include Serial support for communicating with the physical MicroMouse over UART and for interfacing with serial controller inputs" ON)

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

# Set executable filename
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "mm")

###
### Link libraries
###

include(FetchContent)
find_package(PkgConfig REQUIRED)

# ROS2
if(BUILD_WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs geometry_msgs)

  list(APPEND DEF_LIST "WITH_ROS2=1")
endif()

# SimpleBLE
if(BUILD_WITH_BLE)
  find_package(simpleble QUIET)
  if(NOT simpleble_FOUND)
    message(STATUS "simpleble not found, fetching from git")

    FetchContent_Declare(
        simpleble
        GIT_REPOSITORY https://github.com/SimpleBLE/SimpleBLE.git
        GIT_TAG v0.9.1
        GIT_SHALLOW YES
    )

    cmake_policy(SET CMP0169 OLD)

    FetchContent_GetProperties(simpleble)
    if(NOT simpleble_POPULATED)
        FetchContent_Populate(simpleble)
        list(APPEND CMAKE_MODULE_PATH "${simpleble_SOURCE_DIR}/cmake/find")
        add_subdirectory("${simpleble_SOURCE_DIR}/simpleble" "${simpleble_BINARY_DIR}")
    endif()
  endif()

  target_link_libraries(${CMAKE_PROJECT_NAME} simpleble::simpleble)

  list(APPEND DEF_LIST "WITH_BLE=1")
endif()

if(BUILD_WITH_SERIAL)
  list(APPEND DEF_LIST "WITH_SERIAL=1")
endif()

# Isocline

FetchContent_Declare(
    isocline
    GIT_REPOSITORY https://github.com/petelilley/isocline.git
    GIT_TAG main
)

FetchContent_MakeAvailable(isocline)

target_link_libraries(${CMAKE_PROJECT_NAME} isocline)

###
### Definitions
###

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC ${DEF_LIST})

###
### Compiler settings
###

if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
