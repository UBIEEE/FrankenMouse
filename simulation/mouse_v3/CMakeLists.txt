cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Project name
set(CMAKE_PROJECT_NAME mouse-v3-gdextension)

# Export compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project
project(${CMAKE_PROJECT_NAME})

# Language
enable_language(CXX)

###
### Build library
###

# Make library
if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  add_library(${CMAKE_PROJECT_NAME} STATIC)
else()
  add_library(${CMAKE_PROJECT_NAME} SHARED)
endif()


# Sources
file(GLOB_RECURSE LIB_SOURCES lib/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${LIB_SOURCES})

# Include directories
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE include)

# Enable all warnings
# target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)

###
### Link to Godot
###

include(FetchContent)

FetchContent_Declare(
        godot-cpp
        GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
        GIT_TAG godot-4.4-stable
)

FetchContent_MakeAvailable(godot-cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} godot::cpp)

###
### Link to ROS2
###

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs geometry_msgs)

###
### Output
###

set(SIM_PROJECT_DIR "${CMAKE_SOURCE_DIR}/simulator")
set(SIM_BIN_DIR "${SIM_PROJECT_DIR}/bin")

string(TOLOWER ${CMAKE_SYSTEM_NAME} SIM_PLATFORM)
string(TOLOWER ${CMAKE_BUILD_TYPE} SIM_TARGET)
string(TOLOWER ${CMAKE_SYSTEM_PROCESSOR} SIM_ARCH)
set(SIM_NAME "simulator")

file(MAKE_DIRECTORY ${SIM_BIN_DIR})

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  set(SIM_PLATFORM "macos")
  set(CMAKE_SHARED_LIBRARY_SUFFIX "") # Remove .dylib suffix
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${SIM_BIN_DIR}/lib${SIM_NAME}.${SIM_PLATFORM}.${SIM_TARGET}.framework")
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_NAME "${SIM_NAME}.${SIM_PLATFORM}.${SIM_TARGET}")
elseif(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${SIM_BIN_DIR}")
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${SIM_NAME}.${SIM_PLATFORM}.${SIM_TARGET}")
  # TODO: Add godot-cpp framework to bin??
else()
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${SIM_BIN_DIR}")
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME "${SIM_NAME}.${SIM_PLATFORM}.${SIM_TARGET}.${SIM_ARCH}")
endif()

