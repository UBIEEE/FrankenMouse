cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

set(CMAKE_OBJCXX_STANDARD 20)
set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_EXTENSIONS OFF)

# Default to debug build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Project name
set(CMAKE_PROJECT_NAME ROS2AppFeedback)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Languages
enable_language(OBJCXX)

# Make core library
add_library(${CMAKE_PROJECT_NAME} SHARED ROS2AppFeedback.mm)

target_link_libraries(${CMAKE_PROJECT_NAME} "-framework Foundation")

# Set output directory for the library
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

# Includes
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Enable all warnings
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Link with ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs)

