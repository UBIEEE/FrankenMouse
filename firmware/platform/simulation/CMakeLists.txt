cmake_minimum_required(VERSION 3.22)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  message(FATAL_ERROR "This project must only be built as a subdirectory. "
                      "Configure the parent project instead.")
endif()

set(CMAKE_PROJECT_NAME simulation)
project(${CMAKE_PROJECT_NAME})

add_executable(${CMAKE_PROJECT_NAME})

# Sources
file(GLOB_RECURSE USER_SOURCES src/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# Includes
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE include)

# Enable all warnings
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  target_compile_options(micromouse PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Link to core library
target_link_libraries(${CMAKE_PROJECT_NAME} micromouse)

# Link with ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

ament_target_dependencies(${CMAKE_PROJECT_NAME} rclcpp std_msgs geometry_msgs)
