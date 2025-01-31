cmake_minimum_required(VERSION 3.22)

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "In-source builds are not allowed.")
endif()

option(NAV_DESKTOP_TESTS "Enable unit tests" ON)

if(NAV_DESKTOP_TESTS)
  set(CMAKE_PROJECT_NAME navigation_tests)

  message(STATUS "*** Building Navigation Library for desktop testing ***")

  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        f8d7d77
  )

  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  FetchContent_MakeAvailable(googletest)
else()
  set(CMAKE_PROJECT_NAME navigation)

  message(STATUS "*** Building Navigation Library for MicroMouse ***")
endif()

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Project
project(${CMAKE_PROJECT_NAME})

if(NAV_DESKTOP_TESTS)
  # Enable testing
  enable_testing()

  # Build executable to run tests
  add_executable(${CMAKE_PROJECT_NAME})

  # Test sources
  file(GLOB_RECURSE TEST_SOURCES Tests/*.cpp)
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${TEST_SOURCES})

  # Link with Google Test
  target_link_libraries(${CMAKE_PROJECT_NAME} gtest_main)

  # Discover tests
  include(GoogleTest)
  gtest_discover_tests(${CMAKE_PROJECT_NAME})
else()
  # Build library
  add_library(${CMAKE_PROJECT_NAME})
endif()

# Include directories
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC Inc)

# Sources
file(GLOB_RECURSE USER_SOURCES Src/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
