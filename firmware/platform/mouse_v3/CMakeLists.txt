cmake_minimum_required(VERSION 3.22)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  message(FATAL_ERROR "This project must only be built as a subdirectory. "
                      "Configure the parent project instead.")
endif()

set(CMAKE_PROJECT_NAME mouse-v3-firmware)
project(${CMAKE_PROJECT_NAME})

enable_language(ASM)

add_executable(${CMAKE_PROJECT_NAME})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)

# Additional C++ sources
file(GLOB_RECURSE USER_SOURCES Core/Src/*.cpp)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# STM32 generated code has a lot of pointless warnings
target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -w)

# Link to core library
target_link_libraries(${CMAKE_PROJECT_NAME} micromouse)

