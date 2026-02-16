cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(units)

add_library(units INTERFACE)
target_include_directories(units INTERFACE include)
target_compile_definitions(units INTERFACE UNIT_LIB_DEFAULT_TYPE=float)

target_link_libraries(units INTERFACE gcem)