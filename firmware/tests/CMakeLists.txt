cmake_minimum_required(VERSION 3.22)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  message(FATAL_ERROR "This project must only be built as a subdirectory. "
                      "Configure the parent project instead.")
endif()

enable_testing()

set(CMAKE_PROJECT_NAME alltests)
project(${CMAKE_PROJECT_NAME})

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        6910c9d
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

file(GLOB_RECURSE TEST_SOURCES *.cpp)

foreach(TEST_SOURCE ${TEST_SOURCES})
  get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WLE)
  add_executable(${TEST_NAME} ${TEST_SOURCE})

  # Link to core library
  target_link_libraries(${TEST_NAME} micromouse)

  # Link to Google Test
  target_link_libraries(${TEST_NAME} GTest::gtest_main)

  gtest_discover_tests(${TEST_NAME})
endforeach()
