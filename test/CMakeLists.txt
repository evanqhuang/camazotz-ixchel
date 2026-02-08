cmake_minimum_required(VERSION 3.14)
project(nav_math_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
  GIT_SHALLOW TRUE
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Include directories - stubs FIRST to intercept pico.h
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Test executable
add_executable(nav_math_tests
  test_nav_math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/nav_math.cpp
)

# Compiler flags
target_compile_options(nav_math_tests PRIVATE
  -Wall -Wextra -Werror -Wshadow -Wcast-align -Wfloat-equal -Wnull-dereference
)

# Link Google Test
target_link_libraries(nav_math_tests
  GTest::gtest_main
)

# Enable testing
enable_testing()
include(GoogleTest)
gtest_discover_tests(nav_math_tests)
