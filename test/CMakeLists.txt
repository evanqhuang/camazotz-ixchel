cmake_minimum_required(VERSION 3.14)
project(mapper_tests CXX)

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

# Strict compiler flags for all test targets
set(TEST_WARNING_FLAGS -Wall -Wextra -Werror -Wshadow -Wcast-align -Wfloat-equal -Wnull-dereference)

# --- nav_math_tests ---
add_executable(nav_math_tests
  test_nav_math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/nav_math.cpp
)
target_compile_options(nav_math_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(nav_math_tests GTest::gtest_main)

# --- depth_math_tests ---
add_executable(depth_math_tests
  test_depth_math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/depth_math.cpp
)
target_compile_options(depth_math_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(depth_math_tests GTest::gtest_main)

# --- encoder_math_tests ---
add_executable(encoder_math_tests
  test_encoder_math.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/encoder_math.cpp
)
target_compile_options(encoder_math_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(encoder_math_tests GTest::gtest_main)

# --- types_tests ---
add_executable(types_tests
  test_types.cpp
)
target_compile_options(types_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(types_tests GTest::gtest_main)

# --- nav_tick_tests ---
add_executable(nav_tick_tests
  test_nav_tick.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/nav_tick.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/nav_math.cpp
)
target_compile_options(nav_tick_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(nav_tick_tests GTest::gtest_main)

# --- display_tests ---
add_executable(display_tests
  test_display.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../utils/stdio_display.cpp
)
target_compile_options(display_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(display_tests GTest::gtest_main)

# --- depth_recovery_tests ---
add_executable(depth_recovery_tests
  test_depth_recovery.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/depth_recovery.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../logic/nav_math.cpp
)
target_compile_options(depth_recovery_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(depth_recovery_tests GTest::gtest_main)

# --- button_debounce_tests ---
add_executable(button_debounce_tests
  test_button_debounce.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../utils/button_debounce.cpp
)
target_compile_options(button_debounce_tests PRIVATE ${TEST_WARNING_FLAGS})
target_link_libraries(button_debounce_tests GTest::gtest_main)

# Enable testing
enable_testing()
include(GoogleTest)
gtest_discover_tests(nav_math_tests)
gtest_discover_tests(depth_math_tests)
gtest_discover_tests(encoder_math_tests)
gtest_discover_tests(types_tests)
gtest_discover_tests(nav_tick_tests)
gtest_discover_tests(display_tests)
gtest_discover_tests(depth_recovery_tests)
gtest_discover_tests(button_debounce_tests)
