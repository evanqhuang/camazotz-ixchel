# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.14)

# Set board before SDK import - automatically selects RP2350 platform and Cortex-M33 toolchain
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Include SDK bootstrap
include(pico_sdk_import.cmake)

# Project declaration
project(mapper C CXX ASM)

# Standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pico SDK initialization
pico_sdk_init()

# =============================================================================
# External Dependencies (FetchContent)
# =============================================================================
include(FetchContent)

# Allow FetchContent_Populate for libraries that cannot use MakeAvailable
# (AS5600 root CMakeLists.txt conflicts with our SDK init, FatFS target is in src/)
cmake_policy(SET CMP0169 OLD)

# BNO08x IMU (robotcopper) — proper CMake target at repo root
# Exports: BNO08x_Pico_Library (links hardware_i2c, sh2 internally)
FetchContent_Declare(bno08x
    GIT_REPOSITORY https://github.com/robotcopper/BNO08x_Pico_Library.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(bno08x)

# AS5600 Encoder (dancesWithMachines) — no CMake target upstream
# Root CMakeLists.txt calls pico_sdk_init(), so Populate is used to skip it
FetchContent_Declare(as5600
    GIT_REPOSITORY https://github.com/dancesWithMachines/dwm_pico_as5600.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_Populate(as5600)

add_library(dwm_pico_as5600 STATIC
    ${as5600_SOURCE_DIR}/lib/dwm_pico_AS5600/dwm_pico_AS5600.c
)
target_include_directories(dwm_pico_as5600 SYSTEM PUBLIC
    ${as5600_SOURCE_DIR}/lib/dwm_pico_AS5600
)
target_link_libraries(dwm_pico_as5600 PUBLIC pico_stdlib hardware_i2c)

# FatFS SD/SDIO (carlk3) — library CMakeLists.txt in src/ subdirectory
# Exports: no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
FetchContent_Declare(fatfs
    GIT_REPOSITORY https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_Populate(fatfs)
add_subdirectory(${fatfs_SOURCE_DIR}/src ${fatfs_BINARY_DIR}/src)

# Verify SDK version >= 2.0.0
if (PICO_SDK_VERSION_STRING VERSION_LESS "2.0.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.0.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build-type specific flags
# Release: -O2 (not -O3) per IEC 61508/MISRA safety guidelines
# Debug: -Og for debuggability
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")

# Interface library for common include paths
add_library(project_common INTERFACE)
target_include_directories(project_common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/logic
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
)

# Compiler flags for project code only (not applied globally to avoid SDK errors)
# These will be applied to individual source files in src/CMakeLists.txt
set(PROJECT_WARNING_FLAGS
    -Wall
    -Wextra
    -Werror
    -Wshadow
    -Wcast-align
    -Wfloat-equal
    -Wnull-dereference
)

set(PROJECT_SAFETY_FLAGS
    -fstack-protector-strong
    -fstack-usage
)

# Add subdirectories
add_subdirectory(src)
