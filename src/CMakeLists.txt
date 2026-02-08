# Mapper executable
add_executable(mapper
    main.cpp
    ../drivers/encoder_wrapper.cpp
    ../drivers/imu_wrapper.cpp
    ../drivers/pio_i2c.cpp
    ../drivers/depth_wrapper.cpp
    ../logic/calibration_manager.cpp
    ../logic/nav_math.cpp
    ../utils/stdio_display.cpp
)

# Generate PIO header from assembly
pico_generate_pio_header(mapper ${CMAKE_CURRENT_LIST_DIR}/../drivers/pio_i2c.pio)

# Apply strict warning flags only to our source files
string(REPLACE ";" " " PROJECT_FLAGS_STR "${PROJECT_WARNING_FLAGS};${PROJECT_SAFETY_FLAGS}")
set_source_files_properties(
    main.cpp
    ../drivers/encoder_wrapper.cpp
    ../drivers/imu_wrapper.cpp
    ../drivers/pio_i2c.cpp
    ../drivers/depth_wrapper.cpp
    ../logic/calibration_manager.cpp
    ../logic/nav_math.cpp
    ../utils/stdio_display.cpp
    PROPERTIES COMPILE_FLAGS "${PROJECT_FLAGS_STR}"
)

# Link libraries
target_link_libraries(mapper
    pico_stdlib
    pico_multicore
    pico_float
    pico_double
    hardware_pio
    hardware_i2c
    hardware_irq
    hardware_clocks
    BNO08x_Pico_Library
    dwm_pico_as5600
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    project_common
)

# Enable USB output, disable UART
pico_enable_stdio_usb(mapper 1)
pico_enable_stdio_uart(mapper 0)

# Generate UF2 output for drag-and-drop flashing
pico_add_extra_outputs(mapper)
