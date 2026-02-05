# Mapper executable
add_executable(mapper
    main.c
)

# Apply strict warning flags only to our source files
string(REPLACE ";" " " PROJECT_FLAGS_STR "${PROJECT_WARNING_FLAGS};${PROJECT_SAFETY_FLAGS}")
set_source_files_properties(main.c PROPERTIES
    COMPILE_FLAGS "${PROJECT_FLAGS_STR}"
)

# Link libraries
target_link_libraries(mapper
    pico_stdlib
    project_common
)

# Enable USB output, disable UART
pico_enable_stdio_usb(mapper 1)
pico_enable_stdio_uart(mapper 0)

# Generate UF2 output for drag-and-drop flashing
pico_add_extra_outputs(mapper)
