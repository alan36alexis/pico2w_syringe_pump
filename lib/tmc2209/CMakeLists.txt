# tmc2209 Library CMakeLists.txt
cmake_minimum_required(VERSION 3.13)

# Add the PIO assembly file to the library

# Define the library source files
add_library(tmc2209
    tmc2209.c
)

pico_generate_pio_header(tmc2209 ${CMAKE_CURRENT_LIST_DIR}/tmc2209.pio)

# Specify the include directories for the library
target_include_directories(tmc2209 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link hardware dependencies required by the TMC2209 driver
target_link_libraries(tmc2209 PUBLIC
    hardware_dma
    hardware_pio
    hardware_irq
    hardware_clocks
    hardware_gpio
    hardware_timer
    hardware_uart
    pico_stdlib
)
