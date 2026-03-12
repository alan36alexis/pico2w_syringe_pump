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
set(PICO_BOARD pico2_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Initialize Pico SDK
include(pico_sdk_import.cmake)

project(pico2w_syringe_pump C CXX ASM)

# Set standard C/CXX versions
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Include the TMC2209 library
add_subdirectory(lib/tmc2209)

# Include honeywell_spi library if applicable (check if exists locally, if not we assume it needs copying or paths update, but assuming it exists in workspace)
# Since honeywell_spi might not exist locally yet, we need to check this next. I will just link it for now and verify.
# Wait, I see I should see where honeywell_spi is located first via list_dir. I'll defer linking honeywell if it is not in the lib folder yet.
add_subdirectory(lib/honeywell)

add_subdirectory(freertos)
target_include_directories(freertos PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Define the main executable
add_executable(pico2w_syringe_pump 
    src/main.c
    src/core0_main.c
    src/core1_main.c
    src/crosscore_logger.c
    src/test_modes.c
)

# Enable stdout
pico_enable_stdio_usb(pico2w_syringe_pump 1)
pico_enable_stdio_uart(pico2w_syringe_pump 1)

# Link the executable with standard libraries and our custom TMC2209 library
target_link_libraries(pico2w_syringe_pump
    pico_stdlib
    hardware_spi
    tmc2209
    honeywell_spi
    pico_cyw43_arch_none
    pico_multicore
    pico_util
    freertos
)

# Additional output formats
pico_add_extra_outputs(pico2w_syringe_pump)
