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


add_subdirectory(freertos)
target_include_directories(freertos PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(lvgl)

# 👉 IMPORTANTE: lv_conf para LVGL
target_include_directories(lvgl PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/headers
)

target_compile_definitions(lvgl PUBLIC
    LV_CONF_INCLUDE_SIMPLE
)

file(GLOB_RECURSE EEZ_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/eez/*.c
    ${CMAKE_CURRENT_LIST_DIR}/eez/*.cpp
)

list(FILTER EEZ_SOURCES EXCLUDE REGEX "eez/core/.*")
list(FILTER EEZ_SOURCES EXCLUDE REGEX "eez/gui/.*")
list(FILTER EEZ_SOURCES EXCLUDE REGEX "eez/libs/.*")
list(FILTER EEZ_SOURCES EXCLUDE REGEX "eez/platforms/.*")
list(FILTER EEZ_SOURCES EXCLUDE REGEX "eez/flow/.*")

file(GLOB UI_SOURCES CONFIGURE_DEPENDS
    ui/*.c
    ui/*.cpp
    ui/ui_image_*.c
    ui/screens/*.c
    ui/components/*.c
    ui/images/*.c
)

target_link_libraries(lvgl PUBLIC pico_stdlib)

# Define the main executable
add_executable(pico2w_syringe_pump
    src/main.c
    src/core0_main.c
    src/core1_main.c
    src/closed_loop.c
    src/crosscore_logger.c
    src/crosscore_cmd.c
    src/cmd_dispatcher.c
    src/test_modes.c
    src/mqtt_client.c
    src/mqtt_topics.c
    src/syringe_pump_api.c
    src/config_manager.c
    src/cmd_gate.c
    src/ui_state.c
    src/ui_events.c
    src/fsm_table.c
    # EDA Pub-Sub logging infrastructure
    src/system_queues.c
    src/event_broker.c
    src/serial_consumer.c
    src/mqtt_consumer.c
    src/hmi_consumer.c
    src/display_driver.c
    src/touch_driver.c
    src/encoder_driver.c
    src/ModuloTFT.c
    src/ui_actions.c
    ${UI_SOURCES}
    ${EEZ_SOURCES}
)


# Definiciones
target_compile_definitions(pico2w_syringe_pump PRIVATE 
    LV_CONF_INCLUDE_SIMPLE
    EEZ_PLATFORM_PICO
    EEZ_FOR_LVGL
    EEZ_PLATFORM_NO_OS
    EEZ_NO_AGG
    EEZ_SKIP_GUI_DISPLAY
    EEZ_CONF_NOSYS
    EEZ_SKIP_CRYPTO
    CUSTOM_OPERATION_TYPES_H
)


pico_generate_pio_header(pico2w_syringe_pump ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)
pico_generate_pio_header(pico2w_syringe_pump ${CMAKE_CURRENT_LIST_DIR}/src/pulse_counter.pio)

# Link the executable with standard libraries and our custom TMC2209 library
target_link_libraries(pico2w_syringe_pump
    pico_stdlib
    tmc2209
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    pico_multicore
    pico_util
    freertos
    hardware_pio
    hardware_adc
    hardware_flash
    hardware_spi
    hardware_gpio
    lvgl
)

# Suppress CYW43 arch-level debug prints (e.g. "connect status: link up")
target_compile_definitions(pico2w_syringe_pump PRIVATE
    PICO_CYW43_ARCH_DEBUG_ENABLED=0
    ENABLE_FSM_TABLE
    ENABLE_TFT
)

# Provide include directories for lwIP and others
target_include_directories(pico2w_syringe_pump PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/headers
    ${CMAKE_CURRENT_LIST_DIR}/ui
)

# STDIO
pico_enable_stdio_uart(pico2w_syringe_pump 1)
pico_enable_stdio_usb(pico2w_syringe_pump 1)

# Additional output formats
pico_add_extra_outputs(pico2w_syringe_pump)
