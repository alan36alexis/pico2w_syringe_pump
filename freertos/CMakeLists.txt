cmake_minimum_required(VERSION 3.12)
project(freertos)

# Selección del core según la placa
if(${PICO_BOARD} STREQUAL "pico2")
    set(CORE RP2350_ARM_NTZ) # Pico 2 normal con core ARM
elseif(${PICO_BOARD} STREQUAL "pico2_w")
    # Usar ARM en Pico 2 W (M33)
    set(CORE RP2350_ARM_NTZ)
elseif(${PICO_BOARD} STREQUAL "pico")
    set(CORE RP2040)
endif()

# Archivos base de FreeRTOS
add_library(freertos STATIC
    event_groups.c
    list.c
    queue.c
    stream_buffer.c
    tasks.c
    timers.c
    portable/MemMang/heap_3.c
    portable/GCC/${CORE}/port.c
)

# Archivos extra para ARM solamente
if(${CORE} STREQUAL "RP2350_ARM_NTZ")
    target_sources(freertos PRIVATE
        portable/GCC/${CORE}/mpu_wrappers_v2_asm.c
        portable/GCC/${CORE}/portasm.c
    )
endif()

# Includes
target_include_directories(freertos PUBLIC
    include
    portable/GCC/${CORE}/include
)

# Dependencias
target_link_libraries(freertos PUBLIC pico_stdlib hardware_exception)
