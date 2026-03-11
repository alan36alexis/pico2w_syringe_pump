#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "FreeRTOS.h"
#include "task.h"

// Includes de los "mains" divididos
#include "core0_main.h"
#include "core1_main.h"

/**
 * @brief Programa principal e inicializador
 */
int main(void) {
    // Inicialización estándar de entrada/salida (USB/UART)
    stdio_init_all();

    // 1. Arranca el Core 1 con su propia función principal (Baremetal)
    multicore_launch_core1(core1_main);

    // 2. Configura las tareas de FreeRTOS que correrán en el Core 0
    core0_main_setup();

    // 3. Arranca el scheduler (a partir de aquí FreeRTOS toma control del Core 0)
    vTaskStartScheduler();
    
    // Este loop nunca debería ejecutarse a menos que el scheduler se detenga
    while(1);
    
    return 0;
}