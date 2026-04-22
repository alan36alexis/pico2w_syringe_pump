#include "FreeRTOS.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

// Includes de los "mains" divididos
#include "core0_main.h"
#include "core1_main.h"

/**
 * @brief Programa principal e inicializador
 */
int main(void) {
  stdio_init_all();

  // 1. Arranca el Core 1 (Baremetal)
  multicore_launch_core1(core1_main);

  // 2. Configura las tareas y recursos de FreeRTOS que correrán en el Core 0
  core0_main_setup();

  // 3. Arranca el scheduler (a partir de aquí FreeRTOS toma control del Core 0)
  vTaskStartScheduler();

  while (1);

  return 0;
}