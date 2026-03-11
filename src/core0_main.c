#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "FreeRTOS.h"
#include "task.h"
#include "core0_main.h"

/**
 * @brief Tarea de inicializacion
 */
static void task_init(void *params) {
    // Inicializacion de GPIO y Wi-Fi chip (CYW43)
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    
    // Elimino la tarea para liberar recursos tras una única ejecución
    vTaskDelete(NULL);
}

/**
 * @brief Tarea de blinky de LED
 */
static void task_blinky(void *params) {
    while(1) {
        // Toggle del LED de la placa Pico W
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
        // Demora de ticks equivalentes a 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Tarea para imprimir el contador proveniente del Core 1
 */
static void task_print_counter(void *params) {
    while(1) {
        // Verifica si hay mensajes en la FIFO provenientes del Core 1
        if (multicore_fifo_rvalid()) {
            uint32_t counter = multicore_fifo_pop_blocking();
            printf("Core 1 counter: %u\n", counter);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Poll cada 100ms
    }
}

/**
 * @brief Función para configurar todas las tareas del Core 0 antes de iniciar FreeRTOS
 */
void core0_main_setup(void) {
    // Creación de las tareas de FreeRTOS
    xTaskCreate(task_init, "Init", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(task_blinky, "Blinky", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(task_print_counter, "Print", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}
