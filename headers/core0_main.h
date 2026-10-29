#ifndef CORE0_MAIN_H
#define CORE0_MAIN_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Setup tasks for Core 0 (FreeRTOS)
 */
void core0_main_setup(void);

/**
 * @brief Notifica a la tarea de Wi-Fi para despertar de backoff / intentar reconexión
 */
void wifi_manager_notify(void);

/**
 * @brief Fuerza la conexión inmediata a un SSID específico sin importar señal
 */
void wifi_manager_force_connect(const char *ssid);

/**
 * @brief Ejecuta un escaneo de redes disponibles y lo imprime por consola serial
 */
void wifi_manager_scan_and_print(void);

#ifdef __cplusplus
}
#endif

#endif // CORE0_MAIN_H
