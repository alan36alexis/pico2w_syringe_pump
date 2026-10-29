#ifndef MOCK_PCAL9555_H
#define MOCK_PCAL9555_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ENABLE_MOCK_PCAL9555) && (ENABLE_MOCK_PCAL9555 != 0)

/**
 * @brief Inicia la tarea mock del expansor PCAL9555 cuando el hardware I2C no responde.
 * Emite el estado inicial hacia el bus de eventos y refresca periódicamente o ante cambios.
 */
bool mock_pcal9555_start_task(UBaseType_t priority);

/**
 * @brief Permite forzar el estado lógico de un pin simulado (0 o 1).
 */
void mock_pcal9555_set_pin(uint8_t pin, bool level);

#endif // ENABLE_MOCK_PCAL9555

#ifdef __cplusplus
}
#endif

#endif // MOCK_PCAL9555_H

