#ifndef IO_EXPANDER_H
#define IO_EXPANDER_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "pcal9555.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*io_expander_change_cb_t)(uint16_t current_inputs, uint16_t changed_mask);

/**
 * @brief Inicializa el expansor PCAL9555 con la configuración de hardware.
 * @param cfg Configuración de pines, pull-ups y dirección (o NULL para default).
 * @return true si el expansor fue detectado y configurado exitosamente.
 */
bool io_expander_init(const pcal9555_config_t *cfg);

/**
 * @brief Lanza la tarea de FreeRTOS para polling periódico de entradas y debounce.
 * @param priority Prioridad de la tarea FreeRTOS (ej: tskIDLE_PRIORITY + 1).
 * @return true si la tarea se creó correctamente.
 */
bool io_expander_start_task(UBaseType_t priority);

/**
 * @brief Retorna el último estado filtrado (debounced) de los canales de entrada.
 * @return Máscara de 16 bits con los estados de entrada.
 */
uint16_t io_expander_get_inputs(void);

/**
 * @brief Retorna el estado filtrado de un pin de entrada individual (0 a 15).
 * @param pin Índice del pin.
 * @return true = HIGH, false = LOW.
 */
bool io_expander_get_input_pin(uint8_t pin);

/**
 * @brief Configura el nivel de una salida (thread-safe).
 * @param pin Índice del pin (0 a 15).
 * @param level Nivel lógico (true = HIGH, false = LOW).
 * @return true si se actualizó en el hardware, false en caso de error.
 */
bool io_expander_set_output(uint8_t pin, bool level);

/**
 * @brief Invierte el estado de una salida (toggle).
 * @param pin Índice del pin (0 a 15).
 * @return true si se actualizó en el hardware.
 */
bool io_expander_toggle_output(uint8_t pin);

/**
 * @brief Escribe varias salidas simultáneamente según una máscara de pines.
 * @param values Nuevos valores lógicos.
 * @param mask Máscara de pines a modificar.
 * @return true si se actualizó en el hardware.
 */
bool io_expander_write_outputs(uint16_t values, uint16_t mask);

/**
 * @brief Informa si el PCAL9555 está comunicándose correctamente por I2C.
 * @return true si el último polling o comunicación fue exitosa.
 */
bool io_expander_is_online(void);

/**
 * @brief Registra un callback opcional a ejecutar cuando se detecta un cambio de estado en entradas.
 * @param cb Puntero a función callback.
 */
void io_expander_set_change_callback(io_expander_change_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif // IO_EXPANDER_H
