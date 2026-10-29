#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el controlador I2C0 en los pines configurados (GP12/GP13)
 *        y crea el Mutex de FreeRTOS para coordinar el acceso concurrente.
 * @return true si la inicialización fue exitosa, false en caso de error.
 */
bool i2c_bus_init(void);

/**
 * @brief Adquiere el mutex del bus I2C.
 * @param wait_ticks Tiempo máximo de espera en ticks de FreeRTOS.
 * @return true si se obtuvo el bus, false si expiró el timeout.
 */
bool i2c_bus_lock(TickType_t wait_ticks);

/**
 * @brief Libera el mutex del bus I2C.
 */
void i2c_bus_unlock(void);

/**
 * @brief Escritura atómica y thread-safe en el bus I2C con timeout.
 * @param addr Dirección esclavo de 7 bits.
 * @param src Buffer de datos a transmitir.
 * @param len Cantidad de bytes a transmitir.
 * @param nostop Si es true, omite condición de STOP al finalizar (para repeated START).
 * @param timeout_ms Timeout total en milisegundos.
 * @return Cantidad de bytes transmitidos, o código de error negativo (pico-sdk).
 */
int i2c_bus_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop, uint32_t timeout_ms);

/**
 * @brief Lectura atómica y thread-safe en el bus I2C con timeout.
 * @param addr Dirección esclavo de 7 bits.
 * @param dst Buffer de destino donde guardar los datos.
 * @param len Cantidad de bytes a recibir.
 * @param nostop Si es true, omite condición de STOP al finalizar.
 * @param timeout_ms Timeout total en milisegundos.
 * @return Cantidad de bytes leídos, o código de error negativo (pico-sdk).
 */
int i2c_bus_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint32_t timeout_ms);

/**
 * @brief Escritura seguida de lectura atómica con condición Repeated START.
 *        Mantiene el bus bloqueado bajo mutex durante toda la transacción,
 *        evitando colisiones entre el PCAL9555 y el futuro RTC DS3231.
 * @param addr Dirección esclavo de 7 bits.
 * @param tx_buf Buffer a escribir (típicamente registro/comando).
 * @param tx_len Longitud del buffer de escritura.
 * @param rx_buf Buffer donde almacenar la lectura.
 * @param rx_len Longitud de la lectura.
 * @param timeout_ms Timeout total en milisegundos.
 * @return Cantidad de bytes leídos en rx_buf, o código negativo en caso de error.
 */
int i2c_bus_write_read(uint8_t addr, const uint8_t *tx_buf, size_t tx_len,
                       uint8_t *rx_buf, size_t rx_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // I2C_BUS_H
