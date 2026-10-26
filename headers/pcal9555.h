#ifndef PCAL9555_H
#define PCAL9555_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Registros del PCAL9555A (NXP Semiconductors Data Sheet Rev. 2)
// ---------------------------------------------------------------------------

// Registros estándar compatibles con PCA9555
#define PCAL9555_REG_IN_PORT0         0x00 // Entrada Puerto 0 (Solo lectura)
#define PCAL9555_REG_IN_PORT1         0x01 // Entrada Puerto 1 (Solo lectura)
#define PCAL9555_REG_OUT_PORT0        0x02 // Salida Puerto 0 (R/W, default 0xFF)
#define PCAL9555_REG_OUT_PORT1        0x03 // Salida Puerto 1 (R/W, default 0xFF)
#define PCAL9555_REG_POL_INV_PORT0    0x04 // Inversión Polaridad Puerto 0 (R/W, default 0x00)
#define PCAL9555_REG_POL_INV_PORT1    0x05 // Inversión Polaridad Puerto 1 (R/W, default 0x00)
#define PCAL9555_REG_CONFIG_PORT0     0x06 // Configuración Dirección Puerto 0 (1=In, 0=Out, default 0xFF)
#define PCAL9555_REG_CONFIG_PORT1     0x07 // Configuración Dirección Puerto 1 (1=In, 0=Out, default 0xFF)

// Registros Agile I/O extendidos del PCAL9555A
#define PCAL9555_REG_DRV_STR_PORT0_L  0x40 // Corriente de salida P0_0..P0_3
#define PCAL9555_REG_DRV_STR_PORT0_H  0x41 // Corriente de salida P0_4..P0_7
#define PCAL9555_REG_DRV_STR_PORT1_L  0x42 // Corriente de salida P1_0..P1_3
#define PCAL9555_REG_DRV_STR_PORT1_H  0x43 // Corriente de salida P1_4..P1_7
#define PCAL9555_REG_INPUT_LATCH0     0x44 // Latch de entrada Puerto 0 (1=Latch, default 0x00)
#define PCAL9555_REG_INPUT_LATCH1     0x45 // Latch de entrada Puerto 1 (1=Latch, default 0x00)
#define PCAL9555_REG_PULL_EN0         0x46 // Habilitación Pull-up/down Puerto 0 (1=Enable, default 0xFF)
#define PCAL9555_REG_PULL_EN1         0x47 // Habilitación Pull-up/down Puerto 1 (1=Enable, default 0xFF)
#define PCAL9555_REG_PULL_SEL0        0x48 // Selección Pull: 1=Pull-up 100k, 0=Pull-down (default 0xFF)
#define PCAL9555_REG_PULL_SEL1        0x49 // Selección Pull: 1=Pull-up 100k, 0=Pull-down (default 0xFF)
#define PCAL9555_REG_INT_MASK0        0x4A // Máscara de interrupción Puerto 0 (1=Masked, default 0xFF)
#define PCAL9555_REG_INT_MASK1        0x4B // Máscara de interrupción Puerto 1 (1=Masked, default 0xFF)
#define PCAL9555_REG_INT_STATUS0      0x4C // Estado de interrupción Puerto 0 (Solo lectura)
#define PCAL9555_REG_INT_STATUS1      0x4D // Estado de interrupción Puerto 1 (Solo lectura)
#define PCAL9555_REG_OUT_CONF         0x4F // Configuración salida: 0=Push-Pull, 1=Open-Drain

// ---------------------------------------------------------------------------
// Constantes y Helpers de Pines (16 Canales en total)
// Port 0: bits 0..7  (P0_0 .. P0_7)
// Port 1: bits 8..15 (P1_0 .. P1_7)
// ---------------------------------------------------------------------------
#define PCAL9555_PIN_P0(x)            ((uint8_t)(x))
#define PCAL9555_PIN_P1(x)            ((uint8_t)((x) + 8))
#define PCAL9555_PIN_MASK(pin)        (1u << (pin))

// Estructura de configuración inicial para el PCAL9555
typedef struct {
    uint8_t  i2c_addr;        // Dirección esclavo 7-bit (típicamente 0x20)
    uint16_t dir_mask;        // 1 = Entrada, 0 = Salida (16 bits)
    uint16_t pull_en_mask;    // 1 = Resistor pull habilitado, 0 = flotante
    uint16_t pull_sel_mask;   // 1 = Pull-up (100k), 0 = Pull-down (100k)
    uint16_t pol_inv_mask;    // 1 = Polaridad invertida en lectura
    uint16_t latch_en_mask;   // 1 = Retiene estado de entrada hasta lectura
    uint16_t initial_outputs; // Valor inicial para pines configurados como salida
} pcal9555_config_t;

/**
 * @brief Inicializa el expansor PCAL9555 con la configuración especificada.
 * @param cfg Puntero a la configuración (o NULL para configuración segura por defecto).
 * @return true si el dispositivo respondió y fue configurado, false si no hubo ACK.
 */
bool pcal9555_init(const pcal9555_config_t *cfg);

/**
 * @brief Verifica si el PCAL9555 está presente y respondiendo en el bus I2C.
 * @return true si el esclavo responde con ACK, false en caso contrario.
 */
bool pcal9555_is_connected(void);

/**
 * @brief Lee el estado lógico de los 16 canales en una sola transacción ráfaga I2C.
 * @param pin_values Puntero donde se almacenará el valor de 16 bits (bits 0-7: P0, 8-15: P1).
 * @return true si la lectura fue exitosa, false en caso de error.
 */
bool pcal9555_read_all_pins(uint16_t *pin_values);

/**
 * @brief Lee el estado lógico de un pin específico (0 a 15).
 * @param pin Índice de pin (0-7 para P0_0..P0_7, 8-15 para P1_0..P1_7).
 * @param val Puntero donde guardar el estado booleano (true = HIGH, false = LOW).
 * @return true si se leyó correctamente, false en caso de error.
 */
bool pcal9555_read_pin(uint8_t pin, bool *val);

/**
 * @brief Escribe el estado de los 16 canales de salida de forma atómica.
 * @param pin_values Valor de 16 bits para los pines de salida.
 * @return true si la escritura fue exitosa, false en caso de error.
 */
bool pcal9555_write_all_outputs(uint16_t pin_values);

/**
 * @brief Modifica un subconjunto de salidas según la máscara especificada.
 *        No modifica las salidas cuyos bits en pin_mask sean 0.
 * @param pin_values Nuevos valores lógicos.
 * @param pin_mask Máscara de bits que deben actualizarse.
 * @return true si la escritura fue exitosa, false en caso de error.
 */
bool pcal9555_write_pins(uint16_t pin_values, uint16_t pin_mask);

/**
 * @brief Escribe el nivel lógico de un único pin de salida.
 * @param pin Índice del pin (0 a 15).
 * @param val Nivel lógico deseado (true = HIGH, false = LOW).
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool pcal9555_write_pin(uint8_t pin, bool val);

/**
 * @brief Invierte (toggle) el estado actual de un pin de salida.
 * @param pin Índice del pin (0 a 15).
 * @return true si la operación fue exitosa, false en caso de error.
 */
bool pcal9555_toggle_pin(uint8_t pin);

/**
 * @brief Obtiene la última copia en sombra (shadow cache) de los pines de salida.
 * @return Valor de 16 bits del registro de salida en memoria.
 */
uint16_t pcal9555_get_cached_outputs(void);

/**
 * @brief Configura la dirección de los 16 pines (1 = Entrada, 0 = Salida).
 */
bool pcal9555_set_direction(uint16_t input_mask);

/**
 * @brief Configura los resistores internos pull-up y pull-down de 100k (Agile I/O).
 * @param pull_en_mask 1 = Resistor habilitado, 0 = flotante.
 * @param pull_sel_mask 1 = Pull-up (a VDD), 0 = Pull-down (a GND).
 */
bool pcal9555_set_pulls(uint16_t pull_en_mask, uint16_t pull_sel_mask);

/**
 * @brief Habilita/deshabilita la retención de entradas (Input Latch, registros 0x44/0x45).
 * @param latch_mask 1 = Habilita retención de estado hasta próxima lectura.
 */
bool pcal9555_set_input_latch(uint16_t latch_mask);

/**
 * @brief Configura la inversión de polaridad para entradas (registros 0x04/0x05).
 * @param inv_mask 1 = Polaridad invertida.
 */
bool pcal9555_set_polarity_inversion(uint16_t inv_mask);

#ifdef __cplusplus
}
#endif

#endif // PCAL9555_H
