#ifndef TEST_MODES_H
#define TEST_MODES_H

#include <stdbool.h>
#include "tmc2209.h"

// Test Functions extracted from main.c

/**
 * @brief Pruebas básicas de movimiento y RPM.
 * Calcula VACTUAL para una velocidad dada y mueve el motor.
 */
void test_rpm_movement_demo(TMC2209_t *motor);

/**
 * @brief Ciclo de prueba de StallGuard.
 * Varía la corriente y/o velocidad para observar la respuesta de carga.
 */
void test_stallguard_analysis(TMC2209_t *motor);

/**
 * @brief Ciclo de resolución de microstepping.
 * Cambia dinámicamente entre resoluciones de micropasos (1, 2, 4, 16)
 * para verificar el comportamiento del motor.
 */
void test_microstepping_cycle(TMC2209_t *motor, bool use_uart_mode);

/**
 * @brief Prueba simple de polling de registros IOIN (StallGuard / Pines).
 */
void test_ioin_polling(TMC2209_t *motor);

#endif // TEST_MODES_H
