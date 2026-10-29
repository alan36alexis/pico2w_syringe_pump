#include "i2c_bus.h"
#include "system_config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

static SemaphoreHandle_t s_i2c_mutex = NULL;
static bool s_i2c_initialized = false;

bool i2c_bus_init(void) {
    if (s_i2c_initialized) {
        return true;
    }

    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        if (s_i2c_mutex == NULL) {
            return false;
        }
    }

    // Inicializar controlador I2C0 a 400 kHz (Fast-mode)
    i2c_init(I2C0_BUS_INST, I2C0_BAUDRATE_HZ);

    // Configurar pines GP12 y GP13 como I2C
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);

    // Habilitar pull-ups internos de refuerzo en el RP2350
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);

    s_i2c_initialized = true;
    return true;
}

bool i2c_bus_lock(TickType_t wait_ticks) {
    if (s_i2c_mutex == NULL) {
        return false;
    }
    // Si el scheduler aún no arrancó, hay un único flujo de ejecución activo
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return true;
    }
    return (xSemaphoreTake(s_i2c_mutex, wait_ticks) == pdTRUE);
}

void i2c_bus_unlock(void) {
    if (s_i2c_mutex != NULL && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

int i2c_bus_write(uint8_t addr, const uint8_t *src, size_t len, bool nostop, uint32_t timeout_ms) {
    if (!s_i2c_initialized || src == NULL || len == 0) {
        return PICO_ERROR_GENERIC;
    }

    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (!i2c_bus_lock(ticks)) {
        return PICO_ERROR_TIMEOUT;
    }

    uint timeout_us = timeout_ms * 1000u;
    if (timeout_us == 0) {
        timeout_us = I2C0_TIMEOUT_US;
    }

    int res = i2c_write_timeout_us(I2C0_BUS_INST, addr, src, len, nostop, timeout_us);
    i2c_bus_unlock();
    return res;
}

int i2c_bus_read(uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint32_t timeout_ms) {
    if (!s_i2c_initialized || dst == NULL || len == 0) {
        return PICO_ERROR_GENERIC;
    }

    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (!i2c_bus_lock(ticks)) {
        return PICO_ERROR_TIMEOUT;
    }

    uint timeout_us = timeout_ms * 1000u;
    if (timeout_us == 0) {
        timeout_us = I2C0_TIMEOUT_US;
    }

    int res = i2c_read_timeout_us(I2C0_BUS_INST, addr, dst, len, nostop, timeout_us);
    i2c_bus_unlock();
    return res;
}

int i2c_bus_write_read(uint8_t addr, const uint8_t *tx_buf, size_t tx_len,
                       uint8_t *rx_buf, size_t rx_len, uint32_t timeout_ms) {
    if (!s_i2c_initialized || tx_buf == NULL || tx_len == 0 || rx_buf == NULL || rx_len == 0) {
        return PICO_ERROR_GENERIC;
    }

    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (!i2c_bus_lock(ticks)) {
        return PICO_ERROR_TIMEOUT;
    }

    uint timeout_us = timeout_ms * 1000u;
    if (timeout_us == 0) {
        timeout_us = I2C0_TIMEOUT_US;
    }

    // Repeated START: nostop = true en el envío de dirección/comando
    int res = i2c_write_timeout_us(I2C0_BUS_INST, addr, tx_buf, tx_len, true, timeout_us);
    if (res >= 0) {
        res = i2c_read_timeout_us(I2C0_BUS_INST, addr, rx_buf, rx_len, false, timeout_us);
    }

    i2c_bus_unlock();
    return res;
}
