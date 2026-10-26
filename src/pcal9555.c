#include "pcal9555.h"
#include "i2c_bus.h"
#include "system_config.h"
#include <stdio.h>

#define PCAL9555_I2C_TIMEOUT_MS 50

static uint8_t  s_device_addr     = PCAL9555_I2C_ADDR_DEFAULT;
static uint16_t s_cached_outputs  = 0xFFFF;
static uint16_t s_cached_dir      = 0xFFFF;
static bool     s_initialized     = false;

// Helper estático para escribir un par de registros de 16 bits (Puerto 0 y Puerto 1)
static bool write_register_pair(uint8_t reg_port0, uint16_t val) {
    uint8_t tx[3] = {
        reg_port0,
        (uint8_t)(val & 0xFF),
        (uint8_t)((val >> 8) & 0xFF)
    };
    int res = i2c_bus_write(s_device_addr, tx, sizeof(tx), false, PCAL9555_I2C_TIMEOUT_MS);
    return (res == (int)sizeof(tx));
}

// Helper estático para leer un par de registros de 16 bits (Puerto 0 y Puerto 1)
static bool read_register_pair(uint8_t reg_port0, uint16_t *val) {
    if (val == NULL) {
        return false;
    }
    uint8_t reg = reg_port0;
    uint8_t rx[2] = {0, 0};
    int res = i2c_bus_write_read(s_device_addr, &reg, 1, rx, 2, PCAL9555_I2C_TIMEOUT_MS);
    if (res == 2) {
        *val = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
        return true;
    }
    return false;
}

bool pcal9555_is_connected(void) {
    // Intenta leer el registro de configuración (0x06 / 0x07)
    uint16_t dummy = 0;
    return read_register_pair(PCAL9555_REG_CONFIG_PORT0, &dummy);
}

bool pcal9555_init(const pcal9555_config_t *cfg) {
    if (!i2c_bus_init()) {
        return false;
    }

    if (cfg != NULL && cfg->i2c_addr != 0) {
        s_device_addr = cfg->i2c_addr;
    } else {
        s_device_addr = PCAL9555_I2C_ADDR_DEFAULT;
    }

    // Verificar presencia del chip en el bus
    if (!pcal9555_is_connected()) {
        return false;
    }

    pcal9555_config_t default_cfg = {
        .i2c_addr        = s_device_addr,
        .dir_mask        = PCAL9555_DIR_MASK_DEFAULT,
        .pull_en_mask    = PCAL9555_PULL_EN_DEFAULT,
        .pull_sel_mask   = PCAL9555_PULL_SEL_DEFAULT,
        .pol_inv_mask    = 0x0000,
        .latch_en_mask   = 0x0000,
        .initial_outputs = PCAL9555_OUTPUTS_DEFAULT
    };

    const pcal9555_config_t *c = (cfg != NULL) ? cfg : &default_cfg;

    // 1. Establecer valor inicial de salidas ANTES de cambiar dirección a salida
    s_cached_outputs = c->initial_outputs;
    if (!write_register_pair(PCAL9555_REG_OUT_PORT0, s_cached_outputs)) {
        return false;
    }

    // 2. Configurar inversión de polaridad
    if (!write_register_pair(PCAL9555_REG_POL_INV_PORT0, c->pol_inv_mask)) {
        return false;
    }

    // 3. Configurar Agile I/O: Pull-up/pull-down
    if (!write_register_pair(PCAL9555_REG_PULL_EN0, c->pull_en_mask)) {
        return false;
    }
    if (!write_register_pair(PCAL9555_REG_PULL_SEL0, c->pull_sel_mask)) {
        return false;
    }

    // 4. Configurar Agile I/O: Input latching
    if (!write_register_pair(PCAL9555_REG_INPUT_LATCH0, c->latch_en_mask)) {
        return false;
    }

    // 5. Configurar dirección (1 = Input, 0 = Output)
    s_cached_dir = c->dir_mask;
    if (!write_register_pair(PCAL9555_REG_CONFIG_PORT0, s_cached_dir)) {
        return false;
    }

    s_initialized = true;
    return true;
}

bool pcal9555_read_all_pins(uint16_t *pin_values) {
    if (pin_values == NULL) {
        return false;
    }
    return read_register_pair(PCAL9555_REG_IN_PORT0, pin_values);
}

bool pcal9555_read_pin(uint8_t pin, bool *val) {
    if (pin > 15 || val == NULL) {
        return false;
    }
    uint16_t all_pins = 0;
    if (!pcal9555_read_all_pins(&all_pins)) {
        return false;
    }
    *val = ((all_pins & (1u << pin)) != 0);
    return true;
}

bool pcal9555_write_all_outputs(uint16_t pin_values) {
    s_cached_outputs = pin_values;
    return write_register_pair(PCAL9555_REG_OUT_PORT0, s_cached_outputs);
}

bool pcal9555_write_pins(uint16_t pin_values, uint16_t pin_mask) {
    uint16_t new_out = (s_cached_outputs & ~pin_mask) | (pin_values & pin_mask);
    return pcal9555_write_all_outputs(new_out);
}

bool pcal9555_write_pin(uint8_t pin, bool val) {
    if (pin > 15) {
        return false;
    }
    uint16_t mask = (1u << pin);
    uint16_t values = val ? mask : 0;
    return pcal9555_write_pins(values, mask);
}

bool pcal9555_toggle_pin(uint8_t pin) {
    if (pin > 15) {
        return false;
    }
    bool current_state = ((s_cached_outputs & (1u << pin)) != 0);
    return pcal9555_write_pin(pin, !current_state);
}

uint16_t pcal9555_get_cached_outputs(void) {
    return s_cached_outputs;
}

bool pcal9555_set_direction(uint16_t input_mask) {
    s_cached_dir = input_mask;
    return write_register_pair(PCAL9555_REG_CONFIG_PORT0, s_cached_dir);
}

bool pcal9555_set_pulls(uint16_t pull_en_mask, uint16_t pull_sel_mask) {
    if (!write_register_pair(PCAL9555_REG_PULL_EN0, pull_en_mask)) {
        return false;
    }
    return write_register_pair(PCAL9555_REG_PULL_SEL0, pull_sel_mask);
}

bool pcal9555_set_input_latch(uint16_t latch_mask) {
    return write_register_pair(PCAL9555_REG_INPUT_LATCH0, latch_mask);
}

bool pcal9555_set_polarity_inversion(uint16_t inv_mask) {
    return write_register_pair(PCAL9555_REG_POL_INV_PORT0, inv_mask);
}
