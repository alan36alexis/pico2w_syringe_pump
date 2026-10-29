#include "io_expander.h"
#include "pcal9555.h"
#include "system_config.h"
#include "system_events.h"
#include "system_queues.h"
#include "task.h"
#include <stdio.h>

#define IO_EXPANDER_STACK_WORDS  (configMINIMAL_STACK_SIZE * 3)

static volatile uint16_t s_filtered_inputs       = 0xFFFF;
static volatile bool     s_is_online              = false;
static uint16_t          s_input_mask             = 0xFFFF;
static io_expander_change_cb_t s_change_cb        = NULL;
static TaskHandle_t      s_task_handle            = NULL;

/**
 * @brief Init buzzer check: realiza 2 beeps breves (100 ms c/u separados por 100 ms)
 *        para verificar acústicamente el funcionamiento del hardware al inicio.
 */
static void run_init_buzzer_check(void) {
    pcal9555_write_pin(EXP_PIN_BUZZER_CNTL, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    pcal9555_write_pin(EXP_PIN_BUZZER_CNTL, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    pcal9555_write_pin(EXP_PIN_BUZZER_CNTL, true);
    vTaskDelay(pdMS_TO_TICKS(100));
    pcal9555_write_pin(EXP_PIN_BUZZER_CNTL, false);
}

static void task_io_expander(void *params) {
    (void)params;
    uint8_t debounce_counters[16] = {0};
    uint16_t raw_inputs = 0;

    // Lectura inicial para estabilizar estado sin generar eventos falsos
    if (pcal9555_read_all_pins(&raw_inputs)) {
        s_filtered_inputs = raw_inputs & s_input_mask;
        s_is_online = true;
    }

    // Init buzzer check en el arranque de la tarea
    run_init_buzzer_check();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(PCAL9555_POLL_INTERVAL_MS));

        if (!pcal9555_read_all_pins(&raw_inputs)) {
            s_is_online = false;
            continue;
        }
        s_is_online = true;

        uint16_t changed_mask = 0;

        for (uint8_t pin = 0; pin < 16; pin++) {
            uint16_t pin_bit = (1u << pin);

            // Solo filtramos y procesamos pines definidos como entradas
            if (!(s_input_mask & pin_bit)) {
                continue;
            }

            bool raw_state = ((raw_inputs & pin_bit) != 0);
            bool cur_state = ((s_filtered_inputs & pin_bit) != 0);

            if (raw_state != cur_state) {
                debounce_counters[pin]++;
                if (debounce_counters[pin] >= PCAL9555_DEBOUNCE_COUNT) {
                    if (raw_state) {
                        s_filtered_inputs |= pin_bit;
                    } else {
                        s_filtered_inputs &= ~pin_bit;
                    }
                    changed_mask |= pin_bit;
                    debounce_counters[pin] = 0;
                }
            } else {
                debounce_counters[pin] = 0;
            }
        }

        // Si alguna entrada cambió de estado luego del debounce, notificar al sistema
        if (changed_mask != 0) {
            CORE0_EMIT(EV_ACT_IO_EXP_INPUT, param, (uint32_t)s_filtered_inputs);

            if (s_change_cb != NULL) {
                s_change_cb(s_filtered_inputs, changed_mask);
            }
        }
    }
}

bool io_expander_init(const pcal9555_config_t *cfg) {
    if (cfg != NULL) {
        s_input_mask = cfg->dir_mask;
    } else {
        s_input_mask = PCAL9555_DIR_MASK_DEFAULT;
    }

    bool ok = pcal9555_init(cfg);
    s_is_online = ok;
    return ok;
}

bool io_expander_start_task(UBaseType_t priority) {
    if (s_task_handle != NULL) {
        return true; // Ya iniciada
    }

    BaseType_t res = xTaskCreate(task_io_expander,
                                 "IO_Expander",
                                 IO_EXPANDER_STACK_WORDS,
                                 NULL,
                                 priority,
                                 &s_task_handle);
    return (res == pdPASS);
}

uint16_t io_expander_get_inputs(void) {
    return s_filtered_inputs;
}

bool io_expander_get_input_pin(uint8_t pin) {
    if (pin > 15) {
        return false;
    }
    return ((s_filtered_inputs & (1u << pin)) != 0);
}

bool io_expander_set_output(uint8_t pin, bool level) {
    return pcal9555_write_pin(pin, level);
}

bool io_expander_toggle_output(uint8_t pin) {
    return pcal9555_toggle_pin(pin);
}

bool io_expander_write_outputs(uint16_t values, uint16_t mask) {
    return pcal9555_write_pins(values, mask);
}

bool io_expander_is_online(void) {
    return s_is_online;
}

void io_expander_set_change_callback(io_expander_change_cb_t cb) {
    s_change_cb = cb;
}
