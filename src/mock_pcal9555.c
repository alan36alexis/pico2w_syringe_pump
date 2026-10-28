#include "mock_pcal9555.h"

#ifdef ENABLE_MOCK_PCAL9555

#include "system_config.h"
#include "system_events.h"
#include "system_queues.h"
#include "serial_consumer.h"
#include "task.h"
#include <stdio.h>

#define MOCK_PCAL_STACK_WORDS (configMINIMAL_STACK_SIZE * 2)

// Estado por defecto simulado:
// P0_0: SW_PLAY_STOP   = 1 (reposo, pull-up)
// P0_1: SW_BOLUS       = 1 (reposo, pull-up)
// P0_2: SW_DETECT_H    = 0 (jeringa detectada / switch ON)
// P1_1: CHRG_STATUS    = 0 (cargador presente)
// P1_2: BATT_INSTALL   = 0 (batería instalada)
// Demás pines con pull-up en 1
static volatile uint16_t s_mock_inputs = (1u << EXP_PIN_SW_PLAY_STOP) | (1u << EXP_PIN_SW_BOLUS);
static TaskHandle_t s_mock_task_handle = NULL;

void mock_pcal9555_set_pin(uint8_t pin, bool level) {
    if (pin >= 16) return;
    if (level) {
        s_mock_inputs |= (1u << pin);
    } else {
        s_mock_inputs &= ~(1u << pin);
    }
    CORE0_EMIT(EV_ACT_IO_EXP_INPUT, param, (uint32_t)s_mock_inputs);
}

static void task_mock_pcal9555(void *params) {
    (void)params;
    serial_log("SYS", "mock_pcal=INIT play=1 bolus=1 det=0 chrg=0 batt=0");

    // Emisión inicial para sincronizar el snapshot de los consumidores EDA
    CORE0_EMIT(EV_ACT_IO_EXP_INPUT, param, (uint32_t)s_mock_inputs);

    // En arquitectura EDA, las entradas son puramente por evento (cambio de estado).
    // La emisión inicial fija el estado base; cambios posteriores se notifican vía mock_pcal9555_set_pin().
    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}

bool mock_pcal9555_start_task(UBaseType_t priority) {
    if (s_mock_task_handle != NULL) return true;
    BaseType_t res = xTaskCreate(task_mock_pcal9555,
                                 "MockPCAL",
                                 MOCK_PCAL_STACK_WORDS,
                                 NULL,
                                 priority,
                                 &s_mock_task_handle);
    return (res == pdPASS);
}

#endif // ENABLE_MOCK_PCAL9555

