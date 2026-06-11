#include "ui_state.h"
#include "pump_hmi.h"
#include "FreeRTOS.h"
#include "semphr.h"

static UIState_t    s_state  = {0};
static SemaphoreHandle_t s_mutex = NULL;

void ui_state_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    s_state.fsm_state = ST_UNHOMED;
}

void ui_state_update_from_event(const LogMessage_t *msg) {
    if (!s_mutex)
        return;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return;

    switch (msg->id) {
    case LOG_EVENT_FSM_STATE:
        s_state.fsm_state = msg->payload.fsm_state;
        // Sincronizar con la capa HMI para que valide comandos correctamente
        pump_hmi_update_fsm_state(msg->payload.fsm_state);
        break;
    case LOG_EVENT_PRESSURE_UPDATE:
        s_state.pressure_mmhg = msg->payload.pressure_psi * 51.7149f;
        break;
    case LOG_EVENT_MOTOR_PROGRESS:
        s_state.progress_pct = msg->payload.progress_pct;
        break;
    default:
        break;
    }

    xSemaphoreGive(s_mutex);
}

UIState_t ui_state_get_snapshot(void) {
    UIState_t snap = {0};
    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        snap = s_state;
        xSemaphoreGive(s_mutex);
    }
    return snap;
}
