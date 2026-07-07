#include "cmd_gate.h"
#include "crosscore_cmd.h"
#include "core1_main.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define DEFAULT_HOME_VEL_UMS    1500.0f
#define DEFAULT_SEARCH_VEL_UMS   200.0f

static volatile Core1State_t s_fsm_state = ST_UNHOMED;
static SemaphoreHandle_t s_mutex = NULL;

void cmd_gate_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    s_fsm_state = ST_UNHOMED;
}

void cmd_gate_update_fsm_state(Core1State_t state) {
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_fsm_state = state;
        xSemaphoreGive(s_mutex);
    }
}

Core1State_t cmd_gate_get_fsm_state(void) {
    Core1State_t st = ST_UNHOMED;
    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        st = s_fsm_state;
        xSemaphoreGive(s_mutex);
    }
    return st;
}

const PumpContext_t *cmd_gate_get_context(void) {
    return Pump_GetContext();
}

bool cmd_gate_execute(PumpAction_t action, float param1, float param2) {
    Core1State_t st = cmd_gate_get_fsm_state();

    switch (action) {

    case PUMP_ACTION_HOME:
        /* Desde ST_FAULT se exige fsm_reset primero (la tabla ignora HOME ahí). */
        if (st == ST_UNHOMED || st == ST_READY_AT_HOME ||
            st == ST_DISPENSE_COMPLETED || st == ST_END_OF_TRAVEL)
            return cmd_send_home(param1 > 0.0f ? param1 : DEFAULT_HOME_VEL_UMS);
        break;

    case PUMP_ACTION_SEARCH_SYRINGE:
        if (st == ST_READY_AT_HOME)
            return cmd_send_search_syringe(param1 > 0.0f ? param1 : DEFAULT_SEARCH_VEL_UMS);
        break;

    case PUMP_ACTION_START_DISPENSE:
        if (st == ST_SYRINGE_ENGAGED)
            return cmd_send_start_dispense(param1, param2);
        break;

    case PUMP_ACTION_STOP:
        return cmd_send_stop_immediate();

    case PUMP_ACTION_OCC_RELEASE:
        if (st == ST_OCCLUSION_PAUSED)
            return cmd_send_occ_release();
        break;

    case PUMP_ACTION_RESUME:
        if (st == ST_OCCLUSION_PAUSED)
            return cmd_send_resume_dispense();
        break;

    case PUMP_ACTION_CONTINUE_DISPENSE:
        if (st == ST_DISPENSE_COMPLETED || st == ST_SET_NEW_DISPENSE)
            return cmd_send_continue_dispense();
        break;

    case PUMP_ACTION_RESET:
        return cmd_send_reset();

    case PUMP_ACTION_SEARCH_EOT:
        if (st == ST_SYRINGE_ENGAGED || st == ST_DISPENSE_COMPLETED)
            return cmd_send_search_eot();
        break;

    case PUMP_ACTION_CALIBRATE:
        /* Desde ST_FAULT se exige fsm_reset primero (la tabla ignora CALIBRATE ahí). */
        if (st == ST_UNHOMED || st == ST_READY_AT_HOME)
            return cmd_send_calibrate(param1, param2);
        break;

    default:
        break;
    }

    return false;
}
