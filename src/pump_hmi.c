#include "pump_hmi.h"
#include "crosscore_cmd.h"
#include "core1_main.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Velocidades por defecto para comandos que no reciben param1
#define HMI_DEFAULT_HOME_VEL_UMS    1500.0f
#define HMI_DEFAULT_SEARCH_VEL_UMS   200.0f

static volatile Core1State_t s_fsm_state = ST_UNHOMED;
static SemaphoreHandle_t s_mutex = NULL;

void pump_hmi_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    s_fsm_state = ST_UNHOMED;
}

void pump_hmi_update_fsm_state(Core1State_t state) {
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_fsm_state = state;
        xSemaphoreGive(s_mutex);
    }
}

Core1State_t pump_hmi_get_fsm_state(void) {
    Core1State_t st = ST_UNHOMED;
    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        st = s_fsm_state;
        xSemaphoreGive(s_mutex);
    }
    return st;
}

const PumpContext_t *pump_hmi_get_context(void) {
    return Pump_GetContext();
}

bool pump_hmi_execute(PumpHMIAction_t action, float param1, float param2) {
    Core1State_t st = pump_hmi_get_fsm_state();

    switch (action) {

    case HMI_ACTION_HOME:
        if (st == ST_UNHOMED || st == ST_READY_AT_HOME ||
            st == ST_DISPENSE_COMPLETED || st == ST_END_OF_TRAVEL || st == ST_FAULT)
            return cmd_send_home(param1 > 0.0f ? param1 : HMI_DEFAULT_HOME_VEL_UMS);
        break;

    case HMI_ACTION_SEARCH_SYRINGE:
        if (st == ST_READY_AT_HOME)
            return cmd_send_search_syringe(param1 > 0.0f ? param1 : HMI_DEFAULT_SEARCH_VEL_UMS);
        break;

    case HMI_ACTION_START_DISPENSE:
        if (st == ST_SYRINGE_ENGAGED)
            return cmd_send_start_dispense(param1, param2);
        break;

    case HMI_ACTION_STOP:
        return cmd_send_stop_immediate();

    case HMI_ACTION_OCC_RELEASE:
        if (st == ST_OCCLUSION_PAUSED)
            return cmd_send_occ_release();
        break;

    case HMI_ACTION_RESUME:
        if (st == ST_OCCLUSION_PAUSED)
            return cmd_send_resume_dispense();
        break;

    case HMI_ACTION_CONTINUE_DISPENSE:
        if (st == ST_DISPENSE_COMPLETED || st == ST_SET_NEW_DISPENSE)
            return cmd_send_continue_dispense();
        break;

    case HMI_ACTION_RESET:
        return cmd_send_reset();

    case HMI_ACTION_SEARCH_EOT:
        if (st == ST_SYRINGE_ENGAGED || st == ST_DISPENSE_COMPLETED)
            return cmd_send_search_eot();
        break;

    case HMI_ACTION_CALIBRATE:
        if (st == ST_UNHOMED || st == ST_READY_AT_HOME || st == ST_FAULT)
            return cmd_send_calibrate();
        break;

    default:
        break;
    }

    return false;
}

