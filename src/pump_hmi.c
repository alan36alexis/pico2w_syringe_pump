#include "pump_hmi.h"
#include "crosscore_cmd.h"
#include "core1_main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

void pump_hmi_parse_and_execute(const char *str) {
    bool handled = true;
    bool ok      = false;

    if (strncmp(str, "stop_imm", 8) == 0 || strncmp(str, "STOP_IMM", 8) == 0 ||
        strncmp(str, "stop", 4)    == 0 || strncmp(str, "STOP", 4)    == 0) {
        ok = pump_hmi_execute(HMI_ACTION_STOP, 0.0f, 0.0f);

    } else if (strncmp(str, "fsm_home,", 9) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_HOME, (float)atof(str + 9), 0.0f);

    } else if (strncmp(str, "fsm_search,", 11) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_SEARCH_SYRINGE, (float)atof(str + 11), 0.0f);

    } else if (strncmp(str, "fsm_dispense,", 13) == 0) {
        float target = 0.0f, vel = 0.0f;
        if (sscanf(str + 13, "%f,%f", &target, &vel) == 2)
            ok = pump_hmi_execute(HMI_ACTION_START_DISPENSE, target, vel);
        else
            printf("[HMI] Error: uso -> fsm_dispense,TARGET_UM,VELOCITY_UMS\n");

    } else if (strncmp(str, "fsm_search_eot", 14) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_SEARCH_EOT, 0.0f, 0.0f);

    } else if (strncmp(str, "fsm_reset", 9) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_RESET, 0.0f, 0.0f);

    } else if (strncmp(str, "fsm_cont", 8) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_CONTINUE_DISPENSE, 0.0f, 0.0f);

    } else if (strncmp(str, "fsm_occ_rel", 11) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_OCC_RELEASE, 0.0f, 0.0f);

    } else if (strncmp(str, "fsm_resume", 10) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_RESUME, 0.0f, 0.0f);

    } else if (strncmp(str, "fsm_calibrate", 13) == 0) {
        ok = pump_hmi_execute(HMI_ACTION_CALIBRATE, 0.0f, 0.0f);

    } else {
        handled = false;
    }

    if (handled && !ok) {
        printf("[HMI] Comando rechazado en estado: %s\n",
               get_state_name(pump_hmi_get_fsm_state()));
    } else if (!handled) {
        // Comandos de bajo nivel (nsteps, move_linear, config_*, log_*, etc.)
        cmd_parse_and_execute(str);
    }
}
