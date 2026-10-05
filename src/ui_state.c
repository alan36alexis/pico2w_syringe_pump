#include "ui_state.h"
#include "cmd_gate.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "system_events.h"

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
        cmd_gate_update_fsm_state(msg->payload.fsm_state);
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

void ui_state_update_from_system_event(const SystemEvent_t *ev) {
    if (!s_mutex) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    switch (ev->id) {
    case EV_APP_FSM_STATE:
        s_state.fsm_state = (Core1State_t)ev->payload.fsm.state_to;
        cmd_gate_update_fsm_state((Core1State_t)ev->payload.fsm.state_to);
        break;
    case EV_ACT_PRESSURE:
    case EV_ACT_PRESSURE_OCC:
        s_state.pressure_mmhg = ev->payload.force.mmhg;
        break;
    case EV_MOT_PROGRESS:
        s_state.progress_pct = ev->payload.motion.progress_pct;
        break;
    case EV_APP_SESSION_START:
    case EV_APP_SESSION_UPD:
    case EV_APP_SESSION_END: {
        const DtoSession_t *session = &ev->payload.session;
        s_state.session_id = session->session_id;
        s_state.target_volume_ml = session->target_volume_ml;
        s_state.infused_volume_ml = session->infused_volume_ml;
        s_state.rate_ml_h = session->rate_ml_h;
        s_state.elapsed_s = session->elapsed_s;
        s_state.session_active = ev->id != EV_APP_SESSION_END;
        s_state.session_data_valid = true;

        if (session->target_volume_ml > 0.0f) {
            s_state.progress_pct =
                session->infused_volume_ml * 100.0f / session->target_volume_ml;
            if (s_state.progress_pct < 0.0f) s_state.progress_pct = 0.0f;
            if (s_state.progress_pct > 100.0f) s_state.progress_pct = 100.0f;
        }
        break;
    }
    case EV_ALARM_OCCLUSION:
    case EV_ALARM_EOT:
    case EV_ALARM_DRV_FAULT:
    case EV_ALARM_BATTERY_LOW:
    case EV_ALARM_MAINS_LOST:
        s_state.last_alarm_id = ev->id;
        s_state.last_alarm_severity = ev->payload.alarm.severity;
        s_state.last_alarm_fsm_state = ev->payload.alarm.fsm_state;
        s_state.last_alarm_param = ev->payload.alarm.param_f;
        s_state.alarm_event_count++;
        if (ev->id == EV_ALARM_OCCLUSION) s_state.alarms.occlusion = true;
        if (ev->id == EV_ALARM_DRV_FAULT) s_state.alarms.system_error = true;
        break;
    default:
        break;
    }

    xSemaphoreGive(s_mutex);
}
