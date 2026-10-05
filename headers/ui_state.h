#ifndef UI_STATE_H
#define UI_STATE_H

#include "core1_main.h"
#include "syringe_pump_api.h"
#include "crosscore_logger.h"
#include "system_events.h"

// Snapshot del estado de la bomba para consumo exclusivo de task_tft (LVGL).
// Se actualiza desde task_logger; se lee desde task_tft via ui_state_get_snapshot().
typedef struct {
    Core1State_t fsm_state;
    float        pressure_mmhg;
    float        progress_pct;
    float        rate_ml_h;
    float        infused_volume_ml;
    float        target_volume_ml;
    uint32_t     session_id;
    uint32_t     elapsed_s;
    bool         session_active;
    bool         session_data_valid;
    uint16_t     last_alarm_id;
    uint8_t      last_alarm_severity;
    uint16_t     last_alarm_fsm_state;
    float        last_alarm_param;
    uint32_t     alarm_event_count;
    PumpAlarms_t alarms;
} UIState_t;

// Inicializar (llamar antes del scheduler).
void ui_state_init(void);

// Actualizar el estado a partir de un evento legado (LogMessage_t).
void ui_state_update_from_event(const LogMessage_t *msg);

// Actualizar el estado a partir de un evento EDA (SystemEvent_t).
// Llamar desde hmi_consumer por cada evento recibido.
void ui_state_update_from_system_event(const SystemEvent_t *ev);

// Obtener una copia consistente del estado (thread-safe).
// Llamar desde task_tft en cada tick LVGL.
UIState_t ui_state_get_snapshot(void);

#endif // UI_STATE_H
