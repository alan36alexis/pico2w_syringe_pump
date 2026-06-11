#ifndef UI_STATE_H
#define UI_STATE_H

#include "core1_main.h"
#include "syringe_pump_api.h"
#include "crosscore_logger.h"

// Snapshot del estado de la bomba para consumo exclusivo de task_tft (LVGL).
// Se actualiza desde task_logger; se lee desde task_tft via ui_state_get_snapshot().
typedef struct {
    Core1State_t fsm_state;
    float        pressure_mmhg;
    float        progress_pct;
    float        rate_ml_h;
    float        infused_volume_ml;
    float        target_volume_ml;
    PumpAlarms_t alarms;
} UIState_t;

// Inicializar (llamar antes del scheduler).
void ui_state_init(void);

// Actualizar el estado a partir de un evento de Core 1.
// Llamar desde task_logger por cada LogMessage_t recibido.
void ui_state_update_from_event(const LogMessage_t *msg);

// Obtener una copia consistente del estado (thread-safe).
// Llamar desde task_tft en cada tick LVGL.
UIState_t ui_state_get_snapshot(void);

#endif // UI_STATE_H
