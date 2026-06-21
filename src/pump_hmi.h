#ifndef PUMP_HMI_H
#define PUMP_HMI_H

#include <stdbool.h>
#include "core1_main.h"
#include "syringe_pump_api.h"

// Acciones que cualquier interfaz (CLI, MQTT, TFT) puede solicitar.
// pump_hmi valida el estado actual antes de despachar a Core 1.
typedef enum {
    HMI_ACTION_HOME = 0,
    HMI_ACTION_SEARCH_SYRINGE,
    HMI_ACTION_START_DISPENSE,  // param1 = target_um, param2 = velocity_ums
    HMI_ACTION_STOP,            // siempre valida
    HMI_ACTION_OCC_RELEASE,
    HMI_ACTION_RESUME,
    HMI_ACTION_CONTINUE_DISPENSE,
    HMI_ACTION_RESET,
    HMI_ACTION_SEARCH_EOT,
    HMI_ACTION_CALIBRATE,
} PumpHMIAction_t;

// Debe llamarse antes de iniciar el scheduler FreeRTOS.
void pump_hmi_init(void);

// Despacha una acción validando el estado actual del FSM.
// param1 y param2: parámetros opcionales según la acción (ver enum arriba).
// Retorna false si el comando no es válido en el estado actual.
bool pump_hmi_execute(PumpHMIAction_t action, float param1, float param2);

// Llamado por task_logger cada vez que Core 1 reporta una transición de estado.
void pump_hmi_update_fsm_state(Core1State_t state);

// Retorna el último estado FSM conocido de Core 1.
Core1State_t pump_hmi_get_fsm_state(void);

// Retorna el contexto de la bomba (delega a Pump_GetContext).
const PumpContext_t *pump_hmi_get_context(void);

// Punto de entrada unificado para CLI y MQTT.
// Comandos fsm_* se validan contra el estado actual antes de ejecutar.
// Comandos de bajo nivel (nsteps, move_linear, config_*) pasan directo a
// cmd_parse_and_execute para mantener compatibilidad.
bool pump_hmi_parse_and_execute(const char *str);

#endif // PUMP_HMI_H
