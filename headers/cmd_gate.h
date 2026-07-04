/**
 * @file cmd_gate.h
 * @brief Command gate — validación de comandos compartida por todos los medios
 *        generadores de comandos.
 *
 * ROL
 * ---
 * Este módulo es el ÚNICO punto de entrada para cualquier comando de movimiento
 * que provenga de cualquier interfaz (serial, MQTT, TFT/touch/teclas o futura).
 * Ningún generador de comandos debe escribir directamente en crosscore_cmd_queue.
 *
 * Responsabilidades:
 *   1. Mantiene un espejo del estado FSM de Core 1 (s_fsm_state).
 *   2. Valida que el estado actual permita la acción solicitada antes de
 *      encolarla en la crosscore_cmd_queue hacia Core 1.
 *   3. Es AGNÓSTICO al medio: no sabe ni le importa si el comando viene de
 *      serial, MQTT o TFT.
 *
 * ACTUALIZACIÓN DEL ESTADO FSM
 * ----------------------------
 * cmd_gate_update_fsm_state() es la ÚNICA función que escribe s_fsm_state.
 * Debe ser llamada EXCLUSIVAMENTE desde event_broker.c :: broker_side_effects()
 * al recibir EV_APP_FSM_STATE. Así la actualización es incondicional (no depende
 * de que ENABLE_TFT u otra feature esté activa).
 *
 * DIRECTIVAS PARA FUTUROS PROMPTS
 * ---------------------------------
 * - Para agregar un nuevo medio generador (e.g. BLE): crear su consumer/parser
 *   y llamar cmd_gate_execute() al final — nada más.
 * - Para agregar una nueva acción FSM: añadir valor a PumpAction_t, añadir
 *   un case en cmd_gate_execute() con la validación de estado correspondiente,
 *   y agregar el cmd_send_*() en crosscore_cmd.c/.h.
 * - NUNCA llamar cmd_gate_update_fsm_state() desde consumidores específicos
 *   de interfaz (serial_consumer, mqtt_consumer, hmi_consumer). Solo desde
 *   el broker.
 * - NUNCA incluir lógica de display o protocolo de red aquí.
 */

#ifndef CMD_GATE_H
#define CMD_GATE_H

#include <stdbool.h>
#include "core1_main.h"
#include "syringe_pump_api.h"

/**
 * Acciones de bomba que cualquier interfaz puede solicitar.
 * cmd_gate_execute() valida el estado FSM antes de despachar a Core 1.
 */
typedef enum {
    PUMP_ACTION_HOME = 0,
    PUMP_ACTION_SEARCH_SYRINGE,
    PUMP_ACTION_START_DISPENSE,    /**< param1 = target_um, param2 = velocity_ums */
    PUMP_ACTION_STOP,              /**< siempre válido, no valida estado */
    PUMP_ACTION_OCC_RELEASE,
    PUMP_ACTION_RESUME,
    PUMP_ACTION_CONTINUE_DISPENSE,
    PUMP_ACTION_RESET,             /**< siempre válido, no valida estado */
    PUMP_ACTION_SEARCH_EOT,
    PUMP_ACTION_CALIBRATE,
} PumpAction_t;

/** Debe llamarse antes de iniciar el scheduler FreeRTOS. */
void cmd_gate_init(void);

/**
 * Despacha una acción validando el estado actual del FSM.
 * @param param1, param2  Parámetros opcionales según la acción (ver enum).
 * @return true si el comando fue aceptado y encolado; false si el estado FSM
 *         actual no lo permite.
 */
bool cmd_gate_execute(PumpAction_t action, float param1, float param2);

/**
 * Actualiza el espejo de estado FSM.
 * Llamar SOLO desde event_broker.c :: broker_side_effects().
 */
void cmd_gate_update_fsm_state(Core1State_t state);

/** Retorna el último estado FSM conocido de Core 1 (thread-safe). */
Core1State_t cmd_gate_get_fsm_state(void);

/** Retorna el contexto de la bomba (delega a Pump_GetContext). */
const PumpContext_t *cmd_gate_get_context(void);

#endif /* CMD_GATE_H */
