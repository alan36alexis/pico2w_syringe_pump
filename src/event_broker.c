#include "event_broker.h"
#include "system_queues.h"
#include "config_manager.h"
#include "syringe_pump_api.h"
#include "cmd_gate.h"
#include <stdio.h>

#define BROKER_PRIORITY    4
#define BROKER_STACK_WORDS (configMINIMAL_STACK_SIZE * 4)
#define BROKER_POLL_MS     500

// Pre-fanout side effects: update shared state before distributing to consumers.
// The broker sees every event unconditionally (no #ifdef guards), so state
// mirrors that all command sources need must be updated here — never inside
// a consumer that may not be running (e.g. hmi_consumer requires ENABLE_TFT).
static void broker_side_effects(const SystemEvent_t *ev) {
  if (ev->id == EV_ACT_PRESSURE || ev->id == EV_ACT_PRESSURE_OCC)
    Pump_UpdatePressure(ev->payload.force.mmhg);
  if (ev->id == EV_APP_FSM_STATE)
    cmd_gate_update_fsm_state((Core1State_t)ev->payload.fsm.state_to);
}

// Flash persistence pauses Core 1 (multicore lockout) for the whole sector
// erase+program. If that lands mid-motion it starves the DMA/PIO step
// generation (the LSW_END brake/release dies and the FSM hangs), so the
// calibration save must wait until the FSM sits in a motor-idle state.
static bool fsm_state_is_motion_idle(Core1State_t st) {
  switch (st) {
  case ST_UNHOMED:
  case ST_READY_AT_HOME:
  case ST_SYRINGE_ENGAGED:
  case ST_DISPENSE_COMPLETED:
  case ST_SET_NEW_DISPENSE:
  case ST_END_OF_TRAVEL:
  case ST_OCCLUSION_PAUSED:
  case ST_FAULT:
    return true;
  default:
    return false;
  }
}

// Uniform fan-out to all consumer queues.
// Drop silently if a consumer queue is full — acceptable for a logging system.
static void broker_fanout(const SystemEvent_t *ev) {
  broker_side_effects(ev);
  if (g_serial_q != NULL) xQueueSend(g_serial_q, ev, 0);
  if (g_mqtt_q   != NULL) xQueueSend(g_mqtt_q,   ev, 0);
  if (g_hmi_q    != NULL) xQueueSend(g_hmi_q,    ev, 0);
}

static void task_event_broker(void *arg) {
  (void)arg;
  SystemEvent_t ev;

  for (;;) {
    // Persist calibration if Core 1 flagged a new result. The dirty flag is
    // set the instant LSW_END hits (before braking/release finish), so the
    // write is deferred until the FSM reaches a motor-idle state.
    if (g_calibration_dirty &&
        fsm_state_is_motion_idle(cmd_gate_get_fsm_state())) {
      g_calibration_dirty = false;
      config_manager_save(true);
      CORE0_EMIT(EV_SYS_CALIBRATION_SAVED, param, 0);
    }

    // Drain Core 1 queue (non-blocking; hardware spinlock, safe from FreeRTOS task).
    // Core 1 sends with timestamp=0 — broker stamps here.
    while (queue_try_remove(&g_crosscore_event_q, &ev)) {
      ev.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
      broker_fanout(&ev);
    }

    // Drain Core 0 queue (non-blocking).
    // Core 0 events already carry a timestamp set by the emitter task.
    while (xQueueReceive(g_core0_event_q, &ev, 0) == pdTRUE) {
      broker_fanout(&ev);
    }

    // Suspend for a short tick to allow other tasks to run and avoid starvation
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void event_broker_start(void) {
  xTaskCreate(task_event_broker, "EvBroker", BROKER_STACK_WORDS, NULL,
              BROKER_PRIORITY, NULL);
}
