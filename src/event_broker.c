#include "event_broker.h"
#include "system_queues.h"
#include "config_manager.h"
#include "syringe_pump_api.h"
#include <stdio.h>

#define BROKER_PRIORITY    4
#define BROKER_STACK_WORDS (configMINIMAL_STACK_SIZE * 4)
#define BROKER_POLL_MS     500

// Pre-fanout side effects: update clinical API state before distributing.
// The broker is the only place that sees every event, so these fit here.
static void broker_side_effects(const SystemEvent_t *ev) {
  if (ev->id == EV_ACT_PRESSURE || ev->id == EV_ACT_PRESSURE_OCC)
    Pump_UpdatePressure(ev->payload.force.mmhg);
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
    // Persist calibration if Core 1 flagged a new result
    if (g_calibration_dirty) {
      g_calibration_dirty = false;
      config_manager_save(true);
      printf("[CFG]: Calibration saved to Flash.\n");
    }

    // Drain Core 1 queue (non-blocking; hardware spinlock, safe from FreeRTOS task).
    // Core 1 sends with timestamp=0 — broker stamps here.
    while (queue_try_remove(&g_crosscore_event_q, &ev)) {
      ev.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
      broker_fanout(&ev);
    }

    // Block on Core 0 queue with timeout (acts as sleep period).
    // Core 0 events already carry a timestamp set by the emitter task.
    if (xQueueReceive(g_core0_event_q, &ev, pdMS_TO_TICKS(BROKER_POLL_MS)) == pdTRUE) {
      broker_fanout(&ev);
      while (xQueueReceive(g_core0_event_q, &ev, 0) == pdTRUE)
        broker_fanout(&ev);
    }
  }
}

void event_broker_start(void) {
  xTaskCreate(task_event_broker, "EvBroker", BROKER_STACK_WORDS, NULL,
              BROKER_PRIORITY, NULL);
}
