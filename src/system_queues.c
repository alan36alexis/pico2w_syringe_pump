#include "system_queues.h"

// g_crosscore_event_q: also declared extern in system_events.h for CORE1_EMIT.
// This translation unit is the single owner of its storage.
queue_t       g_crosscore_event_q;

QueueHandle_t g_core0_event_q = NULL;
QueueHandle_t g_serial_q      = NULL;
QueueHandle_t g_mqtt_q        = NULL;
QueueHandle_t g_hmi_q         = NULL;

void system_queues_init(void) {
    queue_init(&g_crosscore_event_q, sizeof(SystemEvent_t), 32);

    g_core0_event_q = xQueueCreate(24, sizeof(SystemEvent_t));
    g_serial_q      = xQueueCreate(32, sizeof(SystemEvent_t));
    g_mqtt_q        = xQueueCreate(32, sizeof(SystemEvent_t));

#ifdef ENABLE_TFT
    g_hmi_q = xQueueCreate(16, sizeof(SystemEvent_t));
#endif
    // g_hmi_q remains NULL when !ENABLE_TFT; broker checks NULL before sending.

    configASSERT(g_core0_event_q);
    configASSERT(g_serial_q);
    configASSERT(g_mqtt_q);
}
