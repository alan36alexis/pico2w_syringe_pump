#pragma once

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system_events.h"

// Core1 → Broker (Pico SDK hardware-spinlock queue, cross-core safe)
// Declared extern in system_events.h for CORE1_EMIT (no FreeRTOS dependency).
// Defined in system_queues.c. Capacity: 32 items.

// Core0 tasks → Broker (FreeRTOS queue)
extern QueueHandle_t g_core0_event_q;   // capacity 24

// Broker → Consumers (FreeRTOS queues)
extern QueueHandle_t g_serial_q;        // capacity 32
extern QueueHandle_t g_mqtt_q;          // capacity 32
extern QueueHandle_t g_hmi_q;          // capacity 16, or NULL if !ENABLE_TFT

// Must be called before vTaskStartScheduler().
// Initializes g_crosscore_event_q and all FreeRTOS queues.
void system_queues_init(void);

// Core 0 emission macro: emitter stamps the timestamp.
// Must be called from FreeRTOS task context (not from ISR or Core 1).
// Non-blocking: drops silently if g_core0_event_q is full.
#define CORE0_EMIT(event_id, dto_field, dto_value) do {  \
    SystemEvent_t _ev = {                                  \
        .timestamp_ms      = xTaskGetTickCount()           \
                             * portTICK_PERIOD_MS,         \
        .id                = (event_id),                   \
        .payload.dto_field = (dto_value)                   \
    };                                                     \
    xQueueSend(g_core0_event_q, &_ev, 0);                 \
} while (0)
