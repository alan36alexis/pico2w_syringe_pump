#include "ui_events.h"
#include "FreeRTOS.h"
#include "queue.h"

QueueHandle_t ui_event_queue = NULL;

void ui_events_init(void) {
    ui_event_queue = xQueueCreate(UI_EVENT_QUEUE_DEPTH, sizeof(UIEvent_t));
    configASSERT(ui_event_queue != NULL);
}

bool ui_event_send(const UIEvent_t *event) {
    return xQueueSend(ui_event_queue, event, 0) == pdTRUE;
}

bool ui_event_send_from_isr(const UIEvent_t        *event,
                             BaseType_t             *pxHigherPriorityTaskWoken) {
    return xQueueSendFromISR(ui_event_queue, event, pxHigherPriorityTaskWoken) == pdTRUE;
}
