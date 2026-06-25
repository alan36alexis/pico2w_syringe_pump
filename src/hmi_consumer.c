#include "hmi_consumer.h"
#include "system_queues.h"
#include "ui_state.h"

#define HMI_CONS_PRIORITY    2
#define HMI_CONS_STACK_WORDS (configMINIMAL_STACK_SIZE * 4)

// Returns true for events that are relevant to the clinical UI.
// Discards: EV_TMC_*, EV_MOT_ENCODER, EV_MOT_SPEED, EV_SYS_*, EV_NET_MQTT_TX_DROP.
static int is_hmi_relevant(SystemEventID_t id) {
    if (id == EV_APP_FSM_STATE)                               return 1;
    if (id >= EV_APP_SESSION_START && id <= EV_APP_SESSION_UPD) return 1;
    if (id == EV_ACT_PRESSURE || id == EV_ACT_PRESSURE_OCC)  return 1;
    if (EV_IS_ALARM(id))                                      return 1;
    if (id == EV_PWR_BATTERY_UPD || id == EV_PWR_MAINS_DETECT) return 1;
    return 0;
}

// Dispatches a relevant event to the appropriate LVGL widget.
// Widget calls are stubs — TFT team fills in during LVGL integration.
static void dispatch_to_ui(const SystemEvent_t *ev) {
    switch (ev->id) {
    case EV_APP_FSM_STATE:
        // TODO: update status bar label and action button states
        break;
    case EV_APP_SESSION_START:
    case EV_APP_SESSION_UPD:
    case EV_APP_SESSION_END:
        // TODO: update volume bar, rate label, elapsed timer widget
        break;
    case EV_ACT_PRESSURE:
        // TODO: update pressure gauge arc widget
        break;
    case EV_ACT_PRESSURE_OCC:
        // TODO: show occlusion alarm banner (high severity)
        break;
    case EV_PWR_BATTERY_UPD:
    case EV_PWR_MAINS_DETECT:
        // TODO: update battery icon and mains indicator
        break;
    default:
        if (EV_IS_ALARM(ev->id)) {
            // TODO: show general alarm banner with severity level
        }
        break;
    }
}

static void task_hmi_consumer(void *arg) {
    (void)arg;
    SystemEvent_t ev;

    for (;;) {
        if (xQueueReceive(g_hmi_q, &ev, portMAX_DELAY) == pdTRUE) {
            // Update clinical UI state mirror for all events (task_tft reads via ui_state_get_snapshot)
            ui_state_update_from_system_event(&ev);
            if (is_hmi_relevant(ev.id))
                dispatch_to_ui(&ev);
        }
    }
}

void hmi_consumer_start(void) {
    if (g_hmi_q == NULL)
        return;
    xTaskCreate(task_hmi_consumer, "HmiCons", HMI_CONS_STACK_WORDS,
                NULL, HMI_CONS_PRIORITY, NULL);
}
