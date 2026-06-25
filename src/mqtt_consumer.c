#include "mqtt_consumer.h"
#include "system_queues.h"
#include "mqtt_client.h"
#include "mqtt_topics.h"
#include <stdio.h>
#include <stdint.h>

#define MQTT_CONS_PRIORITY    2
#define MQTT_CONS_STACK_WORDS (configMINIMAL_STACK_SIZE * 6)

// Snapshot of the latest known device state, updated on each event.
typedef struct {
    float    pressure_psi;
    float    pressure_mmhg;
    int32_t  encoder_count;
    float    position_mm;
    float    progress_pct;
    float    encoder_speed_ums;
    uint32_t session_id;
    float    infused_volume_ml;
    float    target_volume_ml;
    float    rate_ml_h;
    uint32_t elapsed_s;
    uint16_t fsm_state;
    float    battery_voltage_v;
    uint8_t  battery_pct;
    uint32_t free_heap_bytes;
    uint8_t  mqtt_tx_drops;
} MqttSnapshot_t;

static MqttSnapshot_t s_snap = {0};

static void apply_event(const SystemEvent_t *ev) {
    switch (ev->id) {
    case EV_ACT_PRESSURE:
    case EV_ACT_PRESSURE_OCC:
        s_snap.pressure_psi  = ev->payload.force.psi;
        s_snap.pressure_mmhg = ev->payload.force.mmhg;
        break;
    case EV_MOT_ENCODER:
        s_snap.encoder_count = ev->payload.motion.encoder_count;
        s_snap.position_mm   = ev->payload.motion.position_mm;
        break;
    case EV_MOT_SPEED:
        s_snap.encoder_speed_ums = ev->payload.motion.encoder_speed_ums;
        break;
    case EV_MOT_PROGRESS:
        s_snap.progress_pct = ev->payload.motion.progress_pct;
        break;
    case EV_APP_FSM_STATE:
        s_snap.fsm_state  = ev->payload.fsm.state_to;
        s_snap.session_id = ev->payload.fsm.session_id;
        break;
    case EV_APP_SESSION_START:
    case EV_APP_SESSION_UPD:
    case EV_APP_SESSION_END:
        s_snap.session_id        = ev->payload.session.session_id;
        s_snap.infused_volume_ml = ev->payload.session.infused_volume_ml;
        s_snap.target_volume_ml  = ev->payload.session.target_volume_ml;
        s_snap.rate_ml_h         = ev->payload.session.rate_ml_h;
        s_snap.elapsed_s         = ev->payload.session.elapsed_s;
        break;
    case EV_PWR_BATTERY_UPD:
        s_snap.battery_voltage_v = ev->payload.power.battery_voltage_v;
        s_snap.battery_pct       = ev->payload.power.battery_pct;
        break;
    case EV_SYS_HEAP_UPD:
        s_snap.free_heap_bytes = ev->payload.sys_health.free_heap_bytes;
        s_snap.mqtt_tx_drops   = ev->payload.sys_health.mqtt_tx_drops;
        break;
    default:
        break;
    }
}

static void publish_sensors(void) {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"pressure_psi\":%.3f,\"pressure_mmhg\":%.1f}",
             s_snap.pressure_psi, s_snap.pressure_mmhg);
    mqtt_client_publish(topic_telemetry_sensors(), buf);
}

static void publish_motion(void) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"encoder_count\":%ld,\"position_mm\":%.2f,"
             "\"speed_ums\":%.1f,\"progress_pct\":%.1f}",
             (long)s_snap.encoder_count, s_snap.position_mm,
             s_snap.encoder_speed_ums, s_snap.progress_pct);
    mqtt_client_publish(topic_telemetry_motion(), buf);
}

static void publish_session(void) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"session_id\":%u,\"infused_ml\":%.2f,\"target_ml\":%.2f,"
             "\"rate_ml_h\":%.1f,\"elapsed_s\":%u}",
             (unsigned)s_snap.session_id,
             s_snap.infused_volume_ml, s_snap.target_volume_ml,
             s_snap.rate_ml_h, (unsigned)s_snap.elapsed_s);
    mqtt_client_publish(topic_telemetry_session(), buf);
}

static void publish_fsm_on_event(const SystemEvent_t *ev) {
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"state\",\"from\":%u,\"to\":%u}",
             (unsigned)ev->payload.fsm.state_from,
             (unsigned)ev->payload.fsm.state_to);
    mqtt_client_publish_qos1(topic_event(), buf);
}

static void task_mqtt_consumer(void *arg) {
    (void)arg;
    SystemEvent_t ev;

    TickType_t t_sensors = 0;
    TickType_t t_motion  = 0;
    TickType_t t_session = 0;
    TickType_t t_drops   = 0;
    uint32_t   last_drops = 0;

    for (;;) {
        // Short timeout drives rate-based cadence without burning CPU
        if (xQueueReceive(g_mqtt_q, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            apply_event(&ev);
            if (ev.id == EV_APP_FSM_STATE)
                publish_fsm_on_event(&ev);

            // Drain burst without blocking
            while (xQueueReceive(g_mqtt_q, &ev, 0) == pdTRUE) {
                apply_event(&ev);
                if (ev.id == EV_APP_FSM_STATE)
                    publish_fsm_on_event(&ev);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - t_sensors) >= pdMS_TO_TICKS(500)) {
            publish_sensors();
            t_sensors = now;
        }
        if ((now - t_motion) >= pdMS_TO_TICKS(1000)) {
            publish_motion();
            t_motion = now;
        }
        if ((now - t_session) >= pdMS_TO_TICKS(2000)) {
            publish_session();
            t_session = now;
        }
        if ((now - t_drops) >= pdMS_TO_TICKS(10000)) {
            uint32_t drops = mqtt_get_tx_drops();
            if (drops != last_drops) {
                CORE0_EMIT(EV_NET_MQTT_TX_DROP, param, drops);
                last_drops = drops;
            }
            t_drops = now;
        }
    }
}

void mqtt_consumer_start(void) {
    xTaskCreate(task_mqtt_consumer, "MqttCons", MQTT_CONS_STACK_WORDS,
                NULL, MQTT_CONS_PRIORITY, NULL);
}
