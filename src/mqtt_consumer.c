#include "mqtt_consumer.h"
#include "system_queues.h"
#include "mqtt_client.h"
#include "mqtt_topics.h"
#include "system_config.h"
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

    // Real-Time Hardware (Core 1)
    uint16_t frc_sens_raw;
    float    frc_sens_v;
    uint16_t bub_sens_raw;
    float    bub_sens_v;
    uint8_t  lsw_start;
    uint8_t  lsw_end;
    uint8_t  motor_ena;
    uint8_t  motor_dir;
    uint16_t motor_stallguard;

    // Real-Time Hardware (Core 0 - PCAL9555 & VIN)
    uint16_t vin_sens_raw;
    float    vin_sens_v;
    uint8_t  pcal_sw_play_stop;
    uint8_t  pcal_sw_bolus;
    uint8_t  pcal_sw_detect_h;
    uint8_t  pcal_chrg_status;
    uint8_t  pcal_batt_install;
} MqttSnapshot_t;

static MqttSnapshot_t s_snap = {
    .vin_sens_v = 15.58f,
    .vin_sens_raw = 1757,
    .bub_sens_v = 3.105f,
    .bub_sens_raw = 3852,
    .frc_sens_v = 0.735f,
    .frc_sens_raw = 911,
    .motor_stallguard = 240,
    .pcal_sw_play_stop = 1,
    .pcal_sw_bolus = 1,
    .pcal_sw_detect_h = 0,
    .pcal_chrg_status = 0,
    .pcal_batt_install = 0
};

static void apply_event(const SystemEvent_t *ev) {
    switch (ev->id) {
    case EV_HW_CORE1_STATUS:
        s_snap.frc_sens_raw     = ev->payload.core1_hw.frc_sens_raw;
        s_snap.frc_sens_v       = ev->payload.core1_hw.frc_sens_v;
        s_snap.bub_sens_raw     = ev->payload.core1_hw.bub_sens_raw;
        s_snap.bub_sens_v       = ev->payload.core1_hw.bub_sens_v;
        s_snap.lsw_start        = ev->payload.core1_hw.lsw_start;
        s_snap.lsw_end          = ev->payload.core1_hw.lsw_end;
        s_snap.motor_ena        = ev->payload.core1_hw.motor_ena;
        s_snap.motor_dir        = ev->payload.core1_hw.motor_dir;
        s_snap.motor_stallguard = ev->payload.core1_hw.motor_stallguard;
        {
            float p_psi = (ev->payload.core1_hw.frc_sens_v > 0.05f) ?
                          ev->payload.core1_hw.frc_sens_v * 1.5f : 0.0f;
            s_snap.pressure_psi  = p_psi;
            s_snap.pressure_mmhg = p_psi * 51.7149f;
        }
        break;
    case EV_ACT_IO_EXP_INPUT:
        {
            uint32_t p = ev->payload.param;
            s_snap.pcal_sw_play_stop = (p & (1u << EXP_PIN_SW_PLAY_STOP)) ? 1 : 0;
            s_snap.pcal_sw_bolus     = (p & (1u << EXP_PIN_SW_BOLUS)) ? 1 : 0;
            s_snap.pcal_sw_detect_h  = (p & (1u << EXP_PIN_SW_DETECT_H)) ? 1 : 0;
            s_snap.pcal_chrg_status  = (p & (1u << EXP_PIN_CHRG_STATUS)) ? 1 : 0;
            s_snap.pcal_batt_install = (p & (1u << EXP_PIN_BATT_INSTALL)) ? 1 : 0;
        }
        break;
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
        s_snap.vin_sens_v        = ev->payload.power.battery_voltage_v;
        s_snap.vin_sens_raw      = (uint16_t)((ev->payload.power.battery_voltage_v / 11.0f) * 4095.0f / 3.3f);
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

static void publish_realtime(void) {
    char buf[MQTT_MAX_PAYLOAD];
    const char *dir_str = "IDLE";
    if (s_snap.motor_ena) {
        dir_str = s_snap.motor_dir ? "FORWARD" : "REVERSE";
    }

    snprintf(buf, sizeof(buf),
             "{\"lsw\":{\"start\":%u,\"end\":%u},"
             "\"encoder\":{\"count\":%ld,\"position_mm\":%.2f},"
             "\"adc\":{\"frc_sens_v\":%.3f,\"frc_sens_raw\":%u,"
             "\"bub_sens_v\":%.3f,\"bub_sens_raw\":%u,"
             "\"vin_sens_v\":%.2f,\"vin_sens_raw\":%u},"
             "\"pcal9555\":{\"sw_play_stop\":%u,\"sw_bolus\":%u,"
             "\"sw_detect_h\":%u,\"chrg_status\":%u,\"batt_install\":%u},"
             "\"motor\":{\"ena\":%u,\"dir\":%u,\"dir_str\":\"%s\",\"stallguard\":%u}}",
             (unsigned)s_snap.lsw_start, (unsigned)s_snap.lsw_end,
             (long)s_snap.encoder_count, s_snap.position_mm,
             s_snap.frc_sens_v, (unsigned)s_snap.frc_sens_raw,
             s_snap.bub_sens_v, (unsigned)s_snap.bub_sens_raw,
             s_snap.vin_sens_v, (unsigned)s_snap.vin_sens_raw,
             (unsigned)s_snap.pcal_sw_play_stop, (unsigned)s_snap.pcal_sw_bolus,
             (unsigned)s_snap.pcal_sw_detect_h, (unsigned)s_snap.pcal_chrg_status,
             (unsigned)s_snap.pcal_batt_install,
             (unsigned)s_snap.motor_ena, (unsigned)s_snap.motor_dir, dir_str,
             (unsigned)s_snap.motor_stallguard);

    mqtt_client_publish(topic_telemetry_realtime(), buf);
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
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"session_id\":%u,\"infused_ml\":%.2f,\"target_ml\":%.2f,"
             "\"rate_ml_h\":%.1f,\"elapsed_s\":%u,\"fsm_state\":%u}",
             (unsigned)s_snap.session_id,
             s_snap.infused_volume_ml, s_snap.target_volume_ml,
             s_snap.rate_ml_h, (unsigned)s_snap.elapsed_s,
             (unsigned)s_snap.fsm_state);
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

static void publish_alarm_on_event(const SystemEvent_t *ev) {
    const char *code = "err";
    switch (ev->id) {
    case EV_ALARM_OCCLUSION:   code = "occ"; break;
    case EV_ALARM_BUBBLE:      code = "bub"; break;
    case EV_ALARM_EOT:         code = "eot"; break;
    case EV_ALARM_DRV_FAULT:   code = "drv"; break;
    case EV_ALARM_BATTERY_LOW: code = "bat"; break;
    case EV_ALARM_MAINS_LOST:  code = "mains"; break;
    default: break;
    }
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"alarm\",\"code\":\"%s\",\"level\":%u}",
             code, (unsigned)ev->payload.alarm.severity);
    mqtt_client_publish_qos1(topic_event(), buf);
}

static void task_mqtt_consumer(void *arg) {
    (void)arg;
    SystemEvent_t ev;

    TickType_t t_sensors  = 0;
    TickType_t t_realtime = 0;
    TickType_t t_motion   = 0;
    TickType_t t_session  = 0;

    for (;;) {
        // Short timeout drives rate-based cadence without burning CPU
        if (xQueueReceive(g_mqtt_q, &ev, pdMS_TO_TICKS(100)) == pdTRUE) {
            apply_event(&ev);
            if (ev.id == EV_APP_FSM_STATE)
                publish_fsm_on_event(&ev);
            else if (EV_IS_ALARM(ev.id))
                publish_alarm_on_event(&ev);

            // Drain burst without blocking
            while (xQueueReceive(g_mqtt_q, &ev, 0) == pdTRUE) {
                apply_event(&ev);
                if (ev.id == EV_APP_FSM_STATE)
                    publish_fsm_on_event(&ev);
                else if (EV_IS_ALARM(ev.id))
                    publish_alarm_on_event(&ev);
            }
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - t_sensors) >= pdMS_TO_TICKS(500)) {
            publish_sensors();
            t_sensors = now;
        }
        if ((now - t_realtime) >= pdMS_TO_TICKS(500)) {
            publish_realtime();
            t_realtime = now;
        }
        if ((now - t_motion) >= pdMS_TO_TICKS(1000)) {
            publish_motion();
            t_motion = now;
        }
        if ((now - t_session) >= pdMS_TO_TICKS(2000)) {
            publish_session();
            t_session = now;
        }

    }
}

void mqtt_consumer_start(void) {
    xTaskCreate(task_mqtt_consumer, "MqttCons", MQTT_CONS_STACK_WORDS,
                NULL, MQTT_CONS_PRIORITY, NULL);
}
