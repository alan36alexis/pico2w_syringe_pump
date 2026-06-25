#include "serial_consumer.h"
#include "system_queues.h"
#include "core1_main.h"
#include <stdio.h>

#define SERIAL_PRIORITY    1
#define SERIAL_STACK_WORDS (configMINIMAL_STACK_SIZE * 4)

// The ONLY place in the firmware where SystemEventID_t values are mapped to
// human-readable strings. All other modules emit typed events — no strings.
static void format_event(const SystemEvent_t *ev, char *buf, size_t len) {
    switch (ev->id) {

    // TMC2209 Driver
    case EV_TMC_DRV_STATUS:
        snprintf(buf, len,
                 "[%6u] [TMC]: DRV=0x%08X GSTAT=0x%08X stall=%u flags=0x%02X",
                 ev->timestamp_ms,
                 (unsigned)ev->payload.tmc.drv_status_raw,
                 (unsigned)ev->payload.tmc.gstat_raw,
                 ev->payload.tmc.stall_count,
                 ev->payload.tmc.flags);
        break;
    case EV_TMC_STALL:
        snprintf(buf, len,
                 "[%6u] [TMC]: Stall count=%u DRV=0x%08X",
                 ev->timestamp_ms,
                 ev->payload.tmc.stall_count,
                 (unsigned)ev->payload.tmc.drv_status_raw);
        break;
    case EV_TMC_OVERTEMP:
        snprintf(buf, len,
                 "[%6u] [TMC]: Sobretemperatura flags=0x%02X",
                 ev->timestamp_ms, ev->payload.tmc.flags);
        break;

    // Actuators / Sensors
    case EV_ACT_PRESSURE:
        snprintf(buf, len,
                 "[%6u] [PRS]: %.2f psi (%.1f mmHg)",
                 ev->timestamp_ms,
                 ev->payload.force.psi, ev->payload.force.mmhg);
        break;
    case EV_ACT_PRESSURE_OCC:
        snprintf(buf, len,
                 "[%6u] [PRS]: OCLUSION %.2f psi (%.1f mmHg)",
                 ev->timestamp_ms,
                 ev->payload.force.psi, ev->payload.force.mmhg);
        break;
    case EV_ACT_LSW_END:
        snprintf(buf, len,
                 "[%6u] [ACT]: Limit switch END alcanzado",
                 ev->timestamp_ms);
        break;
    case EV_ACT_SYRINGE_DET:
        snprintf(buf, len,
                 "[%6u] [ACT]: Jeringa %s",
                 ev->timestamp_ms,
                 ev->payload.param ? "enganchada" : "removida");
        break;

    // Motion
    case EV_MOT_ENCODER:
        snprintf(buf, len,
                 "[%6u] [ENC]: count=%d pos=%.2f mm",
                 ev->timestamp_ms,
                 ev->payload.motion.encoder_count,
                 ev->payload.motion.position_mm);
        break;
    case EV_MOT_SPEED:
        snprintf(buf, len,
                 "[%6u] [MOT]: vel=%.1f um/s target=%.1f um/s",
                 ev->timestamp_ms,
                 ev->payload.motion.encoder_speed_ums,
                 ev->payload.motion.target_speed_ums);
        break;
    case EV_MOT_PROGRESS:
        snprintf(buf, len,
                 "[%6u] [MOT]: progreso=%.1f%%",
                 ev->timestamp_ms, ev->payload.motion.progress_pct);
        break;
    case EV_MOT_CORRECTION:
        snprintf(buf, len,
                 "[%6u] [ENC]: correccion=%.2f um",
                 ev->timestamp_ms, ev->payload.motion.correction_um);
        break;
    case EV_MOT_SPEED_WARN:
        snprintf(buf, len,
                 "[%6u] [ENC]: WARNING vel esperada=%.1f actual=%.1f um/s",
                 ev->timestamp_ms,
                 ev->payload.motion.target_speed_ums,
                 ev->payload.motion.encoder_speed_ums);
        break;

    // Network
    case EV_NET_WIFI_CONNECTING:
        snprintf(buf, len, "[%6u] [NET]: Connecting to Wi-Fi...", ev->timestamp_ms);
        break;
    case EV_NET_WIFI_CONN:
        snprintf(buf, len,
                 "[%6u] [NET]: WiFi conectado RSSI=%d dBm canal=%u",
                 ev->timestamp_ms,
                 ev->payload.wifi.rssi_dbm, ev->payload.wifi.channel);
        break;
    case EV_NET_WIFI_DISC:
        snprintf(buf, len,
                 "[%6u] [NET]: WiFi desconectado reason=%u",
                 ev->timestamp_ms, (unsigned)ev->payload.param);
        break;
    case EV_NET_MQTT_CONN:
        snprintf(buf, len, "[%6u] [NET]: MQTT conectado", ev->timestamp_ms);
        break;
    case EV_NET_MQTT_DISC:
        snprintf(buf, len, "[%6u] [NET]: MQTT desconectado", ev->timestamp_ms);
        break;
    case EV_NET_MQTT_TX_DROP:
        snprintf(buf, len,
                 "[%6u] [NET]: MQTT TX drops=%u",
                 ev->timestamp_ms, (unsigned)ev->payload.param);
        break;

    // FSM / Application
    case EV_APP_FSM_STATE:
        snprintf(buf, len,
                 "[%6u] [FSM]: %s -> %s  sid=%u",
                 ev->timestamp_ms,
                 get_state_name((Core1State_t)ev->payload.fsm.state_from),
                 get_state_name((Core1State_t)ev->payload.fsm.state_to),
                 (unsigned)ev->payload.fsm.session_id);
        break;
    case EV_APP_SESSION_START:
        snprintf(buf, len,
                 "[%6u] [APP]: Sesion iniciada sid=%u vol=%.1f ml rate=%.2f ml/h diam=%.1f mm",
                 ev->timestamp_ms,
                 (unsigned)ev->payload.session.session_id,
                 ev->payload.session.target_volume_ml,
                 ev->payload.session.rate_ml_h,
                 ev->payload.session.syringe_diam_mm);
        break;
    case EV_APP_SESSION_END:
        snprintf(buf, len,
                 "[%6u] [APP]: Sesion terminada sid=%u vol=%.2f ml elapsed=%u s",
                 ev->timestamp_ms,
                 (unsigned)ev->payload.session.session_id,
                 ev->payload.session.infused_volume_ml,
                 (unsigned)ev->payload.session.elapsed_s);
        break;
    case EV_APP_SESSION_UPD:
        snprintf(buf, len,
                 "[%6u] [APP]: sid=%u vol=%.2f/%.1f ml %u s",
                 ev->timestamp_ms,
                 (unsigned)ev->payload.session.session_id,
                 ev->payload.session.infused_volume_ml,
                 ev->payload.session.target_volume_ml,
                 (unsigned)ev->payload.session.elapsed_s);
        break;
    case EV_APP_CMD_EXECUTED:
        snprintf(buf, len,
                 "[%6u] [APP]: Cmd src=%u action=%u %s",
                 ev->timestamp_ms,
                 ev->payload.manual_op.source,
                 ev->payload.manual_op.action_id,
                 ev->payload.manual_op.accepted ? "ACEPTADO" : "RECHAZADO");
        break;
    case EV_APP_CALIBRATION:
        snprintf(buf, len,
                 "[%6u] [CAL]: max_count=%d travel=%.2f mm trigger=%u %s",
                 ev->timestamp_ms,
                 ev->payload.calibration.max_encoder_count,
                 ev->payload.calibration.travel_mm,
                 ev->payload.calibration.trigger,
                 ev->payload.calibration.success ? "OK" : "FAIL");
        break;
    case EV_SYS_CALIBRATION_SAVED:
        snprintf(buf, len, "[%6u] [CFG]: Calibration saved to Flash.", ev->timestamp_ms);
        break;

    // Alarms
    case EV_ALARM_OCCLUSION:
        snprintf(buf, len,
                 "[%6u] [ALARM:HIGH]: Oclusion sid=%u presion=%.1f mmHg fsm=%u",
                 ev->timestamp_ms,
                 (unsigned)ev->payload.alarm.session_id,
                 ev->payload.alarm.param_f,
                 ev->payload.alarm.fsm_state);
        break;
    case EV_ALARM_EOT:
        snprintf(buf, len,
                 "[%6u] [ALARM:MED]: Fin de carrera sid=%u",
                 ev->timestamp_ms, (unsigned)ev->payload.alarm.session_id);
        break;
    case EV_ALARM_DRV_FAULT:
        snprintf(buf, len,
                 "[%6u] [ALARM:HIGH]: Falla driver fsm=%u",
                 ev->timestamp_ms, ev->payload.alarm.fsm_state);
        break;
    case EV_ALARM_BATTERY_LOW:
        snprintf(buf, len,
                 "[%6u] [ALARM:LOW]: Bateria baja",
                 ev->timestamp_ms);
        break;
    case EV_ALARM_MAINS_LOST:
        snprintf(buf, len,
                 "[%6u] [ALARM:MED]: Red electrica perdida",
                 ev->timestamp_ms);
        break;

    // Power
    case EV_PWR_BATTERY_UPD:
        snprintf(buf, len,
                 "[%6u] [PWR]: bat=%.2fV %u%% flags=0x%02X",
                 ev->timestamp_ms,
                 ev->payload.power.battery_voltage_v,
                 ev->payload.power.battery_pct,
                 ev->payload.power.flags);
        break;
    case EV_PWR_MAINS_DETECT:
        snprintf(buf, len,
                 "[%6u] [PWR]: Red electrica %s",
                 ev->timestamp_ms,
                 ev->payload.param ? "presente" : "ausente");
        break;

    // System health
    case EV_SYS_HEAP_UPD:
        snprintf(buf, len,
                 "[%6u] [SYS]: heap=%u bytes min=%u drops tx=%u rx=%u",
                 ev->timestamp_ms,
                 (unsigned)ev->payload.sys_health.free_heap_bytes,
                 (unsigned)ev->payload.sys_health.min_ever_heap_bytes,
                 ev->payload.sys_health.mqtt_tx_drops,
                 ev->payload.sys_health.mqtt_rx_drops);
        break;
    case EV_SYS_HEARTBEAT:
        snprintf(buf, len,
                 "[%6u] [SYS]: Core1 heartbeat #%u",
                 ev->timestamp_ms, (unsigned)ev->payload.param);
        break;
    case EV_SYS_CLI_READY:
        snprintf(buf, len, "[%6u] [SYS]: Pico CLI Ready. Waiting for commands...", ev->timestamp_ms);
        break;

    default:
        snprintf(buf, len,
                 "[%6u] [???]: id=0x%04X param=0x%08X",
                 ev->timestamp_ms, ev->id, (unsigned)ev->payload.param);
        break;
    }
}

static void task_serial_consumer(void *arg) {
    (void)arg;
    SystemEvent_t ev;
    char buf[256];

    for (;;) {
        if (xQueueReceive(g_serial_q, &ev, portMAX_DELAY) == pdTRUE) {
            buf[0] = '\0';
            format_event(&ev, buf, sizeof(buf));
            printf("%s\n", buf);
        }
    }
}

void serial_consumer_start(void) {
    xTaskCreate(task_serial_consumer, "SerialCons", SERIAL_STACK_WORDS,
                NULL, SERIAL_PRIORITY, NULL);
}
