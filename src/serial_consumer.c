#include "serial_consumer.h"
#include "system_queues.h"
#include "core1_main.h"
#include "system_config.h"
#include "crosscore_logger.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SERIAL_PRIORITY    1
#define SERIAL_STACK_WORDS (configMINIMAL_STACK_SIZE * 4)

#if defined(ENABLE_SERIAL_ANSI_COLOR) && ENABLE_SERIAL_ANSI_COLOR
  #define ANSI_CLR_RESET   "\033[0m"
  #define ANSI_CLR_RED     "\033[1;31m"
  #define ANSI_CLR_GREEN   "\033[1;32m"
  #define ANSI_CLR_YELLOW  "\033[1;33m"
  #define ANSI_CLR_CYAN    "\033[1;36m"
  #define ANSI_CLR_GRAY    "\033[0;90m"
#else
  #define ANSI_CLR_RESET   ""
  #define ANSI_CLR_RED     ""
  #define ANSI_CLR_GREEN   ""
  #define ANSI_CLR_YELLOW  ""
  #define ANSI_CLR_CYAN    ""
  #define ANSI_CLR_GRAY    ""
#endif

static const char* tag_color(const char *tag) {
#if defined(ENABLE_SERIAL_ANSI_COLOR) && ENABLE_SERIAL_ANSI_COLOR
    if (tag[0] == '!') return ANSI_CLR_RED;
    if (strncmp(tag, "FSM", 3) == 0) return ANSI_CLR_CYAN;
    if (strncmp(tag, "SYS", 3) == 0) return ANSI_CLR_GREEN;
    if (strncmp(tag, "CFG", 3) == 0) return ANSI_CLR_GREEN;
    if (strncmp(tag, "APP", 3) == 0) return ANSI_CLR_GREEN;
    if (strncmp(tag, "CMD", 3) == 0) return ANSI_CLR_GREEN;
    if (strncmp(tag, "MOT", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "MTR", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "ACT", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "ADC", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "ENC", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "PRG", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "TGT", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "KIN", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "PRF", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "TMC", 3) == 0) return ANSI_CLR_YELLOW;
    if (strncmp(tag, "HWS", 3) == 0) return ANSI_CLR_CYAN;
    if (strncmp(tag, "NET", 3) == 0) return ANSI_CLR_CYAN;
    if (strncmp(tag, "DBG", 3) == 0) return ANSI_CLR_GRAY;
#endif
    return "";
}

static void fmt_msg(char *buf, size_t len, uint32_t ts_ms, const char *tag, const char *fmt, ...) {
    uint32_t s = ts_ms / 1000;
    uint32_t ds = (ts_ms % 1000) / 100;
    const char *clr = tag_color(tag);
    const char *rst = (clr[0] != '\0') ? ANSI_CLR_RESET : "";
    const char *ts_clr = ANSI_CLR_GRAY;
    const char *ts_rst = (ts_clr[0] != '\0') ? ANSI_CLR_RESET : "";

    // Formato Tag-Value / Key-Value: @<sec>.<dec> <TAG>: <key>=<val> ...
    int written = snprintf(buf, len, "%s@%4u.%1u%s %s%-4s:%s ",
                           ts_clr, (unsigned)s, (unsigned)ds, ts_rst,
                           clr, tag, rst);
    if (written > 0 && (size_t)written < len) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf + written, len - written, fmt, args);
        va_end(args);
    }

    size_t slen = strlen(buf);
    while (slen > 0 && (buf[slen - 1] == '\n' || buf[slen - 1] == '\r')) {
        buf[--slen] = '\0';
    }
}

void serial_log(const char *tag, const char *fmt, ...) {
    if (!log_filter_is_enabled(tag)) return;

    uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
    uint32_t s = ts_ms / 1000;
    uint32_t ds = (ts_ms % 1000) / 100;
    const char *clr = tag_color(tag);
    const char *rst = (clr[0] != '\0') ? ANSI_CLR_RESET : "";
    const char *ts_clr = ANSI_CLR_GRAY;
    const char *ts_rst = (ts_clr[0] != '\0') ? ANSI_CLR_RESET : "";

    char buf[256];
    int written = snprintf(buf, sizeof(buf), "%s@%4u.%1u%s %s%-4s:%s ",
                           ts_clr, (unsigned)s, (unsigned)ds, ts_rst,
                           clr, tag, rst);
    if (written > 0 && (size_t)written < sizeof(buf)) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf + written, sizeof(buf) - written, fmt, args);
        va_end(args);
    }

    size_t slen = strlen(buf);
    while (slen > 0 && (buf[slen - 1] == '\n' || buf[slen - 1] == '\r')) {
        buf[--slen] = '\0';
    }
    printf("%s\n", buf);
}

// The ONLY place in the firmware where SystemEventID_t values are mapped to
// human-readable strings. All other modules emit typed events — no strings.
static void format_event(const SystemEvent_t *ev, char *buf, size_t len) {
    switch (ev->id) {

    // TMC2209 Driver
    case EV_TMC_DRV_STATUS: {
        uint8_t f = ev->payload.tmc.flags;
        if (f == 0) {
            if (!log_filter_is_enabled("TMC")) return;
            fmt_msg(buf, len, ev->timestamp_ms, "TMC", "ok=1");
        } else {
            fmt_msg(buf, len, ev->timestamp_ms, "!TMC", "err=0x%02X%s%s%s%s%s%s%s%s",
                     f,
                     (f & TMC_FLAG_OT_WARN) ? " OT_WARN"  : "",
                     (f & TMC_FLAG_OT_SHUT) ? " OT_SHUT"  : "",
                     (f & TMC_FLAG_SHORT_A) ? " SHORT_A"  : "",
                     (f & TMC_FLAG_SHORT_B) ? " SHORT_B"  : "",
                     (f & TMC_FLAG_OPEN_A)  ? " OPEN_A"   : "",
                     (f & TMC_FLAG_OPEN_B)  ? " OPEN_B"   : "",
                     (f & TMC_FLAG_UV_CP)   ? " UV_CP"    : "",
                     (f & TMC_FLAG_DRV_ERR) ? " DRV_ERR"  : "");
        }
        break;
    }
    case EV_TMC_STALL:
        fmt_msg(buf, len, ev->timestamp_ms, "!TMC", "stall=%u",
                ev->payload.tmc.stall_count);
        break;
    case EV_TMC_OVERTEMP:
        fmt_msg(buf, len, ev->timestamp_ms, "!TMC", "ot flags=0x%02X",
                ev->payload.tmc.flags);
        break;

    // Throttling state for high-frequency telemetry
    static uint32_t s_last_pressure_ms = 0;
    static uint32_t s_last_encoder_ms  = 0;
    static uint32_t s_last_speed_ms    = 0;
    static uint32_t s_last_progress_ms = 0;

    // Actuators / Sensors
    case EV_ACT_PRESSURE:
        if (!log_filter_is_enabled("ADC")) return;
#if defined(SERIAL_THROTTLE_PRESSURE_MS) && (SERIAL_THROTTLE_PRESSURE_MS > 0)
        if ((ev->timestamp_ms - s_last_pressure_ms) < SERIAL_THROTTLE_PRESSURE_MS) {
            return;
        }
        s_last_pressure_ms = ev->timestamp_ms;
#endif
        fmt_msg(buf, len, ev->timestamp_ms, "ACT", "prs=%.1f (%.0f mmHg)",
                ev->payload.force.psi, ev->payload.force.mmhg);
        break;
    case EV_ACT_PRESSURE_OCC:
        fmt_msg(buf, len, ev->timestamp_ms, "!OCC", "prs=%.1f (%.0f mmHg)",
                ev->payload.force.psi, ev->payload.force.mmhg);
        break;
    case EV_ACT_LSW_END:
        fmt_msg(buf, len, ev->timestamp_ms, "ACT", "lsw=END hit=1");
        break;
    case EV_ACT_SYRINGE_DET:
        fmt_msg(buf, len, ev->timestamp_ms, "ACT", "syr=%u",
                ev->payload.param ? 1 : 0);
        break;
    case EV_ACT_IO_EXP_INPUT:
        if (!log_filter_is_enabled("SYS")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "ACT", "exp_in=0x%04X",
                (unsigned int)ev->payload.param);
        break;
    case EV_HW_CORE1_STATUS:
        if (log_filter_is_enabled("HWS")) {
            fmt_msg(buf, len, ev->timestamp_ms, "HWS",
                    "frc=%.3fV(%u) bub=%.3fV(%u) lsw=[%u,%u] mot=[ena=%u,dir=%u,sg=%u]",
                    ev->payload.core1_hw.frc_sens_v, ev->payload.core1_hw.frc_sens_raw,
                    ev->payload.core1_hw.bub_sens_v, ev->payload.core1_hw.bub_sens_raw,
                    ev->payload.core1_hw.lsw_start, ev->payload.core1_hw.lsw_end,
                    ev->payload.core1_hw.motor_ena, ev->payload.core1_hw.motor_dir,
                    ev->payload.core1_hw.motor_stallguard);
        }
        break;

    // Motion
    case EV_MOT_ENCODER:
        if (!log_filter_is_enabled("ENC")) return;
#if defined(SERIAL_THROTTLE_ENCODER_MS) && (SERIAL_THROTTLE_ENCODER_MS > 0)
        if ((ev->timestamp_ms - s_last_encoder_ms) < SERIAL_THROTTLE_ENCODER_MS) {
            return;
        }
        s_last_encoder_ms = ev->timestamp_ms;
#endif
        fmt_msg(buf, len, ev->timestamp_ms, "ENC", "c=%d p=%.2f/%.2f",
                ev->payload.motion.encoder_count,
                ev->payload.motion.position_mm,
                calibration_max_encoder_count *
                    calc_um_per_pulse(USE_QUADRATURE_ENCODER) / 1000.0f);
        break;
    case EV_MOT_SPEED:
        if (!log_filter_is_enabled("MOT")) return;
#if defined(SERIAL_THROTTLE_SPEED_MS) && (SERIAL_THROTTLE_SPEED_MS > 0)
        if ((ev->timestamp_ms - s_last_speed_ms) < SERIAL_THROTTLE_SPEED_MS) {
            return;
        }
        s_last_speed_ms = ev->timestamp_ms;
#endif
        fmt_msg(buf, len, ev->timestamp_ms, "MOT", "v=%.1f tgt=%.1f",
                ev->payload.motion.encoder_speed_ums,
                ev->payload.motion.target_speed_ums);
        break;
    case EV_MOT_PROGRESS: {
        if (!log_filter_is_enabled("PRG")) return;
#if defined(SERIAL_THROTTLE_PROGRESS_MS) && (SERIAL_THROTTLE_PROGRESS_MS > 0)
        if (ev->payload.motion.progress_pct < 100.0f &&
            (ev->timestamp_ms - s_last_progress_ms) < SERIAL_THROTTLE_PROGRESS_MS) {
            return;
        }
        s_last_progress_ms = ev->timestamp_ms;
#endif
#if defined(ENABLE_SERIAL_PROGRESS_BAR) && (ENABLE_SERIAL_PROGRESS_BAR)
        char bar[16];
        const int total_slots = 10;
        float pct = ev->payload.motion.progress_pct;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        int filled = (int)((pct / 100.0f) * total_slots);
        if (filled > total_slots) filled = total_slots;
        for (int i = 0; i < total_slots; i++) {
            if (i < filled) {
                bar[i] = '=';
            } else if (i == filled && filled < total_slots && pct > 0.0f) {
                bar[i] = '>';
            } else {
                bar[i] = ' ';
            }
        }
        bar[total_slots] = '\0';

        fmt_msg(buf, len, ev->timestamp_ms, "PRG",
                "[%s] %5.1f%% pos=%.2fmm",
                bar, pct, ev->payload.motion.position_mm);
#else
        fmt_msg(buf, len, ev->timestamp_ms, "PRG", "p=%.1f%% pos=%.2fmm",
                ev->payload.motion.progress_pct, ev->payload.motion.position_mm);
#endif
        break;
    }
    case EV_MOT_CORRECTION:
        if (!log_filter_is_enabled("MOT")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "MOT", "corr=%+.1f um",
                ev->payload.motion.correction_um);
        break;
    case EV_MOT_SPEED_WARN:
        fmt_msg(buf, len, ev->timestamp_ms, "!MOT", "WARN_VEL exp=%.1f act=%.1f",
                ev->payload.motion.target_speed_ums,
                ev->payload.motion.encoder_speed_ums);
        break;

    // Network
    case EV_NET_WIFI_CONNECTING:
        if (!log_filter_is_enabled("NET")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "NET", "wifi=CONNECTING");
        break;
    case EV_NET_WIFI_CONN:
        if (!log_filter_is_enabled("NET")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "NET", "wifi=CONN rssi=%d ch=%u",
                ev->payload.wifi.rssi_dbm, ev->payload.wifi.channel);
        break;
    case EV_NET_WIFI_DISC:
        fmt_msg(buf, len, ev->timestamp_ms, "!NET", "wifi=DISC reason=%u",
                (unsigned)ev->payload.param);
        break;
    case EV_NET_MQTT_CONN:
        if (!log_filter_is_enabled("NET")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "NET", "mqtt=CONN");
        break;
    case EV_NET_MQTT_DISC:
        fmt_msg(buf, len, ev->timestamp_ms, "!NET", "mqtt=DISC");
        break;
    case EV_NET_MQTT_TX_DROP:
        fmt_msg(buf, len, ev->timestamp_ms, "!NET", "mqtt_drop=%u",
                (unsigned)ev->payload.param);
        break;

    // FSM / Application
    case EV_APP_FSM_STATE:
        if (!log_filter_is_enabled("FSM")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "FSM", "%s->%s sid=%u",
                get_state_name((Core1State_t)ev->payload.fsm.state_from),
                get_state_name((Core1State_t)ev->payload.fsm.state_to),
                (unsigned)ev->payload.fsm.session_id);
        break;
    case EV_APP_SESSION_START:
        if (!log_filter_is_enabled("APP")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "APP", "START sid=%u v=%.1f r=%.2f d=%.1f",
                (unsigned)ev->payload.session.session_id,
                ev->payload.session.target_volume_ml,
                ev->payload.session.rate_ml_h,
                ev->payload.session.syringe_diam_mm);
        break;
    case EV_APP_SESSION_END:
        if (!log_filter_is_enabled("APP")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "APP", "END sid=%u v=%.2f t=%u",
                (unsigned)ev->payload.session.session_id,
                ev->payload.session.infused_volume_ml,
                (unsigned)ev->payload.session.elapsed_s);
        break;
    case EV_APP_SESSION_UPD:
        if (!log_filter_is_enabled("APP")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "APP", "UPD sid=%u v=%.2f/%.1f t=%u",
                (unsigned)ev->payload.session.session_id,
                ev->payload.session.infused_volume_ml,
                ev->payload.session.target_volume_ml,
                (unsigned)ev->payload.session.elapsed_s);
        break;
    case EV_APP_CMD_EXECUTED:
        if (!log_filter_is_enabled("CMD")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "CMD", "src=%u act=%u ok=%u",
                ev->payload.manual_op.source,
                ev->payload.manual_op.action_id,
                ev->payload.manual_op.accepted ? 1 : 0);
        break;
    case EV_APP_CALIBRATION:
        if (!log_filter_is_enabled("CFG")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "CFG", "calib max=%d pos=%.2f trig=%u ok=%u",
                ev->payload.calibration.max_encoder_count,
                ev->payload.calibration.travel_mm,
                ev->payload.calibration.trigger,
                ev->payload.calibration.success ? 1 : 0);
        break;
    case EV_SYS_CALIBRATION_SAVED:
        if (!log_filter_is_enabled("CFG")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "CFG", "saved=flash");
        break;

    // Alarms (Always displayed)
    case EV_ALARM_OCCLUSION:
        fmt_msg(buf, len, ev->timestamp_ms, "!ALM", "occ sid=%u p=%.1f st=%u",
                (unsigned)ev->payload.alarm.session_id,
                ev->payload.alarm.param_f,
                ev->payload.alarm.fsm_state);
        break;
    case EV_ALARM_BUBBLE:
        fmt_msg(buf, len, ev->timestamp_ms, "!ALM", "bub sid=%u v=%.3f st=%u",
                (unsigned)ev->payload.alarm.session_id,
                ev->payload.alarm.param_f,
                ev->payload.alarm.fsm_state);
        break;
    case EV_ALARM_EOT:
        fmt_msg(buf, len, ev->timestamp_ms, "!ALM", "eot sid=%u",
                (unsigned)ev->payload.alarm.session_id);
        break;
    case EV_ALARM_DRV_FAULT:
        fmt_msg(buf, len, ev->timestamp_ms, "!ALM", "drv_fault fsm=%u",
                ev->payload.alarm.fsm_state);
        break;
    case EV_ALARM_BATTERY_LOW:
        fmt_msg(buf, len, ev->timestamp_ms, "!ALM", "bat_low=1");
        break;
    case EV_ALARM_MAINS_LOST:
        fmt_msg(buf, len, ev->timestamp_ms, "!ALM", "mains_lost=1");
        break;

    // Power
    case EV_PWR_BATTERY_UPD:
        if (!log_filter_is_enabled("SYS")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "SYS", "bat=%.2fV p=%u%% fl=0x%02X",
                ev->payload.power.battery_voltage_v,
                ev->payload.power.battery_pct,
                ev->payload.power.flags);
        break;
    case EV_PWR_MAINS_DETECT:
        if (!log_filter_is_enabled("SYS")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "SYS", "mains=%u",
                ev->payload.param ? 1 : 0);
        break;

    // System health
    case EV_SYS_HEAP_UPD:
        if (!log_filter_is_enabled("SYS")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "SYS", "heap=%u min=%u drop_tx=%u",
                (unsigned)ev->payload.sys_health.free_heap_bytes,
                (unsigned)ev->payload.sys_health.min_ever_heap_bytes,
                ev->payload.sys_health.mqtt_tx_drops);
        break;
    case EV_SYS_HEARTBEAT:
        if (!log_filter_is_enabled("SYS")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "SYS", "hb=%u",
                (unsigned)ev->payload.param);
        break;
    case EV_SYS_CLI_READY:
        if (!log_filter_is_enabled("SYS")) return;
        fmt_msg(buf, len, ev->timestamp_ms, "SYS", "cli=READY");
        break;

    case EV_DBG_STRING: {
        const char *msg_str = ev->payload.dbg_str.buf;
        char tag_str[8] = "DBG";
        if (msg_str[0] == '[') {
            const char *end_bracket = strchr(msg_str, ']');
            if (end_bracket != NULL && (end_bracket - msg_str) < 7) {
                size_t tag_len = end_bracket - (msg_str + 1);
                memcpy(tag_str, msg_str + 1, tag_len);
                tag_str[tag_len] = '\0';
                msg_str = end_bracket + 1;
                if (*msg_str == ':') msg_str++;
                while (*msg_str == ' ') msg_str++;
            }
        }
        if (!log_filter_is_enabled(tag_str)) return;
        fmt_msg(buf, len, ev->timestamp_ms, tag_str, "%s", msg_str);
        break;
    }

    default:
        fmt_msg(buf, len, ev->timestamp_ms, "???", "id=0x%04X p=0x%08X",
                ev->id, (unsigned)ev->payload.param);
        break;
    }
}

static void task_serial_consumer(void *arg) {
    (void)arg;
    SystemEvent_t ev;
    char buf[256];
    static bool s_last_was_progress = false;

    for (;;) {
        if (xQueueReceive(g_serial_q, &ev, portMAX_DELAY) == pdTRUE) {
            buf[0] = '\0';
            format_event(&ev, buf, sizeof(buf));
            size_t slen = strlen(buf);
            while (slen > 0 && (buf[slen - 1] == '\n' || buf[slen - 1] == '\r')) {
                buf[--slen] = '\0';
            }
            if (slen > 0) {
#if defined(ENABLE_SERIAL_PROGRESS_BAR) && (ENABLE_SERIAL_PROGRESS_BAR)
                if (ev.id == EV_MOT_PROGRESS) {
                    if (ev.payload.motion.progress_pct < 100.0f) {
#if ENABLE_SERIAL_ANSI_COLOR
                        printf("%s\033[K\r", buf);
#else
                        printf("%-68s\r", buf);
#endif
                        fflush(stdout);
                        s_last_was_progress = true;
                    } else {
#if ENABLE_SERIAL_ANSI_COLOR
                        printf("%s\033[K\n", buf);
#else
                        printf("%-68s\n", buf);
#endif
                        s_last_was_progress = false;
                    }
                } else {
                    if (s_last_was_progress) {
                        printf("\n");
                        s_last_was_progress = false;
                    }
                    printf("%s\n", buf);
                }
#else
                printf("%s\n", buf);
#endif
            }
        }
    }
}

void serial_consumer_start(void) {
    xTaskCreate(task_serial_consumer, "SerialCons", SERIAL_STACK_WORDS,
                NULL, SERIAL_PRIORITY, NULL);
}
