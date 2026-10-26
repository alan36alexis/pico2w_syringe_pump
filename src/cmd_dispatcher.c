#include "cmd_dispatcher.h"
#include "cmd_gate.h"
#include "crosscore_cmd.h"
#include "crosscore_logger.h"
#include "config_manager.h"
#include "mqtt_client.h"
#include "core1_main.h"
#include "system_queues.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Sentinel for action_id when a low-level / system command bypasses the FSM enum.
#define CMD_ACTION_LOWLEVEL 0xFF

static inline void print_fsm_rejected(void) {
    printf("CMD : ERR=rejected fsm=%s\n", get_state_name(cmd_gate_get_fsm_state()));
}

static CmdDispatchResult_t dispatch_result(bool ok, const char *reason,
                                            CmdSource_t src, uint8_t action_id) {
    DtoManualOp_t op = {
        .source      = (uint16_t)src,
        .accepted    = ok ? 1u : 0u,
        .reject_code = ok ? 0u : 1u,
        .fsm_from    = 0,
        .fsm_to      = 0,
        .action_id   = action_id,
    };
    CORE0_EMIT(EV_APP_CMD_EXECUTED, manual_op, op);
    return (CmdDispatchResult_t){ .accepted = ok, .reason = reason };
}

CmdDispatchResult_t cmd_dispatch_string(const char *str, CmdSource_t src, int32_t cid) {
    (void)cid;  // ACK correlation is the caller's responsibility

    // ------------------------------------------------------------------
    // STOP — unconditional early exit (safety invariant: never gated by FSM)
    // Check stop_imm before stop because stop_imm starts with "stop".
    // ------------------------------------------------------------------
    if (strncmp(str, "stop_imm", 8) == 0 || strncmp(str, "STOP_IMM", 8) == 0 ||
        strncmp(str, "stop",     4) == 0 || strncmp(str, "STOP",     4) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_STOP, 0.0f, 0.0f);
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, (uint8_t)PUMP_ACTION_STOP);
    }

    // ------------------------------------------------------------------
    // FSM commands — validated against current Core 1 state via cmd_gate
    // ------------------------------------------------------------------
    if (strncmp(str, "fsm_home,", 9) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_HOME, (float)atof(str + 9), 0.0f);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_HOME);
    }
    if (strncmp(str, "fsm_search,", 11) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_SEARCH_SYRINGE, (float)atof(str + 11), 0.0f);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_SEARCH_SYRINGE);
    }
    if (strncmp(str, "fsm_dispense,", 13) == 0) {
        float target = 0.0f, vel = 0.0f;
        if (sscanf(str + 13, "%f,%f", &target, &vel) != 2) {
            printf("CMD : ERR=bad_format use=fsm_dispense,TARGET_UM,VELOCITY_UMS\n");
            return dispatch_result(false, "bad_format", src, (uint8_t)PUMP_ACTION_START_DISPENSE);
        }
        bool ok = cmd_gate_execute(PUMP_ACTION_START_DISPENSE, target, vel);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_START_DISPENSE);
    }
    if (strncmp(str, "fsm_search_eot", 14) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_SEARCH_EOT, 0.0f, 0.0f);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_SEARCH_EOT);
    }
    if (strncmp(str, "fsm_reset", 9) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_RESET, 0.0f, 0.0f);
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, (uint8_t)PUMP_ACTION_RESET);
    }
    if (strncmp(str, "fsm_cont", 8) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_CONTINUE_DISPENSE, 0.0f, 0.0f);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_CONTINUE_DISPENSE);
    }
    if (strncmp(str, "fsm_occ_rel", 11) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_OCC_RELEASE, 0.0f, 0.0f);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_OCC_RELEASE);
    }
    if (strncmp(str, "fsm_resume", 10) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_RESUME, 0.0f, 0.0f);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_RESUME);
    }
    if (strncmp(str, "fsm_calibrate", 13) == 0) {
        // Velocidades opcionales: fsm_calibrate[,vel_move[,vel_seek]] (0 = default)
        float calib_vel_move = 0.0f, calib_vel_seek = 0.0f;
        if (str[13] == ',') {
            calib_vel_move = (float)atof(str + 14);
            const char *sep = strchr(str + 14, ',');
            if (sep) calib_vel_seek = (float)atof(sep + 1);
        }
        bool ok = cmd_gate_execute(PUMP_ACTION_CALIBRATE, calib_vel_move, calib_vel_seek);
        if (!ok) print_fsm_rejected();
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_CALIBRATE);
    }
    if (strncmp(str, "fsm_enc_reset", 13) == 0) {
        // Utilitario: pone en cero la cuenta del encoder. No pasa por el gate
        // ni cambia el estado de la FSM.
        bool ok = cmd_send_enc_reset();
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, CMD_ACTION_LOWLEVEL);
    }

    // ------------------------------------------------------------------
    // Low-level motor commands (bypass FSM state validation)
    // ------------------------------------------------------------------
    if (strncmp(str, "home_start,", 11) == 0) {
        bool ok = cmd_send_home_start((float)atof(str + 11));
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "home_end,", 9) == 0) {
        bool ok = cmd_send_home_end((float)atof(str + 9));
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "nsteps,", 7) == 0) {
        char tmp[64];
        strncpy(tmp, str + 7, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        char *comma = strchr(tmp, ',');
        if (!comma) {
            printf("CMD : ERR=bad_format use=nsteps,STEPS,FREQ_HZ\n");
            return dispatch_result(false, "bad_format", src, CMD_ACTION_LOWLEVEL);
        }
        *comma = '\0';
        bool ok = cmd_send_move_nsteps((int32_t)atoi(tmp), (float)atof(comma + 1));
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, CMD_ACTION_LOWLEVEL);
    }

    // ------------------------------------------------------------------
    // Log filter
    // ------------------------------------------------------------------
    if (strncmp(str, "log_en,", 7) == 0) {
        log_filter_set(str + 7, true);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "log_dis,", 8) == 0) {
        log_filter_set(str + 8, false);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }

    // ------------------------------------------------------------------
    // Configuration commands
    // ------------------------------------------------------------------
    if (strncmp(str, "config_wifi,", 12) == 0) {
        char tmp[128];
        strncpy(tmp, str + 12, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        char *comma = strchr(tmp, ',');
        if (!comma) {
            printf("CFG : ERR=bad_format use=config_wifi,SSID,PASS\n");
            return dispatch_result(false, "bad_format", src, CMD_ACTION_LOWLEVEL);
        }
        *comma = '\0';
        config_set_wifi(tmp, comma + 1);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "config_mqtt,", 12) == 0) {
        char tmp[64];
        strncpy(tmp, str + 12, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        char *comma = strchr(tmp, ',');
        if (!comma) {
            printf("CFG : ERR=bad_format use=config_mqtt,IP,PORT\n");
            return dispatch_result(false, "bad_format", src, CMD_ACTION_LOWLEVEL);
        }
        *comma = '\0';
        config_set_mqtt(tmp, (uint16_t)atoi(comma + 1));
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_ssid,", 9) == 0) {
        config_set_wifi_ssid(str + 9);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_wpass,", 10) == 0) {
        config_set_wifi_pass(str + 10);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_mqtt_ip,", 12) == 0) {
        config_set_mqtt_ip(str + 12);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_mqtt_port,", 14) == 0) {
        config_set_mqtt_port((uint16_t)atoi(str + 14));
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "config_thresholds,", 18) == 0) {
        char tmp[64];
        strncpy(tmp, str + 18, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        char *c1 = strchr(tmp, ',');
        if (!c1) {
            printf("CFG : ERR=bad_format use=config_thresholds,V_SYR,V_OCC,V_REL\n");
            return dispatch_result(false, "bad_format", src, CMD_ACTION_LOWLEVEL);
        }
        *c1 = '\0';
        char *c2 = strchr(c1 + 1, ',');
        if (!c2) {
            printf("CFG : ERR=bad_format use=config_thresholds,V_SYR,V_OCC,V_REL\n");
            return dispatch_result(false, "bad_format", src, CMD_ACTION_LOWLEVEL);
        }
        *c2 = '\0';
        float v_s = (float)atof(tmp);
        float v_o = (float)atof(c1 + 1);
        float v_r = (float)atof(c2 + 1);
        config_set_adc_thresholds(v_s, v_o, v_r);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_syringe_thresh,", 19) == 0) {
        float v = (float)atof(str + 19);
        config_set_adc_thresholds(v, 0.0f, 0.0f);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_occ_thresh,", 15) == 0) {
        float v = (float)atof(str + 15);
        config_set_adc_thresholds(0.0f, v, 0.0f);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_occ_rel_thresh,", 19) == 0) {
        float v = (float)atof(str + 19);
        config_set_adc_thresholds(0.0f, 0.0f, v);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "set_antibolus_auto,", 19) == 0) {
        bool auto_en = (atoi(str + 19) != 0);
        config_set_antibolus_mode(auto_en);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "config_antibolus,", 17) == 0) {
        const char *val = str + 17;
        bool auto_en = true;
        if (strcmp(val, "manual") == 0 || strcmp(val, "0") == 0 || strcmp(val, "false") == 0) {
            auto_en = false;
        } else if (strcmp(val, "auto") == 0 || strcmp(val, "1") == 0 || strcmp(val, "true") == 0) {
            auto_en = true;
        } else {
            printf("CFG : ERR=bad_format use=config_antibolus,<auto|manual|1|0>\n");
            return dispatch_result(false, "bad_format", src, CMD_ACTION_LOWLEVEL);
        }
        config_set_antibolus_mode(auto_en);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "config_save", 11) == 0) {
        config_manager_save(false);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "config_info", 11) == 0) {
        printf("CFG : ssid=\"%s\" ip=%s port=%u syr=%.2fV occ=%.2fV rel=%.2fV antibolus=%s\n",
               g_sys_config.wifi_ssid,
               g_sys_config.mqtt_ip,
               g_sys_config.mqtt_port,
               g_sys_config.thresh_syringe_contact_v,
               g_sys_config.thresh_occlusion_v,
               g_sys_config.thresh_occ_release_v,
               g_sys_config.auto_antibolus_enabled ? "AUTO" : "MANUAL");
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }

    // ------------------------------------------------------------------
    // Network commands
    // ------------------------------------------------------------------
    if (strncmp(str, "reconnect", 9) == 0) {
        printf("NET : reconnecting wifi=1 mqtt=1\n");
        cyw43_arch_lwip_begin();
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_lwip_end();
        mqtt_client_force_reconnect();
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "net_disable", 11) == 0) {
        printf("NET : wifi_disabled=1\n");
        config_set_wifi_enabled(false);
        mqtt_client_force_reconnect();
        cyw43_arch_disable_sta_mode();
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "net_enable", 10) == 0) {
        printf("NET : wifi_enabled=1\n");
        config_set_wifi_enabled(true);
        cyw43_arch_enable_sta_mode();
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }

    // ------------------------------------------------------------------
    // Legacy fallback: "target_um,velocity_ums"
    // ------------------------------------------------------------------
    {
        char tmp[64];
        strncpy(tmp, str, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        char *comma = strchr(tmp, ',');
        if (comma) {
            *comma = '\0';
            float target = (float)atof(tmp);
            float vel    = (float)atof(comma + 1);
            bool ok = cmd_send_move_linear_um(target, vel);
            return dispatch_result(ok, ok ? "ok" : "queue_full", src, CMD_ACTION_LOWLEVEL);
        }
    }

    printf("CMD : UNKNOWN cmd=\"%s\"\n", str);
    return dispatch_result(false, "unknown_cmd", src, CMD_ACTION_LOWLEVEL);
}
