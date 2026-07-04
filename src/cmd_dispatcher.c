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
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_HOME);
    }
    if (strncmp(str, "fsm_search,", 11) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_SEARCH_SYRINGE, (float)atof(str + 11), 0.0f);
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_SEARCH_SYRINGE);
    }
    if (strncmp(str, "fsm_dispense,", 13) == 0) {
        float target = 0.0f, vel = 0.0f;
        if (sscanf(str + 13, "%f,%f", &target, &vel) != 2) {
            printf("[CMD]: Error: uso -> fsm_dispense,TARGET_UM,VELOCITY_UMS\n");
            return dispatch_result(false, "bad_format", src, (uint8_t)PUMP_ACTION_START_DISPENSE);
        }
        bool ok = cmd_gate_execute(PUMP_ACTION_START_DISPENSE, target, vel);
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_START_DISPENSE);
    }
    if (strncmp(str, "fsm_search_eot", 14) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_SEARCH_EOT, 0.0f, 0.0f);
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_SEARCH_EOT);
    }
    if (strncmp(str, "fsm_reset", 9) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_RESET, 0.0f, 0.0f);
        return dispatch_result(ok, ok ? "ok" : "queue_full", src, (uint8_t)PUMP_ACTION_RESET);
    }
    if (strncmp(str, "fsm_cont", 8) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_CONTINUE_DISPENSE, 0.0f, 0.0f);
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_CONTINUE_DISPENSE);
    }
    if (strncmp(str, "fsm_occ_rel", 11) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_OCC_RELEASE, 0.0f, 0.0f);
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_OCC_RELEASE);
    }
    if (strncmp(str, "fsm_resume", 10) == 0) {
        bool ok = cmd_gate_execute(PUMP_ACTION_RESUME, 0.0f, 0.0f);
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
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
        if (!ok) printf("[CMD]: Rechazado en estado: %s\n", get_state_name(cmd_gate_get_fsm_state()));
        return dispatch_result(ok, ok ? "ok" : "invalid_state", src, (uint8_t)PUMP_ACTION_CALIBRATE);
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
            printf("[CMD]: Error: nsteps,STEPS,FREQ_HZ\n");
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
            printf("[CFG]: Error: config_wifi,SSID,PASS\n");
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
            printf("[CFG]: Error: config_mqtt,IP,PORT\n");
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
    if (strncmp(str, "config_save", 11) == 0) {
        config_manager_save(false);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "config_info", 11) == 0) {
        printf("[CFG]: WiFi SSID: %s\n", g_sys_config.wifi_ssid);
        printf("[CFG]: WiFi PASS: %s\n", g_sys_config.wifi_pass);
        printf("[CFG]: MQTT IP  : %s\n", g_sys_config.mqtt_ip);
        printf("[CFG]: MQTT PORT: %u\n", g_sys_config.mqtt_port);
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }

    // ------------------------------------------------------------------
    // Network commands
    // ------------------------------------------------------------------
    if (strncmp(str, "reconnect", 9) == 0) {
        printf("[NET]: Force reconnecting WiFi and MQTT...\n");
        cyw43_arch_lwip_begin();
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_lwip_end();
        mqtt_client_force_reconnect();
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "net_disable", 11) == 0) {
        printf("[NET]: Disabling Wi-Fi/MQTT (Battery Save Mode)...\n");
        config_set_wifi_enabled(false);
        mqtt_client_force_reconnect();
        cyw43_arch_disable_sta_mode();
        return dispatch_result(true, "ok", src, CMD_ACTION_LOWLEVEL);
    }
    if (strncmp(str, "net_enable", 10) == 0) {
        printf("[NET]: Enabling Wi-Fi/MQTT...\n");
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

    printf("[CMD]: Comando desconocido: %s\n", str);
    return dispatch_result(false, "unknown_cmd", src, CMD_ACTION_LOWLEVEL);
}
