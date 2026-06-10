#include "crosscore_cmd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config_manager.h"
#include "pico/cyw43_arch.h"
#include "mqtt_client.h"
#include "crosscore_logger.h"

// The actual queue instance
queue_t crosscore_cmd_queue;

// Initialize the queue. Must be called before any core tries to use it.
void crosscore_cmd_init(void) {
    // Initialize the queue to hold 10 Core1CmdMessage_t structures.
    queue_init(&crosscore_cmd_queue, sizeof(Core1CmdMessage_t), 10);
}

// Send a move linear command
bool cmd_send_move_linear_um(float target_um, float target_velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_MOVE_LINEAR_UM;
    msg.payload.move_linear.target_um = target_um;
    msg.payload.move_linear.target_velocity_ums = target_velocity_ums;
    
    // Add to queue (non-blocking). Returns true if added, false if queue is full.
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

// Send a stop motor command
bool cmd_send_stop_motor(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_STOP_MOTOR;
    msg.payload.raw_data = 0; // Unused
    
    // Add to queue (non-blocking). Returns true if added, false if queue is full.
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_move_2part_profile(float start_freq, uint32_t a_p1, float f_mid_accel,
                                 uint32_t a_p2, float f_target, uint32_t c_steps,
                                 uint32_t d_p1, float f_mid_decel, uint32_t d_p2,
                                 float f_end) {
    Core1CmdMessage_t msg;
    msg.id = CMD_MOVE_2PART_PROFILE;
    msg.payload.move_2part.start_freq = start_freq;
    msg.payload.move_2part.a_p1 = a_p1;
    msg.payload.move_2part.f_mid_accel = f_mid_accel;
    msg.payload.move_2part.a_p2 = a_p2;
    msg.payload.move_2part.f_target = f_target;
    msg.payload.move_2part.c_steps = c_steps;
    msg.payload.move_2part.d_p1 = d_p1;
    msg.payload.move_2part.f_mid_decel = f_mid_decel;
    msg.payload.move_2part.d_p2 = d_p2;
    msg.payload.move_2part.f_end = f_end;
    
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_home_start(float target_velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_HOME_START;
    msg.payload.move_home.target_velocity_ums = target_velocity_ums;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_home_end(float target_velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_HOME_END;
    msg.payload.move_home.target_velocity_ums = target_velocity_ums;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_move_nsteps(int32_t nsteps, float freq_hz) {
    Core1CmdMessage_t msg;
    msg.id = CMD_MOVE_NSTEPS;
    msg.payload.move_nsteps.nsteps = nsteps;
    msg.payload.move_nsteps.freq_hz = freq_hz;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_stop_immediate(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_STOP_IMMEDIATE;
    msg.payload.raw_data = 0; // Unused
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

// --- FSM Commands ---

bool cmd_send_home(float velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_HOME;
    msg.payload.move_home.target_velocity_ums = velocity_ums;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_search_syringe(float velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_SEARCH_SYRINGE;
    msg.payload.move_home.target_velocity_ums = velocity_ums;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_start_dispense(float target_um, float target_velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_START_DISPENSE;
    msg.payload.start_dispense.target_um = target_um;
    msg.payload.start_dispense.target_velocity_ums = target_velocity_ums;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_search_eot(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_SEARCH_EOT;
    msg.payload.raw_data = 0;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_reset(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_RESET;
    msg.payload.raw_data = 0;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_continue_dispense(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_CONTINUE_DISPENSE;
    msg.payload.raw_data = 0;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_occ_release(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_OCC_RELEASE;
    msg.payload.raw_data = 0;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_resume_dispense(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_RESUME_DISPENSE;
    msg.payload.raw_data = 0;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_calibrate(void) {
    Core1CmdMessage_t msg;
    msg.id = CMD_CALIBRATE;
    msg.payload.raw_data = 0;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

bool cmd_send_set_virtual_lsw(bool enabled, int32_t start_count, int32_t end_count) {
    Core1CmdMessage_t msg;
    msg.id = CMD_SET_VIRTUAL_LSW;
    msg.payload.set_virtual_lsw.enabled = enabled;
    msg.payload.set_virtual_lsw.start_count = start_count;
    msg.payload.set_virtual_lsw.end_count = end_count;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}

void cmd_parse_and_execute(const char *payload_str) {
    // We expect a string like "10000.0,450.0" or "stop_imm" etc.
    char cmd_str[128];
    strncpy(cmd_str, payload_str, sizeof(cmd_str) - 1);
    cmd_str[sizeof(cmd_str) - 1] = '\0';

    // Check for "STOP_IMM" command
    if (strncmp(cmd_str, "stop_imm", 8) == 0 || strncmp(cmd_str, "STOP_IMM", 8) == 0) {
        printf("Executing STOP IMM command\n");
        cmd_send_stop_immediate();
        return;
    }

    // Check for "STOP" or "stop" command
    if (strncmp(cmd_str, "stop", 4) == 0 || strncmp(cmd_str, "STOP", 4) == 0) {
        printf("Executing STOP command\n");
        cmd_send_stop_motor();
        return;
    }

    // --- FSM Commands ---
    if (strncmp(cmd_str, "fsm_home,", 9) == 0) {
        float speed = (float)atof(cmd_str + 9);
        printf("FSM: Sending CMD_HOME (%.1f um/s)\n", speed);
        cmd_send_home(speed);
        return;
    }
    if (strncmp(cmd_str, "fsm_search,", 11) == 0) {
        float speed = (float)atof(cmd_str + 11);
        printf("FSM: Sending CMD_SEARCH_SYRINGE (%.1f um/s)\n", speed);
        cmd_send_search_syringe(speed);
        return;
    }
    if (strncmp(cmd_str, "fsm_dispense,", 13) == 0) {
        char *comma = strchr(cmd_str + 13, ',');
        if (comma != NULL) {
            *comma = '\0';
            float target = (float)atof(cmd_str + 13);
            float vel = (float)atof(comma + 1);
            printf("FSM: Sending CMD_START_DISPENSE (%.1f um @ %.1f um/s)\n", target, vel);
            cmd_send_start_dispense(target, vel);
        } else {
            printf("Error formating fsm_dispense. Use: fsm_dispense,TARGET,VELOCITY\n");
        }
        return;
    }
    if (strncmp(cmd_str, "fsm_search_eot", 14) == 0) {
        printf("FSM: Sending CMD_SEARCH_EOT\n");
        cmd_send_search_eot();
        return;
    }
    if (strncmp(cmd_str, "fsm_reset", 9) == 0) {
        printf("FSM: Sending CMD_RESET\n");
        cmd_send_reset();
        return;
    }
    if (strncmp(cmd_str, "fsm_cont", 8) == 0) {
        printf("FSM: Sending CMD_CONTINUE_DISPENSE\n");
        cmd_send_continue_dispense();
        return;
    }
    if (strncmp(cmd_str, "fsm_occ_rel", 11) == 0) {
        printf("FSM: Sending CMD_OCC_RELEASE\n");
        cmd_send_occ_release();
        return;
    }
    if (strncmp(cmd_str, "fsm_resume", 10) == 0) {
        printf("FSM: Sending CMD_RESUME_DISPENSE\n");
        cmd_send_resume_dispense();
        return;
    }
    if (strncmp(cmd_str, "fsm_calibrate", 13) == 0) {
        printf("FSM: Sending CMD_CALIBRATE\n");
        cmd_send_calibrate();
        return;
    }
    if (strncmp(cmd_str, "fsm_virtual_lsw,", 16) == 0) {
        int enabled, start_c, end_c;
        if (sscanf(cmd_str + 16, "%d,%d,%d", &enabled, &start_c, &end_c) == 3) {
            printf("FSM: Sending CMD_SET_VIRTUAL_LSW (ena:%d, %d -> %d)\n", enabled, start_c, end_c);
            cmd_send_set_virtual_lsw(enabled > 0, start_c, end_c);
        } else {
            printf("Error formating fsm_virtual_lsw. Use: fsm_virtual_lsw,ENABLED,START,END\n");
        }
        return;
    }
    // --------------------

    // Check for Home Start
    if (strncmp(cmd_str, "home_start,", 11) == 0) {
        float speed = (float)atof(cmd_str + 11);
        printf("Executing HOME START command at %.2f um/s\n", speed);
        cmd_send_home_start(speed);
        return;
    }

    // Check for Home End
    if (strncmp(cmd_str, "home_end,", 9) == 0) {
        float speed = (float)atof(cmd_str + 9);
        printf("Executing HOME END command at %.2f um/s\n", speed);
        cmd_send_home_end(speed);
        return;
    }

    // Check for N Steps (nsteps,3200,500.0)
    if (strncmp(cmd_str, "nsteps,", 7) == 0) {
        char *comma = strchr(cmd_str + 7, ',');
        if (comma != NULL) {
            *comma = '\0';
            int32_t nsteps = (int32_t)atoi(cmd_str + 7);
            float freq_hz = (float)atof(comma + 1);
            printf("Executing NSTEPS command: steps=%d, freq=%.2f Hz\n", nsteps, freq_hz);
            cmd_send_move_nsteps(nsteps, freq_hz);
        } else {
            printf("Failed to parse NSTEPS command format\n");
        }
        return;
    }

    // Check for log_en,HDR
    if (strncmp(cmd_str, "log_en,", 7) == 0) {
        log_filter_set(cmd_str + 7, true);
        return;
    }

    // Check for log_dis,HDR
    if (strncmp(cmd_str, "log_dis,", 8) == 0) {
        log_filter_set(cmd_str + 8, false);
        return;
    }

    // Check for config_wifi,SSID,PASS
    if (strncmp(cmd_str, "config_wifi,", 12) == 0) {
        char *comma = strchr(cmd_str + 12, ',');
        if (comma != NULL) {
            *comma = '\0';
            char *ssid = cmd_str + 12;
            char *pass = comma + 1;
            config_set_wifi(ssid, pass);
        } else {
            printf("Error formating config_wifi. Use: config_wifi,SSID,PASS\n");
        }
        return;
    }

    // Check for config_mqtt,IP,PORT
    if (strncmp(cmd_str, "config_mqtt,", 12) == 0) {
        char *comma = strchr(cmd_str + 12, ',');
        if (comma != NULL) {
            *comma = '\0';
            char *ip = cmd_str + 12;
            uint16_t port = (uint16_t)atoi(comma + 1);
            config_set_mqtt(ip, port);
        } else {
            printf("Error formating config_mqtt. Use: config_mqtt,IP,PORT\n");
        }
        return;
    }

    // Granular setters
    if (strncmp(cmd_str, "set_ssid,", 9) == 0) {
        config_set_wifi_ssid(cmd_str + 9);
        return;
    }
    if (strncmp(cmd_str, "set_wpass,", 10) == 0) {
        config_set_wifi_pass(cmd_str + 10);
        return;
    }
    if (strncmp(cmd_str, "set_mqtt_ip,", 12) == 0) {
        config_set_mqtt_ip(cmd_str + 12);
        return;
    }
    if (strncmp(cmd_str, "set_mqtt_port,", 14) == 0) {
        config_set_mqtt_port((uint16_t)atoi(cmd_str + 14));
        return;
    }

    // Check for config_save
    if (strncmp(cmd_str, "config_save", 11) == 0) {
        // Guardado no permite override manual (siempre verifica si el motor se mueve)
        config_manager_save(false);
        return;
    }

    // Check for config_info
    if (strncmp(cmd_str, "config_info", 11) == 0) {
        printf("\n--- CURRENT SYSTEM CONFIG ---\n");
        printf("WiFi SSID: %s\n", g_sys_config.wifi_ssid);
        printf("WiFi PASS: %s\n", g_sys_config.wifi_pass);
        printf("MQTT IP  : %s\n", g_sys_config.mqtt_ip);
        printf("MQTT PORT: %u\n", g_sys_config.mqtt_port);
        printf("-----------------------------\n");
        return;
    }

    // Check for reconnect
    if (strncmp(cmd_str, "reconnect", 9) == 0) {
        printf("Force reconnecting WiFi and MQTT...\n");
        // Force wifi disconnect
        cyw43_arch_lwip_begin();
        cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_lwip_end();
        
        // Force mqtt disconnect
        mqtt_client_force_reconnect();
        return;
    }

    // Check for net_disable
    if (strncmp(cmd_str, "net_disable", 11) == 0) {
        printf("Disabling Wi-Fi/MQTT (Battery Save Mode)...\n");
        config_set_wifi_enabled(false);
        mqtt_client_force_reconnect();
        cyw43_arch_disable_sta_mode();
        return;
    }

    // Check for net_enable
    if (strncmp(cmd_str, "net_enable", 10) == 0) {
        printf("Enabling Wi-Fi/MQTT...\n");
        config_set_wifi_enabled(true);
        cyw43_arch_enable_sta_mode();
        return;
    }

    // Fallback: assume format "target_um,velocity_ums"
    float target_um = 0.0f, velocity_ums = 0.0f;
    char *comma = strchr(cmd_str, ',');
    if (comma != NULL) {
        *comma = '\0';
        target_um = (float)atof(cmd_str);
        velocity_ums = (float)atof(comma + 1);
        printf("Executing linear move: target=%.2f um, vel=%.2f um/s\n", target_um, velocity_ums);
        cmd_send_move_linear_um(target_um, velocity_ums);
    } else {
        printf("Failed to parse command payload: %s\n", payload_str);
    }
}
