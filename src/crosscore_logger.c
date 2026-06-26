#include "crosscore_logger.h"
#include "system_events.h"
#include <string.h>

#define PSI_TO_MMHG 51.7149f

LogFilterConfig_t g_log_filter = {
    .show_tgt = true,
    .show_cfg = true,
    .show_kin = true,
    .show_prf = true,
    .show_fsm = true,
    .show_adc = true,
    .show_prg = true,
    .show_enc = true,
    .show_mtr = true
};

void log_filter_set(const char *hdr, bool state) {
    if (strcmp(hdr, "ALL") == 0) {
        g_log_filter.show_tgt = state;
        g_log_filter.show_cfg = state;
        g_log_filter.show_kin = state;
        g_log_filter.show_prf = state;
        g_log_filter.show_fsm = state;
        g_log_filter.show_adc = state;
        g_log_filter.show_prg = state;
        g_log_filter.show_enc = state;
        g_log_filter.show_mtr = state;
    } else if (strcmp(hdr, "TGT") == 0) g_log_filter.show_tgt = state;
    else if (strcmp(hdr, "CFG") == 0) g_log_filter.show_cfg = state;
    else if (strcmp(hdr, "KIN") == 0) g_log_filter.show_kin = state;
    else if (strcmp(hdr, "PRF") == 0) g_log_filter.show_prf = state;
    else if (strcmp(hdr, "FSM") == 0) g_log_filter.show_fsm = state;
    else if (strcmp(hdr, "ADC") == 0) g_log_filter.show_adc = state;
    else if (strcmp(hdr, "PRG") == 0) g_log_filter.show_prg = state;
    else if (strcmp(hdr, "ENC") == 0) g_log_filter.show_enc = state;
    else if (strcmp(hdr, "MTR") == 0) g_log_filter.show_mtr = state;
}

static Core1State_t s_prev_fsm_state = (Core1State_t)0;

void logger_send_heartbeat(uint32_t count) {
    CORE1_EMIT(EV_SYS_HEARTBEAT, param, count);
}

void logger_send_pressure_update(float psi) {
    DtoForce_t dto = { .psi = psi, .mmhg = psi * PSI_TO_MMHG };
    CORE1_EMIT(EV_ACT_PRESSURE, force, dto);
}

void logger_send_pressure_alert(float psi) {
    DtoForce_t dto = { .psi = psi, .mmhg = psi * PSI_TO_MMHG };
    CORE1_EMIT(EV_ACT_PRESSURE_OCC, force, dto);
}

void logger_send_pressure_safe(float psi) {
    DtoForce_t dto = { .psi = psi, .mmhg = psi * PSI_TO_MMHG };
    CORE1_EMIT(EV_ACT_PRESSURE, force, dto);
}

void logger_send_motor_moving(void)     { /* covered by EV_APP_FSM_STATE */ }
void logger_send_motor_stopped(void)    { /* covered by EV_APP_FSM_STATE */ }
void logger_send_motor_retracting(void) { /* covered by EV_APP_FSM_STATE */ }

void logger_send_motor_start_hit(void) {
    CORE1_EMIT(EV_ACT_LSW_END, param, (uint32_t)0);
}

void logger_send_motor_end_hit(void) {
    CORE1_EMIT(EV_ACT_LSW_END, param, (uint32_t)1);
}

void logger_send_motor_stall(uint16_t stall) {
    DtoTmcStatus_t dto = { .stall_count = stall };
    CORE1_EMIT(EV_TMC_STALL, tmc, dto);
}

void logger_send_drv_status_error(uint16_t stall, uint32_t drv_status, uint32_t gstat) {
    (void)stall; (void)drv_status; (void)gstat;
    // Replaced by direct CORE1_EMIT with parsed flags in core1_main.c
}

void logger_send_uart_init_ok(uint32_t ioin)          { (void)ioin;   /* startup diagnostic */ }
void logger_send_uart_init_microsteps_read(uint16_t m) { (void)m;      /* startup diagnostic */ }
void logger_send_pins_init_mode(void)                  {               /* startup diagnostic */ }

void logger_send_uart_init_fail(void) {
    DtoAlarm_t dto = { .severity = 3, .alarm_id = EV_ALARM_DRV_FAULT };
    CORE1_EMIT(EV_ALARM_DRV_FAULT, alarm, dto);
}

void logger_send_string(const char *str) {
    (void)str; /* strings not allowed in EDA queue — call site to be removed */
}

void logger_send_encoder_count(int32_t count) {
    DtoMotion_t dto = { .encoder_count = count };
    CORE1_EMIT(EV_MOT_ENCODER, motion, dto);
}

void logger_send_encoder_indep_counts(int32_t count_a, int32_t count_b) {
    (void)count_a; (void)count_b; /* no EDA event for independent channel counts */
}

void logger_send_encoder_speed(float ums) {
    DtoMotion_t dto = { .encoder_speed_ums = ums };
    CORE1_EMIT(EV_MOT_SPEED, motion, dto);
}

void logger_send_speed_warning(float expected_ums, float actual_ums) {
    DtoMotion_t dto = { .target_speed_ums = expected_ums, .encoder_speed_ums = actual_ums };
    CORE1_EMIT(EV_MOT_SPEED_WARN, motion, dto);
}

void logger_send_correction_applied(float correction_um) {
    DtoMotion_t dto = { .correction_um = correction_um };
    CORE1_EMIT(EV_MOT_CORRECTION, motion, dto);
}

void logger_send_motor_progress(float pct) {
    DtoMotion_t dto = { .progress_pct = pct };
    CORE1_EMIT(EV_MOT_PROGRESS, motion, dto);
}

void logger_send_fsm_state(Core1State_t state) {
    DtoFsm_t dto = {
        .state_from = (uint16_t)s_prev_fsm_state,
        .state_to   = (uint16_t)state,
        .session_id = 0
    };
    CORE1_EMIT(EV_APP_FSM_STATE, fsm, dto);
    s_prev_fsm_state = state;
}
