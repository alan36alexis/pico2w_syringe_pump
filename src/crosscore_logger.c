#include "crosscore_logger.h"

// The queue instance
queue_t crosscore_log_queue;

void crosscore_logger_init(void) {
    // Initialize the queue with a size of 32 messages
    queue_init(&crosscore_log_queue, sizeof(LogMessage_t), 32);
}

// -----------------------------------------------------------------------------
// Core 1 Helpers (Non-blocking additions to the spinlock queue)
// -----------------------------------------------------------------------------

static void _try_send(LogMessage_t *msg) {
    // try_add is non-blocking. If the queue is full, the message is dropped.
    // This ensures Core 1 never hangs waiting for Core 0 to print.
    queue_try_add(&crosscore_log_queue, msg);
}

void logger_send_heartbeat(uint32_t count) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_HEARTBEAT;
    msg.payload.counter = count;
    _try_send(&msg);
}

void logger_send_pressure_update(float psi) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_PRESSURE_UPDATE;
    msg.payload.pressure_psi = psi;
    _try_send(&msg);
}

void logger_send_pressure_alert(float psi) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_PRESSURE_ALERT;
    msg.payload.pressure_psi = psi;
    _try_send(&msg);
}

void logger_send_pressure_safe(float psi) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_PRESSURE_SAFE;
    msg.payload.pressure_psi = psi;
    _try_send(&msg);
}

void logger_send_motor_moving(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_MOVING;
    _try_send(&msg);
}

void logger_send_motor_stopped(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_STOPPED;
    _try_send(&msg);
}

void logger_send_motor_retracting(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_RETRACTING;
    _try_send(&msg);
}

void logger_send_motor_start_hit(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_START_HIT;
    _try_send(&msg);
}

void logger_send_motor_end_hit(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_END_HIT;
    _try_send(&msg);
}

void logger_send_motor_stall(uint16_t stall) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_STALL;
    msg.payload.motor_status.stall = stall;
    _try_send(&msg);
}

void logger_send_drv_status_error(uint16_t stall, uint32_t drv_status, uint32_t gstat) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_DRV_STATUS_ERROR;
    msg.payload.motor_status.stall = stall;
    msg.payload.motor_status.drv_status = drv_status;
    msg.payload.motor_status.gstat = gstat;
    _try_send(&msg);
}

void logger_send_uart_init_ok(uint32_t ioin) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_UART_INIT_OK;
    msg.payload.raw_data = ioin;
    _try_send(&msg);
}

void logger_send_uart_init_fail(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_UART_INIT_FAIL;
    _try_send(&msg);
}

void logger_send_uart_init_microsteps_read(uint16_t msteps) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_UART_INIT_MICROSTEPS_READ;
    msg.payload.raw_data = msteps;
    _try_send(&msg);
}

void logger_send_pins_init_mode(void) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_PINS_INIT_MODE;
    _try_send(&msg);
}

void logger_send_string(const char *str) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_STRING_MSG;
    msg.payload.msg_str = str;
    _try_send(&msg);
}

void logger_send_encoder_count(int32_t count) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_ENCODER_UPDATE;
    msg.payload.encoder_count = count;
    _try_send(&msg);
}

void logger_send_encoder_indep_counts(int32_t count_a, int32_t count_b) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_ENCODER_INDEP_UPDATE;
    msg.payload.encoder_indep_count.count_a = count_a;
    msg.payload.encoder_indep_count.count_b = count_b;
    _try_send(&msg);
}

void logger_send_encoder_speed(float pps) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_ENCODER_SPEED;
    msg.payload.encoder_speed = pps;
    _try_send(&msg);
}

void logger_send_speed_warning(float expected_ums, float actual_ums) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_SPEED_WARNING;
    msg.payload.speed_warning.expected_ums = expected_ums;
    msg.payload.speed_warning.actual_ums = actual_ums;
    _try_send(&msg);
}

void logger_send_correction_applied(float correction_um) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_CORRECTION_APPLIED;
    msg.payload.correction_um = correction_um;
    _try_send(&msg);
}

void logger_send_motor_progress(float pct) {
    LogMessage_t msg;
    msg.id = LOG_EVENT_MOTOR_PROGRESS;
    msg.payload.progress_pct = pct;
    _try_send(&msg);
}

