#ifndef CROSSCORE_LOGGER_H
#define CROSSCORE_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include "core1_main.h"

// Log Filter Configuration
typedef struct {
    bool show_tgt; // Target
    bool show_cfg; // Configuration
    bool show_kin; // Kinematics
    bool show_prf; // Profile
    bool show_fsm; // State Machine
    bool show_adc; // ADC Voltage
    bool show_prg; // Progress
    bool show_enc; // Encoder
    bool show_mtr; // Motor
} LogFilterConfig_t;

extern LogFilterConfig_t g_log_filter;
void log_filter_set(const char *hdr, bool state);

// Event IDs
typedef enum {
    LOG_EVENT_HEARTBEAT = 0,
    LOG_EVENT_PRESSURE_UPDATE,
    LOG_EVENT_PRESSURE_ALERT,
    LOG_EVENT_PRESSURE_SAFE,
    LOG_EVENT_MOTOR_MOVING,
    LOG_EVENT_MOTOR_STOPPED,
    LOG_EVENT_MOTOR_RETRACTING,
    LOG_EVENT_MOTOR_START_HIT,
    LOG_EVENT_MOTOR_END_HIT,
    LOG_EVENT_MOTOR_STALL,
    LOG_EVENT_DRV_STATUS_ERROR,
    LOG_EVENT_UART_INIT_OK,
    LOG_EVENT_UART_INIT_FAIL,
    LOG_EVENT_UART_INIT_MICROSTEPS_READ,
    LOG_EVENT_PINS_INIT_MODE,
    LOG_EVENT_GENERAL_DEBUG,
    LOG_EVENT_STRING_MSG,
    LOG_EVENT_ENCODER_UPDATE,
    LOG_EVENT_ENCODER_INDEP_UPDATE,
    LOG_EVENT_ENCODER_SPEED,
    LOG_EVENT_SPEED_WARNING,
    LOG_EVENT_CORRECTION_APPLIED,
    LOG_EVENT_MOTOR_PROGRESS,
    LOG_EVENT_FSM_STATE
} LogEventID_t;

// Payload Struct (Union to save space)
typedef struct {
    LogEventID_t id;
    union {
        uint32_t counter;          // For heartbeat
        float pressure_psi;        // For pressure readings
        struct {
            uint16_t stall;
            uint32_t drv_status;
            uint32_t gstat;
        } motor_status;            // For motor status polling
        struct {
            bool is_ccw;
            float target_freq;
        } move_info;               // For move logging
        struct {
            int step_count;
            int time_ms;
        } steps_info;              // For step logging
        int32_t encoder_count;     // For encoder polling
        struct {
            int32_t count_a;
            int32_t count_b;
        } encoder_indep_count;
        float encoder_speed;       // For encoder speed
        struct {
            float expected_ums;
            float actual_ums;
        } speed_warning;
        float correction_um;
        float progress_pct;        // For motor progress
        Core1State_t fsm_state;    // For FSM state transitions
        uint32_t raw_data;         // For arbitrary data
        const char *msg_str;       // For string messages (pointers)
    } payload;
} LogMessage_t;

// Helper functions for Core 1 (Non-blocking)
void logger_send_heartbeat(uint32_t count);
void logger_send_pressure_update(float psi);
void logger_send_pressure_alert(float psi);
void logger_send_pressure_safe(float psi);
void logger_send_motor_moving(void);
void logger_send_motor_stopped(void);
void logger_send_motor_retracting(void);
void logger_send_motor_start_hit(void);
void logger_send_motor_end_hit(void);
void logger_send_motor_stall(uint16_t stall);
void logger_send_drv_status_error(uint16_t stall, uint32_t drv_status, uint32_t gstat);
void logger_send_uart_init_ok(uint32_t ioin);
void logger_send_uart_init_fail(void);
void logger_send_uart_init_microsteps_read(uint16_t msteps);
void logger_send_pins_init_mode(void);
void logger_send_string(const char *str);
void logger_send_encoder_count(int32_t count);
void logger_send_encoder_indep_counts(int32_t count_a, int32_t count_b);
void logger_send_encoder_speed(float ums);
void logger_send_speed_warning(float expected_ums, float actual_ums);
void logger_send_correction_applied(float correction_um);
void logger_send_motor_progress(float pct);
void logger_send_fsm_state(Core1State_t state);

#endif // CROSSCORE_LOGGER_H
