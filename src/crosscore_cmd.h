#ifndef CROSSCORE_CMD_H
#define CROSSCORE_CMD_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/util/queue.h"

// Define the queue that will pass commands from Core 0 to Core 1
extern queue_t crosscore_cmd_queue;

// Command IDs
typedef enum {
    CMD_MOVE_LINEAR_UM = 0,
    CMD_STOP_MOTOR,
    CMD_MOVE_2PART_PROFILE,
    CMD_HOME_START,
    CMD_HOME_END,
    CMD_MOVE_NSTEPS,
    CMD_STOP_IMMEDIATE,
    CMD_HOME,
    CMD_SEARCH_SYRINGE,
    CMD_START_DISPENSE,
    CMD_SEARCH_EOT,
    CMD_RESET,
    CMD_CONTINUE_DISPENSE,
    CMD_OCC_RELEASE,
    CMD_RESUME_DISPENSE,
    CMD_CALIBRATE,
    // Add more commands here as needed
} Core1CmdID_t;

// Payload Struct (Union to save space)
typedef struct {
    Core1CmdID_t id;
    union {
        struct {
            float target_um;
            float target_velocity_ums;
        } move_linear;
        struct {
            float start_freq;
            uint32_t a_p1;
            float f_mid_accel;
            uint32_t a_p2;
            float f_target;
            uint32_t c_steps;
            uint32_t d_p1;
            float f_mid_decel;
            uint32_t d_p2;
            float f_end;
        } move_2part;
        struct {
            float target_velocity_ums;
        } move_home;
        struct {
            int32_t nsteps;
            float freq_hz;
        } move_nsteps;
        struct {
            float target_um;
            float target_velocity_ums;
        } start_dispense;
        uint32_t raw_data; // For arbitrary data
    } payload;
} Core1CmdMessage_t;

// Initialization function (called by Core 0 before starting scheduler)
void crosscore_cmd_init(void);

// Helper functions for Core 0 (Non-blocking)
bool cmd_send_move_linear_um(float target_um, float target_velocity_ums);
bool cmd_send_stop_motor(void);
bool cmd_send_move_2part_profile(float start_freq, uint32_t a_p1, float f_mid_accel,
                                 uint32_t a_p2, float f_target, uint32_t c_steps,
                                 uint32_t d_p1, float f_mid_decel, uint32_t d_p2,
                                 float f_end);
bool cmd_send_home_start(float target_velocity_ums);
bool cmd_send_home_end(float target_velocity_ums);
bool cmd_send_move_nsteps(int32_t nsteps, float freq_hz);
bool cmd_send_stop_immediate(void);

// FSM Commands
bool cmd_send_home(float velocity_ums);
bool cmd_send_search_syringe(float velocity_ums);
bool cmd_send_start_dispense(float target_um, float target_velocity_ums);
bool cmd_send_search_eot(void);
bool cmd_send_reset(void);
bool cmd_send_continue_dispense(void);
bool cmd_send_occ_release(void);
bool cmd_send_resume_dispense(void);
bool cmd_send_calibrate(void);

// Parse and execute a string command (used by MQTT and CLI)
void cmd_parse_and_execute(const char *payload_str);

#endif // CROSSCORE_CMD_H
