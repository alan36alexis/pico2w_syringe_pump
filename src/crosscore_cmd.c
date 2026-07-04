#include "crosscore_cmd.h"

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

bool cmd_send_calibrate(float move_velocity_ums, float seek_velocity_ums) {
    Core1CmdMessage_t msg;
    msg.id = CMD_CALIBRATE;
    msg.payload.calibrate.move_velocity_ums = move_velocity_ums;
    msg.payload.calibrate.seek_velocity_ums = seek_velocity_ums;
    return queue_try_add(&crosscore_cmd_queue, &msg);
}
