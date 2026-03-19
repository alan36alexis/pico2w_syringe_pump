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
