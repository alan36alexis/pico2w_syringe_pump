#ifndef CMD_DISPATCHER_H
#define CMD_DISPATCHER_H

#include <stdbool.h>
#include <stdint.h>

// Origin of a command. Stored in DtoManualOp_t.source for traceability.
typedef enum {
    CMD_SRC_SERIAL = 0,
    CMD_SRC_MQTT   = 1,
    CMD_SRC_HMI    = 2,
} CmdSource_t;

typedef struct {
    bool        accepted;
    const char *reason;  // "ok" | "invalid_state" | "bad_format" | "unknown_cmd" | "queue_full"
} CmdDispatchResult_t;

// Single entry point for all external command sources.
// Parses str once, validates FSM state for motor commands, dispatches to crosscore_cmd.
// Emits EV_APP_CMD_EXECUTED with the correct source field.
// cid is the MQTT correlation ID for the caller's ACK — not used inside this function.
CmdDispatchResult_t cmd_dispatch_string(const char *str, CmdSource_t src, int32_t cid);

#endif // CMD_DISPATCHER_H
