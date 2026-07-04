#ifndef CORE1_MAIN_H
#define CORE1_MAIN_H

#include <stdint.h>

typedef enum {
    ST_UNHOMED = 0,
    ST_HOMING,
    ST_RELEASING_LSW_START,
    ST_READY_AT_HOME,
    ST_SEARCHING_SYRINGE,
    ST_SYRINGE_ENGAGED,
    ST_DISPENSING,
    ST_DISPENSE_COMPLETED,
    ST_SET_NEW_DISPENSE,
    ST_SEARCHING_EOT,
    ST_RELEASING_LSW_END,
    ST_END_OF_TRAVEL,
    ST_FAULT,
    ST_OCCLUSION_STOPPING,
    ST_OCCLUSION_RELEASE,
    ST_OCCLUSION_PAUSED,
    ST_MANUAL_OVERRIDE,
    ST_CALIB_SEEK_START,
    ST_CALIB_SEEK_END,
    ST_BRAKING_LSW_START,
    ST_BRAKING_LSW_END,
    ST_COUNT  /* sentinel — fsm_table.c uses this for _Static_assert and audit */
} Core1State_t;

typedef enum {
    EV_NONE = 0,
    // External Commands (Core 0 -> Core 1)
    EV_CMD_HOME,
    EV_CMD_SEARCH_SYRINGE,
    EV_CMD_START_DISPENSE,
    EV_CMD_SEARCH_EOT,
    EV_CMD_RESET,
    EV_CMD_CONTINUE_DISPENSE,
    EV_CMD_OCC_RELEASE,
    EV_CMD_RESUME_DISPENSE,
    EV_CMD_CALIBRATE,

    // Internal Events (Generated in Core 1)
    iEV_LSW_START_HIT,
    iEV_LSW_START_RELEASED,
    iEV_LSW_END_HIT,
    iEV_LSW_END_RELEASED,
    iEV_CONTACT_DETECTED,
    iEV_TARGET_REACHED,
    iEV_ENCODER_FAULT,
    iEV_OCCLUSION_DETECTED,
    iEV_OCC_RELEASED,
    iEV_LSW_END_CONTINUE,  /* dead event — no active handlers; remove in Commit 6 */

    /* Fault-detection events (Commit 3) */
    iEV_ENCODER_STALL,  /* Layer 2: encoder count unchanged while DMA active   */
    iEV_TIMEOUT,        /* Layer 3: deadline exceeded (calc'd from dist/vel)   */

    EV_COUNT  /* sentinel — keep in sync with fsm_table.c _Static_assert */
} Core1Event_t;

/**
 * @brief Active total-travel calibration in encoder counts (LSW_START to
 * LSW_END).  Defined in core1_main.c.  Updated by a completed calibration
 * routine; restored from Flash by Core 0 at boot (task_init) since Core 1
 * starts before the config is loaded.  Falls back to
 * MAX_TRAVEL_ENCODER_COUNT when no stored calibration is valid.
 */
extern volatile int32_t calibration_max_encoder_count;

/**
 * @brief Entry point function for Core 1 (Baremetal)
 */
void core1_main(void);

/**
 * @brief Returns a human-readable string for a Core1State_t value.
 */
const char *get_state_name(Core1State_t state);

#endif // CORE1_MAIN_H
