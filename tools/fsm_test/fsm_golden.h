/**
 * @file fsm_golden.h
 * @brief Golden reference table for Core 1 FSM transition matrix.
 *
 * This file documents every (state, event) pair that has a defined
 * transition in the current switch-based FSM (src/core1_main.c).
 *
 * Two categories of entries:
 *  - PRESERVED : behavior identical in old switch and new fsm_dispatch().
 *  - DELIBERATE: cells that today fall to default:break (state unchanged)
 *    and will intentionally transition to ST_FAULT after the refactor.
 *    These represent hardware failure conditions (endstop missed, encoder
 *    disconnected) that currently cause indefinite hangs.
 *
 * Usage:
 *  Commit 1: test_fsm_golden.c validates expected_current via a software
 *            simulator of the current switch (no hardware required).
 *  Commit 4: the same test re-runs against fsm_dispatch() and validates
 *            expected_new for all entries.
 *
 * Note on StallGuard: TMC2209 SG_RESULT is unreliable at low speeds (homing,
 * release), which is exactly the regime where fault detection matters most.
 * StallGuard is therefore NOT used as a safety mechanism; it remains available
 * as optional diagnostic telemetry only. See Layers 2-3 in fsm_table.c.
 *
 * IEC 62304 traceability: this table is the evidence artifact for FSM
 * coverage analysis. Do not remove entries without updating the risk record.
 */

#ifndef FSM_GOLDEN_H
#define FSM_GOLDEN_H

#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------------
 * Enum mirrors — self-contained copy for host compilation.
 * Keep in sync with src/core1_main.h.  The numeric values MUST match.
 * ------------------------------------------------------------------------- */
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
    ST_COUNT   /* sentinel — must be last */
} Core1State_t;

typedef enum {
    EV_NONE = 0,
    EV_CMD_HOME,
    EV_CMD_SEARCH_SYRINGE,
    EV_CMD_START_DISPENSE,
    EV_CMD_SEARCH_EOT,
    EV_CMD_RESET,
    EV_CMD_CONTINUE_DISPENSE,
    EV_CMD_OCC_RELEASE,
    EV_CMD_RESUME_DISPENSE,
    EV_CMD_CALIBRATE,
    iEV_LSW_START_HIT,
    iEV_LSW_START_RELEASED,
    iEV_LSW_END_HIT,
    iEV_LSW_END_RELEASED,
    iEV_CONTACT_DETECTED,
    iEV_TARGET_REACHED,
    iEV_ENCODER_FAULT,
    iEV_OCCLUSION_DETECTED,
    iEV_OCC_RELEASED,
    iEV_LSW_END_CONTINUE,   /* dead event — no active handlers, remove Commit 6 */
    EV_COUNT   /* sentinel — must be last; update when adding iEV_ENCODER_STALL, iEV_TIMEOUT */
} Core1Event_t;

/* ST_ANY: sentinel used in GoldenEntry_t.from to denote a global transition
 * (evaluated before the per-state switch, applies to every non-fault state). */
#define ST_ANY  ((Core1State_t)ST_COUNT)

/* -------------------------------------------------------------------------
 * Entry type
 * ------------------------------------------------------------------------- */
typedef enum {
    ENTRY_PRESERVED,  /* new fsm_dispatch() must match expected_current exactly */
    ENTRY_DELIBERATE  /* current: stay (default:break); new: ST_FAULT */
} GoldenKind_t;

typedef struct {
    Core1State_t from;             /* source state; ST_ANY = global          */
    Core1Event_t event;            /* triggering event                        */
    Core1State_t expected_current; /* next state produced by today's switch   */
    Core1State_t expected_new;     /* next state produced by fsm_dispatch()   */
    GoldenKind_t kind;
    /* Guards that affect the outcome of this specific entry */
    bool         guard_motor_moving;   /* relevant only for iEV_LSW_*_HIT    */
    bool         guard_calibrating;    /* relevant only for iEV_LSW_START_RELEASED */
    bool         guard_cl_correction;  /* relevant only for iEV_TARGET_REACHED in DISPENSING/MANUAL */
    const char  *note;
} GoldenEntry_t;

/* -------------------------------------------------------------------------
 * Golden table — 28 preserved + 6 deliberate changes = 34 entries total.
 * Ordered: globals first, then locals by state value.
 * ------------------------------------------------------------------------- */
#define GOLDEN_TABLE_SIZE 34

extern const GoldenEntry_t GOLDEN_TABLE[GOLDEN_TABLE_SIZE];

/* -------------------------------------------------------------------------
 * Human-readable name helpers (implemented in test_fsm_golden.c)
 * ------------------------------------------------------------------------- */
const char *golden_state_name(Core1State_t s);
const char *golden_event_name(Core1Event_t e);

#endif /* FSM_GOLDEN_H */
