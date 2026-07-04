/**
 * @file fsm_table.h
 * @brief Table-driven FSM for Core 1 motion states.
 *
 * Architecture: two tables + policy.
 *
 *   GLOBAL[]      — High-priority rows with from == ST_COUNT ("any state").
 *                   Evaluated FIRST. Applies to all states except ST_FAULT
 *                   (dispatcher skips GLOBAL when state == ST_FAULT).
 *
 *   TRANSITIONS[] — Per-state rows. Evaluated after GLOBAL fails to match.
 *
 *   default_policy() — Called when no row matches. Returns POL_IGNORE or
 *                      POL_ILLEGAL. No silent default:break allowed.
 *
 * Dispatch order preserves the priority implicit in the original switch:
 *   global (LSW, fault, stall, timeout) > local state logic.
 *
 * IEC 62304 traceability: fsm_audit_coverage() produces a runtime coverage
 * matrix. Every (state, event) cell must be classified; SIN_CLASIFICAR == 0
 * is a build criterion for each commit that modifies the table.
 *
 * StallGuard note: TMC2209 SG_RESULT is unreliable at low speeds (the
 * exact regime where homing and release moves operate), so it is NOT used
 * as a safety mechanism. It remains available as optional diagnostic
 * telemetry only. Safety detection uses Layers 1–3 (spatial completeness,
 * encoder stall window, and deadline timeout).
 */

#ifndef FSM_TABLE_H
#define FSM_TABLE_H

#include "core1_main.h"
#include "closed_loop.h"
#include "tmc2209.h"

#include "hardware/pio.h"

#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Policy for unmatched (state, event) cells
 * ------------------------------------------------------------------------- */
typedef enum {
    POL_IGNORE  = 0, /* Event is irrelevant in this state. Stay, no log.      */
    POL_ILLEGAL = 1, /* Event is physically impossible in this state.          */
                     /* -> ST_FAULT + log (Decision A, applied in Commit 6).  */
    POL_VALID   = 2, /* Internal: cell is covered by a table row.             */
} FsmPolicy_t;

/* -------------------------------------------------------------------------
 * FSM context — all mutable state that action/guard functions may access.
 * Populated by core1_main.c before calling fsm_dispatch().
 * ------------------------------------------------------------------------- */
typedef struct {
    /* ---- Motor ---- */
    TMC2209_t *motor;

    /* Linear move launcher — points to tmc2209_move_linear_um_dma() wrapper
     * defined in core1_main.c. Using a function pointer avoids exposing a
     * static local function across translation units. */
    void      (*move_linear_fn)(TMC2209_t *motor, float target_um, float vel_ums);

    /* Current command payload (set by core1_main.c when dispatching CMD events) */
    float      cmd_target_um;
    float      cmd_velocity_ums;

    /* Motor moving state — updated each polling cycle by core1_main.c BEFORE
     * calling fsm_dispatch(). Guards read this field; they do NOT call the
     * hardware directly, keeping dispatch hardware-free. */
    bool       motor_is_moving;

    /* ---- Encoder / PIO ---- */
    PIO        pio_enc;
    uint       sm_enc_q;   /* quadrature state machine */
    uint       sm_enc_a;   /* pulse-counter channel A  */
    uint       sm_enc_b;   /* pulse-counter channel B  */
    bool       use_quadrature;
    int32_t   *last_encoder_count;
    int32_t   *last_encoder_a;
    int32_t   *last_encoder_b;
    int32_t   *last_speed_encoder_count;

    /* ---- Closed-loop ---- */
    ClosedLoopState_t *scl;

    /* ---- Calibration bifurcation guard (Decision B) ----
     * Set to true by act_calib_seek_start; cleared by any reset action.
     * The ST_RELEASING_LSW_START bifurcation uses guard_is_calibrating /
     * guard_not_calibrating to select the correct post-release state. */
    bool       is_calibrating;

    /* ---- Calibration velocities ----
     * Set by core1_main.c when CMD_CALIBRATE arrives (payload or defaults).
     * seek vel must persist in the ctx: act_seek_calib_end fires cycles later
     * (on iEV_LSW_START_RELEASED), when cmd_velocity_ums is no longer valid. */
    float      calib_move_vel_ums;   /* Fase 1: hacia LSW_START */
    float      calib_seek_vel_ums;   /* Fase 2: hacia LSW_END   */

    /* ---- Layer 2: encoder stall detection ----
     * Populated/checked by the polling section in core1_main.c.
     * When stall is detected, core1_main.c injects iEV_ENCODER_STALL
     * before calling fsm_dispatch(). */
    int32_t    last_stall_encoder_count;
    uint32_t   stall_window_start_ms;
    bool       stall_window_active;

    /* ---- Layer 3: derived deadline timeout ----
     * Calculated by action functions when launching a move:
     *   deadline_ms = now_ms + DEADLINE_K * t_nominal_ms + DEADLINE_FLOOR_MS
     * Checked by polling section; triggers iEV_TIMEOUT when exceeded. */
    uint32_t   deadline_ms;
    bool       deadline_active;

    /* ---- Current encoder snapshot ----
     * core1_main.c reads the encoder and stores it here immediately before
     * calling fsm_dispatch().  Action functions that need the encoder count
     * (closed_loop_init_move, calibration save) read this field instead of
     * accessing the PIO hardware directly. */
    int32_t    current_encoder_count;

    /* ---- Calibration-complete callback ----
     * Set by core1_main.c (Commit 5) to save calibrated_max_encoder_count to
     * g_sys_config and set g_calibration_dirty.  NULL-safe: action checks
     * before calling.  Called by act_abort_brake_end when is_calibrating. */
    void     (*on_calibration_complete_fn)(int32_t max_encoder_count);
} FsmCtx_t;

/* -------------------------------------------------------------------------
 * Transition table entry
 * ------------------------------------------------------------------------- */
typedef void (*FsmAction_t)(FsmCtx_t *ctx);
typedef bool (*FsmGuard_t)(const FsmCtx_t *ctx);

typedef struct {
    Core1State_t  from;     /* Source state. ST_COUNT == "any state" (GLOBAL). */
    Core1Event_t  trigger;  /* Event that fires this row.                       */
    Core1State_t  to;       /* Destination state after action.                  */
    FsmAction_t   action;   /* Called before state change. NULL == no action.   */
    FsmGuard_t    guard;    /* Must return true for row to fire. NULL == always. */
} Transition_t;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * @brief Dispatch one event through the FSM.
 *
 * Evaluation order:
 *   1. GLOBAL[] (skipped when state == ST_FAULT)
 *   2. TRANSITIONS[] (first row where from==state, trigger==event, guard passes)
 *   3. fsm_default_policy() for unmatched cells
 *
 * The matched action is called BEFORE the state transition.
 *
 * @param ctx    Mutable FSM context; must be initialized by core1_main.c
 * @param state  Current FSM state
 * @param event  Event to process (EV_NONE is a no-op)
 * @return       Next FSM state
 */
Core1State_t fsm_dispatch(FsmCtx_t *ctx, Core1State_t state, Core1Event_t event);

/**
 * @brief Determine the default policy for an unmatched (state, event) pair.
 *
 * Returns POL_IGNORE or POL_ILLEGAL. Never returns POL_VALID (that is only
 * used by fsm_audit_coverage() to mark table-covered cells).
 *
 * Current stub: returns POL_IGNORE for all cells.
 * Commit 6 refines this with the full classification.
 */
FsmPolicy_t fsm_default_policy(Core1State_t state, Core1Event_t event);

/**
 * @brief Audit FSM table coverage over the full ST_COUNT × EV_COUNT matrix.
 *
 * For each cell, checks:
 *   - Covered by GLOBAL[] (applies to all states except ST_FAULT)
 *   - Covered by TRANSITIONS[] (specific from/trigger pair)
 *   - Classified by fsm_default_policy() as IGNORE or ILLEGAL
 *   - SIN_CLASIFICAR: cell not covered by any of the above (must be 0)
 *
 * Commit 3 extends this to assert that every "wait state" (HOMING,
 * SEARCHING_EOT, CALIB_*, RELEASING_*) has at least one exit to ST_FAULT.
 *
 * Outputs via printf (host builds) or LOG_DEBUG (embedded).
 */
void fsm_audit_coverage(void);

/* ---- Guards ---- */
bool guard_is_calibrating(const FsmCtx_t *ctx);
bool guard_not_calibrating(const FsmCtx_t *ctx);
bool guard_motor_moving(const FsmCtx_t *ctx);
bool guard_motor_stopped(const FsmCtx_t *ctx);

#endif /* FSM_TABLE_H */
