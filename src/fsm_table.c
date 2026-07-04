/**
 * @file fsm_table.c
 * @brief Table-driven FSM implementation for Core 1 motion states.
 *
 * Commit 4 state: all action functions make real driver calls.
 * Wired into core1_main.c under #ifdef ENABLE_FSM_TABLE in Commit 5.
 *
 * Table sizes asserted at compile time via _Static_assert.
 * Run fsm_audit_coverage() at boot to verify SIN_CLASIFICAR == 0.
 *
 * IEC 62304 change log:
 *   Commit 2 — baseline table structure; 28 preserved transitions + stubs.
 *   Commit 3 — add Layer 1 fault rows + iEV_ENCODER_STALL + iEV_TIMEOUT.
 *   Commit 4 — replace stubs with real driver calls; validate golden table.
 */

#include "fsm_table.h"
#include "system_config.h"

#include "pico/time.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* -------------------------------------------------------------------------
 * Compile-time coverage: table sizes must match enum sentinels.
 *
 * ST_COUNT == 21  (ST_UNHOMED=0 … ST_BRAKING_LSW_END=20, sentinel=21)
 * EV_COUNT == 22  (EV_NONE=0 … iEV_TIMEOUT=21, sentinel=22)
 * Update these values and the fsm_golden.h copy whenever states/events change.
 * ------------------------------------------------------------------------- */
_Static_assert(ST_COUNT == 21,
    "ST_COUNT changed — update TRANSITIONS table, audit, and this assertion");
_Static_assert(EV_COUNT == 22,
    "EV_COUNT changed — update GLOBAL/TRANSITIONS tables, audit, and this assertion");

/* -------------------------------------------------------------------------
 * Private helper macros (Commit 4)
 * ------------------------------------------------------------------------- */

/* Mirror of RESET_ENCODER_COUNTS() from core1_main.c.
 * Uses the PIO fields stored in FsmCtx_t instead of function-local vars. */
#define RESET_ENCODER_VIA_CTX(ctx)  do {                                       \
    if ((ctx)->use_quadrature) {                                                \
        pio_sm_exec((ctx)->pio_enc, (ctx)->sm_enc_q, 0xe040u); /* set y,0 */  \
    } else {                                                                    \
        pio_sm_exec((ctx)->pio_enc, (ctx)->sm_enc_a, 0xa02bu);                 \
        pio_sm_exec((ctx)->pio_enc, (ctx)->sm_enc_b, 0xa02bu);                 \
    }                                                                           \
    *(ctx)->last_encoder_count       = 0;                                       \
    *(ctx)->last_encoder_a           = 0;                                       \
    *(ctx)->last_encoder_b           = 0;                                       \
    *(ctx)->last_speed_encoder_count = 0;                                       \
    (ctx)->scl->start_encoder_count_cl = 0;                                    \
} while (0)

/* Set the Layer-3 deadline for a move of |dist_um| at vel_ums. */
#define SET_DEADLINE(ctx, dist_um, vel_ums) do {                               \
    uint32_t _t_nom = (uint32_t)(fabsf(dist_um) / (vel_ums) * 1000.0f);       \
    (ctx)->deadline_ms    = to_ms_since_boot(get_absolute_time())              \
                          + DEADLINE_K * _t_nom + DEADLINE_FLOOR_MS;           \
    (ctx)->deadline_active = true;                                              \
} while (0)

/* -------------------------------------------------------------------------
 * Action functions (Commit 4 — real driver calls)
 * ------------------------------------------------------------------------- */

/* Generic no-op: used for transitions that require no hardware side-effect. */
static void act_stub(FsmCtx_t *ctx) { (void)ctx; }

/* Reset to UNHOMED: clear flags that must not survive across reset.
 * is_calibrating MUST be cleared here — if a calibration aborted via fault
 * and then EV_CMD_RESET is sent, the guard_is_calibrating path in
 * ST_RELEASING_LSW_START would fire incorrectly on the next homing sequence. */
static void act_reset_to_unhomed(FsmCtx_t *ctx) {
    ctx->is_calibrating  = false;
    ctx->deadline_active = false;
}

/* ---- Global handlers ---- */

static void act_abort_brake_start(FsmCtx_t *ctx) {
    /* During calibration, LSW_START_HIT is the position reference — reset encoder
     * before braking so the count is zero at the known start point. */
    if (ctx->is_calibrating)
        RESET_ENCODER_VIA_CTX(ctx);
    tmc2209_abort_profile_dma(ctx->motor);
    ctx->deadline_active = false;
}

/* For LSW_END hit during calibration, save the max encoder count before
 * braking.  core1_main.c sets on_calibration_complete_fn in Commit 5. */
static void act_abort_brake_end(FsmCtx_t *ctx) {
    if (ctx->is_calibrating && ctx->on_calibration_complete_fn) {
        ctx->on_calibration_complete_fn(ctx->current_encoder_count);
    }
    tmc2209_abort_profile_dma(ctx->motor);
    ctx->deadline_active = false;
}

/* Motor was already stopped when LSW_START fired (e.g. at startup). */
static void act_release_lsw_start(FsmCtx_t *ctx) {
    if (ctx->is_calibrating)
        RESET_ENCODER_VIA_CTX(ctx);
    tmc2209_stop(ctx->motor);
    tmc2209_send_nsteps_at_freq(ctx->motor, 1000000, 2000.0f);
}

/* Motor was already stopped when LSW_END fired. */
static void act_release_lsw_end(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    tmc2209_send_nsteps_at_freq(ctx->motor, -1000000, 2000.0f);
}

/* Layer 1 / ENCODER_FAULT / stall / timeout: safe stop. */
static void act_fault_stop(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    ctx->deadline_active = false;
}

/* ---- Local (per-state) handlers ---- */

static void act_home_start(FsmCtx_t *ctx) {
    RESET_ENCODER_VIA_CTX(ctx);
    closed_loop_init_move(ctx->scl, -105000.0f, ctx->cmd_velocity_ums,
                          ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, -105000.0f, ctx->cmd_velocity_ums);
    SET_DEADLINE(ctx, -105000.0f, ctx->cmd_velocity_ums);
}

/* Begin calibration: seek toward LSW_START. Sets is_calibrating flag used by
 * the guard in ST_RELEASING_LSW_START to bifurcate toward ST_CALIB_SEEK_END. */
static void act_calib_seek_start(FsmCtx_t *ctx) {
    float vel = (ctx->calib_move_vel_ums > 0.0f) ? ctx->calib_move_vel_ums
                                                 : CALIBRATION_MOVE_SPEED;
    ctx->is_calibrating = true;
    closed_loop_init_move(ctx->scl, -105000.0f, vel, ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, -105000.0f, vel);
    SET_DEADLINE(ctx, -105000.0f, vel);
}

/* Normal homing complete (non-calibration path).  ST_READY_AT_HOME defines
 * the position reference: encoder count (and therefore position in mm) is
 * zero here, valid until the next homing. */
static void act_stop_at_home(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    RESET_ENCODER_VIA_CTX(ctx);
    ctx->is_calibrating  = false;
    ctx->deadline_active = false;
}

/* Calibration phase 2: after LSW_START released, seek LSW_END. */
static void act_seek_calib_end(FsmCtx_t *ctx) {
    float vel = (ctx->calib_seek_vel_ums > 0.0f) ? ctx->calib_seek_vel_ums
                                                 : CALIBRATION_SEEK_SPEED;
    tmc2209_stop(ctx->motor);
    closed_loop_init_move(ctx->scl, 105000.0f, vel, ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, 105000.0f, vel);
    SET_DEADLINE(ctx, 105000.0f, vel);
}

/* After braking on LSW_START: begin controlled release. */
static void act_send_nsteps_release_start(FsmCtx_t *ctx) {
    tmc2209_send_nsteps_at_freq(ctx->motor, 1000000, 2000.0f);
}

/* After braking on LSW_END: begin controlled release. */
static void act_send_nsteps_release_end(FsmCtx_t *ctx) {
    tmc2209_send_nsteps_at_freq(ctx->motor, -1000000, 2000.0f);
}

static void act_search_syringe(FsmCtx_t *ctx) {
    closed_loop_init_move(ctx->scl, 105000.0f, ctx->cmd_velocity_ums,
                          ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, 105000.0f, ctx->cmd_velocity_ums);
    SET_DEADLINE(ctx, 105000.0f, ctx->cmd_velocity_ums);
}

static void act_stop_on_contact(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    ctx->deadline_active = false;
}

static void act_start_dispense(FsmCtx_t *ctx) {
    closed_loop_init_move(ctx->scl, ctx->cmd_target_um, ctx->cmd_velocity_ums,
                          ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, ctx->cmd_target_um, ctx->cmd_velocity_ums);
    SET_DEADLINE(ctx, ctx->cmd_target_um, ctx->cmd_velocity_ums);
}

/* CL correction check is intercepted by core1_main.c before iEV_TARGET_REACHED
 * reaches the FSM: only dispatched when CL is satisfied.  This action just
 * clears the deadline. */
static void act_dispense_target_reached(FsmCtx_t *ctx) {
    ctx->deadline_active = false;
}

static void act_abort_occlusion(FsmCtx_t *ctx) {
    tmc2209_abort_profile_dma(ctx->motor);
    ctx->deadline_active = false;
}

static void act_search_eot(FsmCtx_t *ctx) {
    closed_loop_init_move(ctx->scl, 105000.0f, 1200.0f, ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, 105000.0f, 1200.0f);
    SET_DEADLINE(ctx, 105000.0f, 1200.0f);
}

static void act_stop_at_eot(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    ctx->deadline_active = false;
}

static void act_occ_release_move(FsmCtx_t *ctx) {
    closed_loop_init_move(ctx->scl, -105000.0f, 200.0f, ctx->current_encoder_count);
    ctx->move_linear_fn(ctx->motor, -105000.0f, 200.0f);
    SET_DEADLINE(ctx, -105000.0f, 200.0f);
}

static void act_stop_occ_paused(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    ctx->deadline_active = false;
}

/* Transition only: original switch did not re-launch the move here.
 * core1_main.c handles the CL correction when the resumed move completes. */
static void act_resume_dispense(FsmCtx_t *ctx) { (void)ctx; }

static void act_manual_stop(FsmCtx_t *ctx) {
    tmc2209_stop(ctx->motor);
    ctx->is_calibrating  = false;
    ctx->deadline_active = false;
}

/* CL correction for ST_MANUAL_OVERRIDE is handled by core1_main.c (same
 * pattern as ST_DISPENSING).  The FSM stays in ST_MANUAL_OVERRIDE. */
static void act_manual_cl_correction(FsmCtx_t *ctx) { (void)ctx; }

/* -------------------------------------------------------------------------
 * Guards
 * ------------------------------------------------------------------------- */
bool guard_is_calibrating(const FsmCtx_t *ctx)  { return ctx->is_calibrating; }
bool guard_not_calibrating(const FsmCtx_t *ctx) { return !ctx->is_calibrating; }
bool guard_motor_moving(const FsmCtx_t *ctx)    { return ctx->motor_is_moving; }
bool guard_motor_stopped(const FsmCtx_t *ctx)   { return !ctx->motor_is_moving; }

/* -------------------------------------------------------------------------
 * GLOBAL table — evaluated first for all states EXCEPT ST_FAULT.
 * Rows with from == ST_COUNT apply to every non-fault state.
 *
 * Preserves current priority: LSW events and driver faults interrupt anything.
 * Commit 3 adds: iEV_ENCODER_STALL, iEV_TIMEOUT.
 * ------------------------------------------------------------------------- */
static const Transition_t GLOBAL[] = {
    /* LSW_START hit while motor is moving -> controlled brake */
    { ST_COUNT, iEV_LSW_START_HIT, ST_BRAKING_LSW_START,
      act_abort_brake_start, guard_motor_moving },

    /* LSW_START hit while motor is already stopped (startup / manual) */
    { ST_COUNT, iEV_LSW_START_HIT, ST_RELEASING_LSW_START,
      act_release_lsw_start, guard_motor_stopped },

    /* LSW_END hit while motor is moving -> controlled brake */
    { ST_COUNT, iEV_LSW_END_HIT, ST_BRAKING_LSW_END,
      act_abort_brake_end, guard_motor_moving },

    /* LSW_END hit while motor is already stopped */
    { ST_COUNT, iEV_LSW_END_HIT, ST_RELEASING_LSW_END,
      act_release_lsw_end, guard_motor_stopped },

    /* TMC2209 driver error (drv_err / uv_cp from GSTAT) */
    { ST_COUNT, iEV_ENCODER_FAULT, ST_FAULT,
      act_fault_stop, NULL },

    /* Layer 2: encoder stall — DMA active but encoder count unchanged for
     * ENCODER_STALL_WINDOW_MS. Injected by the polling section in
     * core1_main.c before calling fsm_dispatch(). */
    { ST_COUNT, iEV_ENCODER_STALL, ST_FAULT,
      act_fault_stop, NULL },

    /* Layer 3: deadline exceeded — move took > DEADLINE_K * t_nominal +
     * DEADLINE_FLOOR_MS. Covers a step-generator that stops pulsing without
     * emitting iEV_TARGET_REACHED or moving the encoder. */
    { ST_COUNT, iEV_TIMEOUT, ST_FAULT,
      act_fault_stop, NULL },
};
#define GLOBAL_SIZE ((int)(sizeof(GLOBAL) / sizeof(GLOBAL[0])))

/* -------------------------------------------------------------------------
 * TRANSITIONS table — per-state local transitions, evaluated after GLOBAL.
 * Row order matters when the same (from, trigger) appears twice with guards
 * (e.g., the calibration bifurcation at ST_RELEASING_LSW_START).
 * ------------------------------------------------------------------------- */
static const Transition_t TRANSITIONS[] = {

    /* ST_UNHOMED */
    { ST_UNHOMED, EV_CMD_HOME,      ST_HOMING,          act_home_start,       NULL },
    { ST_UNHOMED, EV_CMD_CALIBRATE, ST_CALIB_SEEK_START, act_calib_seek_start, NULL },

    /* ST_HOMING — Layer 1 (Commit 3): motor ran full stroke without triggering LSW_START */
    { ST_HOMING, iEV_TARGET_REACHED, ST_FAULT, act_fault_stop, NULL },

    /* ST_BRAKING_LSW_* — motor coasted to stop, begin release move */
    { ST_BRAKING_LSW_START, iEV_TARGET_REACHED, ST_RELEASING_LSW_START,
      act_send_nsteps_release_start, NULL },
    { ST_BRAKING_LSW_END, iEV_TARGET_REACHED, ST_RELEASING_LSW_END,
      act_send_nsteps_release_end, NULL },

    /* ST_RELEASING_LSW_START — bifurcation on is_calibrating.
     * Guard order: not_calibrating checked first (common path). */
    { ST_RELEASING_LSW_START, iEV_LSW_START_RELEASED, ST_READY_AT_HOME,
      act_stop_at_home, guard_not_calibrating },
    { ST_RELEASING_LSW_START, iEV_LSW_START_RELEASED, ST_CALIB_SEEK_END,
      act_seek_calib_end, guard_is_calibrating },
    /* Layer 1: 1M release steps without LSW_START_RELEASED */
    { ST_RELEASING_LSW_START, iEV_TARGET_REACHED, ST_FAULT, act_fault_stop, NULL },

    /* ST_READY_AT_HOME */
    { ST_READY_AT_HOME, EV_CMD_SEARCH_SYRINGE, ST_SEARCHING_SYRINGE,
      act_search_syringe, NULL },

    /* ST_SEARCHING_SYRINGE */
    { ST_SEARCHING_SYRINGE, iEV_CONTACT_DETECTED, ST_SYRINGE_ENGAGED,
      act_stop_on_contact, NULL },

    /* ST_SYRINGE_ENGAGED */
    { ST_SYRINGE_ENGAGED, EV_CMD_START_DISPENSE, ST_DISPENSING,
      act_start_dispense, NULL },

    /* ST_DISPENSING — no CL correction needed (Commit 4 handles loop case) */
    { ST_DISPENSING, iEV_TARGET_REACHED,    ST_DISPENSE_COMPLETED,
      act_dispense_target_reached, NULL },
    { ST_DISPENSING, iEV_OCCLUSION_DETECTED, ST_OCCLUSION_STOPPING,
      act_abort_occlusion, NULL },

    /* ST_DISPENSE_COMPLETED */
    { ST_DISPENSE_COMPLETED, EV_CMD_RESET,             ST_UNHOMED,          act_reset_to_unhomed, NULL },
    { ST_DISPENSE_COMPLETED, EV_CMD_CONTINUE_DISPENSE, ST_SET_NEW_DISPENSE, act_stub,             NULL },
    { ST_DISPENSE_COMPLETED, EV_CMD_SEARCH_EOT,        ST_SEARCHING_EOT,    act_search_eot,       NULL },

    /* ST_SET_NEW_DISPENSE */
    { ST_SET_NEW_DISPENSE, EV_CMD_START_DISPENSE, ST_DISPENSING,
      act_start_dispense, NULL },

    /* ST_SEARCHING_EOT — Layer 1 (Commit 3): motor ran full stroke without triggering LSW_END */
    { ST_SEARCHING_EOT, iEV_TARGET_REACHED, ST_FAULT, act_fault_stop, NULL },

    /* ST_RELEASING_LSW_END */
    { ST_RELEASING_LSW_END, iEV_LSW_END_RELEASED, ST_END_OF_TRAVEL,
      act_stop_at_eot, NULL },
    /* Layer 1: 1M release steps without LSW_END_RELEASED */
    { ST_RELEASING_LSW_END, iEV_TARGET_REACHED, ST_FAULT, act_fault_stop, NULL },

    /* ST_CALIB_SEEK_START — Layer 1 (Commit 3): motor ran full stroke without triggering LSW_START */
    { ST_CALIB_SEEK_START, iEV_TARGET_REACHED, ST_FAULT, act_fault_stop, NULL },

    /* ST_CALIB_SEEK_END — Layer 1 (Commit 3): motor ran full stroke without triggering LSW_END */
    { ST_CALIB_SEEK_END, iEV_TARGET_REACHED, ST_FAULT, act_fault_stop, NULL },

    /* ST_END_OF_TRAVEL */
    { ST_END_OF_TRAVEL, EV_CMD_RESET, ST_UNHOMED, act_reset_to_unhomed, NULL },

    /* ST_OCCLUSION_STOPPING */
    { ST_OCCLUSION_STOPPING, EV_CMD_OCC_RELEASE, ST_OCCLUSION_RELEASE,
      act_occ_release_move, NULL },

    /* ST_OCCLUSION_RELEASE */
    { ST_OCCLUSION_RELEASE, iEV_OCC_RELEASED, ST_OCCLUSION_PAUSED,
      act_stop_occ_paused, NULL },

    /* ST_OCCLUSION_PAUSED */
    { ST_OCCLUSION_PAUSED, EV_CMD_RESUME_DISPENSE, ST_DISPENSING,
      act_resume_dispense, NULL },

    /* ST_MANUAL_OVERRIDE */
    { ST_MANUAL_OVERRIDE, EV_CMD_RESET,       ST_UNHOMED,        act_manual_stop,          NULL },
    { ST_MANUAL_OVERRIDE, iEV_TARGET_REACHED, ST_MANUAL_OVERRIDE, act_manual_cl_correction, NULL },

    /* ST_FAULT */
    { ST_FAULT, EV_CMD_RESET, ST_UNHOMED, act_reset_to_unhomed, NULL },
};
#define TRANSITIONS_SIZE ((int)(sizeof(TRANSITIONS) / sizeof(TRANSITIONS[0])))

/* -------------------------------------------------------------------------
 * fsm_dispatch — evaluate GLOBAL then TRANSITIONS, apply policy on miss.
 * ------------------------------------------------------------------------- */
Core1State_t fsm_dispatch(FsmCtx_t *ctx, Core1State_t state, Core1Event_t event)
{
    if (event == EV_NONE) return state;

    /* 1. GLOBAL table — skipped entirely when in ST_FAULT */
    if (state != ST_FAULT) {
        for (int i = 0; i < GLOBAL_SIZE; i++) {
            if (GLOBAL[i].trigger != event) continue;
            if (GLOBAL[i].guard && !GLOBAL[i].guard(ctx)) continue;
            if (GLOBAL[i].action) GLOBAL[i].action(ctx);
            return GLOBAL[i].to;
        }
    }

    /* 2. TRANSITIONS table — first matching row with passing guard wins */
    for (int i = 0; i < TRANSITIONS_SIZE; i++) {
        if (TRANSITIONS[i].from != state)    continue;
        if (TRANSITIONS[i].trigger != event) continue;
        if (TRANSITIONS[i].guard && !TRANSITIONS[i].guard(ctx)) continue;
        if (TRANSITIONS[i].action) TRANSITIONS[i].action(ctx);
        return TRANSITIONS[i].to;
    }

    /* 3. Default policy for unmatched cells */
    FsmPolicy_t pol = fsm_default_policy(state, event);
    if (pol == POL_ILLEGAL) {
        /* Decision A (Alan, 2026-06-26): illegal transition -> ST_FAULT + log.
         * Full implementation deferred to Commit 6 pending state-by-state
         * review. Stub: log only, no fault yet. */
        printf("[FSM]: ILLEGAL (state=%d, event=%d) — not yet faulting (Commit 6)\n",
               (int)state, (int)event);
    }
    return state; /* POL_IGNORE or POL_ILLEGAL stub: stay in current state */
}

/* -------------------------------------------------------------------------
 * fsm_default_policy — stub: POL_IGNORE for all unmatched cells.
 * Commit 6 refines this with a full per-(state,event) classification.
 * ------------------------------------------------------------------------- */
FsmPolicy_t fsm_default_policy(Core1State_t state, Core1Event_t event)
{
    (void)state;
    (void)event;
    return POL_IGNORE;
}

/* -------------------------------------------------------------------------
 * fsm_audit_coverage — iterates the full matrix and classifies every cell.
 * SIN_CLASIFICAR must be 0; an assert fires if not.
 * ------------------------------------------------------------------------- */
void fsm_audit_coverage(void)
{
    int covered_global     = 0;
    int covered_local      = 0;
    int policy_ignore      = 0;
    int policy_illegal     = 0;
    int sin_clasificar     = 0;

    for (int s = 0; s < (int)ST_COUNT; s++) {
        for (int e = 0; e < (int)EV_COUNT; e++) {
            Core1State_t state = (Core1State_t)s;
            Core1Event_t event = (Core1Event_t)e;

            /* Skip EV_NONE — it is never dispatched */
            if (event == EV_NONE) continue;

            bool matched = false;

            /* Check GLOBAL[] (applies to all states except ST_FAULT) */
            if (state != ST_FAULT) {
                for (int g = 0; g < GLOBAL_SIZE && !matched; g++) {
                    if (GLOBAL[g].trigger == event) {
                        covered_global++;
                        matched = true;
                    }
                }
            }

            /* Check TRANSITIONS[] */
            if (!matched) {
                for (int t = 0; t < TRANSITIONS_SIZE && !matched; t++) {
                    if (TRANSITIONS[t].from == state &&
                        TRANSITIONS[t].trigger == event) {
                        covered_local++;
                        matched = true;
                    }
                }
            }

            /* Apply default policy */
            if (!matched) {
                FsmPolicy_t pol = fsm_default_policy(state, event);
                if      (pol == POL_IGNORE)   policy_ignore++;
                else if (pol == POL_ILLEGAL)  policy_illegal++;
                else {
                    /* POL_VALID returned by default_policy is a bug */
                    printf("[FSM_AUDIT]: SIN_CLASIFICAR: state=%d event=%d pol=%d\n",
                           s, e, (int)pol);
                    sin_clasificar++;
                }
            }
        }
    }

    int total = (int)ST_COUNT * ((int)EV_COUNT - 1); /* exclude EV_NONE */
    printf("[FSM_AUDIT]: total=%d  global=%d  local=%d  ignore=%d  illegal=%d  SIN_CLASIFICAR=%d\n",
           total, covered_global, covered_local,
           policy_ignore, policy_illegal, sin_clasificar);

    assert(sin_clasificar == 0 &&
           "fsm_audit_coverage: SIN_CLASIFICAR > 0 — add rows or refine default_policy()");

    /* Commit 3: verify every wait-state has at least one explicit fault exit in
     * TRANSITIONS[] (Layer 1 — spatial completeness).  GLOBAL[] provides
     * Layers 2 and 3 for all states automatically; this check is specifically
     * about Layer 1 coverage so that the audit catches if a new wait-state is
     * added to the enum without a corresponding fault row. */
    static const Core1State_t WAIT_STATES[] = {
        ST_HOMING,
        ST_SEARCHING_EOT,
        ST_CALIB_SEEK_START,
        ST_CALIB_SEEK_END,
        ST_RELEASING_LSW_START,
        ST_RELEASING_LSW_END,
    };
    int wait_state_count = (int)(sizeof(WAIT_STATES) / sizeof(WAIT_STATES[0]));
    int missing_fault_exit = 0;

    for (int w = 0; w < wait_state_count; w++) {
        Core1State_t ws = WAIT_STATES[w];
        bool has_fault_exit = false;
        for (int t = 0; t < TRANSITIONS_SIZE && !has_fault_exit; t++) {
            if (TRANSITIONS[t].from == ws && TRANSITIONS[t].to == ST_FAULT) {
                has_fault_exit = true;
            }
        }
        if (!has_fault_exit) {
            printf("[FSM_AUDIT]: MISSING Layer-1 fault exit for wait-state %d\n", (int)ws);
            missing_fault_exit++;
        }
    }

    printf("[FSM_AUDIT]: wait-state Layer-1 check: %d/%d covered, missing=%d\n",
           wait_state_count - missing_fault_exit, wait_state_count, missing_fault_exit);

    assert(missing_fault_exit == 0 &&
           "fsm_audit_coverage: a wait-state has no Layer 1 fault exit in TRANSITIONS[]");
}
