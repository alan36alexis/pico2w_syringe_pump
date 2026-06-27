/**
 * @file test_fsm_golden.c
 * @brief Host-executable FSM equivalence test.
 *
 * Compile and run (Linux/macOS/WSL):
 *   gcc -Wall -Wextra -o test_golden test_fsm_golden.c && ./test_golden
 *
 * Compile and run (Windows, MinGW-w64 via GNU Octave):
 *   set PATH=C:\Program Files\GNU Octave\Octave-10.1.0\mingw64\bin;%PATH%
 *   gcc -Wall -Wextra -o test_golden.exe test_fsm_golden.c && test_golden.exe
 *
 * Exit code: 0 = all preserved entries pass, 1 = at least one failure.
 *
 * Commit 1: validates expected_current via simulate_switch().
 * Commit 4: add --new flag to validate expected_new via fsm_dispatch().
 */

#include "fsm_golden.h"

#include <stdio.h>

/* =========================================================================
 * Software simulator — mirrors src/core1_main.c switch logic WITHOUT
 * hardware calls. Pure state-machine logic only.
 *
 * Parameters:
 *   from            current state before the event
 *   event           the event to process
 *   motor_moving    true if tmc2209_is_moving() would return true
 *   is_calibrating  true if post_lsw_start_state == ST_CALIB_SEEK_END
 *   cl_needs_corr   true if closed_loop_calculate_correction() returns true
 *
 * Returns the next state.
 * ========================================================================= */
static Core1State_t simulate_switch(Core1State_t from, Core1Event_t event,
                                    bool motor_moving, bool is_calibrating,
                                    bool cl_needs_corr)
{
    Core1State_t s  = from;
    Core1Event_t ev = event;

    /* ------------------------------------------------------------------
     * Global LSW_START_HIT handler (pre-switch, evaluated first).
     * Mirror of core1_main.c:653-671.
     * ------------------------------------------------------------------ */
    if (ev == iEV_LSW_START_HIT && s != ST_FAULT) {
        if (motor_moving) {
            /* tmc2209_abort_profile_dma() -> wait for motor to coast to stop */
            return ST_BRAKING_LSW_START;
        } else {
            /* Motor already stopped on switch: start slow release move */
            return ST_RELEASING_LSW_START;
        }
    }

    /* ------------------------------------------------------------------
     * Global LSW_END_HIT handler (pre-switch).
     * Mirror of core1_main.c:674-697.
     * ------------------------------------------------------------------ */
    if (ev == iEV_LSW_END_HIT && s != ST_FAULT) {
        if (motor_moving) {
            return ST_BRAKING_LSW_END;
        } else {
            return ST_RELEASING_LSW_END;
        }
    }

    /* ------------------------------------------------------------------
     * Global ENCODER_FAULT handler (pre-switch).
     * Mirror of core1_main.c:731-735.
     * ------------------------------------------------------------------ */
    if (ev == iEV_ENCODER_FAULT && s != ST_FAULT) {
        return ST_FAULT;
    }

    /* ------------------------------------------------------------------
     * Per-state switch.
     * Mirror of core1_main.c:737-949.
     * ------------------------------------------------------------------ */
    switch (s) {
    case ST_UNHOMED:
        if (ev == EV_CMD_HOME)      return ST_HOMING;
        if (ev == EV_CMD_CALIBRATE) return ST_CALIB_SEEK_START;
        break;

    case ST_HOMING:
        /* Waits for iEV_LSW_START_HIT (global) - no local events handled. */
        break;

    case ST_CALIB_SEEK_START:
        /* Waits for iEV_LSW_START_HIT (global). */
        break;

    case ST_CALIB_SEEK_END:
        /* Waits for iEV_LSW_END_HIT (global). */
        break;

    case ST_BRAKING_LSW_START:
        if (ev == iEV_TARGET_REACHED) return ST_RELEASING_LSW_START;
        break;

    case ST_BRAKING_LSW_END:
        if (ev == iEV_TARGET_REACHED) return ST_RELEASING_LSW_END;
        break;

    case ST_RELEASING_LSW_START:
        if (ev == iEV_LSW_START_RELEASED) {
            /* Bifurcation: calibration run goes to CALIB_SEEK_END; normal
             * homing goes to READY_AT_HOME. */
            return is_calibrating ? ST_CALIB_SEEK_END : ST_READY_AT_HOME;
        }
        break;

    case ST_READY_AT_HOME:
        if (ev == EV_CMD_SEARCH_SYRINGE) return ST_SEARCHING_SYRINGE;
        break;

    case ST_SEARCHING_SYRINGE:
        if (ev == iEV_CONTACT_DETECTED) return ST_SYRINGE_ENGAGED;
        break;

    case ST_SYRINGE_ENGAGED:
        if (ev == EV_CMD_START_DISPENSE) return ST_DISPENSING;
        break;

    case ST_DISPENSING:
        if (ev == iEV_TARGET_REACHED) {
            /* closed_loop_calculate_correction() decides whether to loop. */
            return cl_needs_corr ? ST_DISPENSING : ST_DISPENSE_COMPLETED;
        }
        if (ev == iEV_OCCLUSION_DETECTED) return ST_OCCLUSION_STOPPING;
        break;

    case ST_DISPENSE_COMPLETED:
        if (ev == EV_CMD_RESET)             return ST_UNHOMED;
        if (ev == EV_CMD_CONTINUE_DISPENSE) return ST_SET_NEW_DISPENSE;
        if (ev == EV_CMD_SEARCH_EOT)        return ST_SEARCHING_EOT;
        break;

    case ST_SET_NEW_DISPENSE:
        if (ev == EV_CMD_START_DISPENSE) return ST_DISPENSING;
        break;

    case ST_SEARCHING_EOT:
        /* Waits for iEV_LSW_END_HIT (global). */
        break;

    case ST_RELEASING_LSW_END:
        if (ev == iEV_LSW_END_RELEASED) return ST_END_OF_TRAVEL;
        break;

    case ST_END_OF_TRAVEL:
        if (ev == EV_CMD_RESET) return ST_UNHOMED;
        break;

    case ST_OCCLUSION_STOPPING:
        if (ev == EV_CMD_OCC_RELEASE) return ST_OCCLUSION_RELEASE;
        break;

    case ST_OCCLUSION_RELEASE:
        if (ev == iEV_OCC_RELEASED) return ST_OCCLUSION_PAUSED;
        break;

    case ST_OCCLUSION_PAUSED:
        if (ev == EV_CMD_RESUME_DISPENSE) return ST_DISPENSING;
        break;

    case ST_MANUAL_OVERRIDE:
        if (ev == EV_CMD_RESET) return ST_UNHOMED;
        if (ev == iEV_TARGET_REACHED) {
            /* Whether or not CL correction is needed, state stays in
             * MANUAL_OVERRIDE (correction move is issued internally). */
            return ST_MANUAL_OVERRIDE;
        }
        break;

    case ST_FAULT:
        if (ev == EV_CMD_RESET) return ST_UNHOMED;
        break;

    default:
        break;
    }

    return s; /* default: state unchanged (mirrors default: break) */
}

/* =========================================================================
 * Golden table definition — 34 entries.
 * Use explicit .field = value designators throughout to avoid positional
 * ambiguity with bool fields preceding the note string pointer.
 * ========================================================================= */
const GoldenEntry_t GOLDEN_TABLE[GOLDEN_TABLE_SIZE] = {

    /* -- GLOBAL transitions (ST_ANY, evaluated before per-state switch) -- */

    /* [0] LSW_START hit while motor moving -> brake to a stop */
    { .from = ST_ANY, .event = iEV_LSW_START_HIT,
      .expected_current = ST_BRAKING_LSW_START, .expected_new = ST_BRAKING_LSW_START,
      .kind = ENTRY_PRESERVED, .guard_motor_moving = true,
      .note = "Global: hit start endstop while moving -> brake" },

    /* [1] LSW_START hit while motor stopped (startup case) -> release immediately */
    { .from = ST_ANY, .event = iEV_LSW_START_HIT,
      .expected_current = ST_RELEASING_LSW_START, .expected_new = ST_RELEASING_LSW_START,
      .kind = ENTRY_PRESERVED, .guard_motor_moving = false,
      .note = "Global: start endstop already pressed at rest -> release" },

    /* [2] LSW_END hit while motor moving -> brake to a stop */
    { .from = ST_ANY, .event = iEV_LSW_END_HIT,
      .expected_current = ST_BRAKING_LSW_END, .expected_new = ST_BRAKING_LSW_END,
      .kind = ENTRY_PRESERVED, .guard_motor_moving = true,
      .note = "Global: hit end endstop while moving -> brake" },

    /* [3] LSW_END hit while motor stopped -> release immediately */
    { .from = ST_ANY, .event = iEV_LSW_END_HIT,
      .expected_current = ST_RELEASING_LSW_END, .expected_new = ST_RELEASING_LSW_END,
      .kind = ENTRY_PRESERVED, .guard_motor_moving = false,
      .note = "Global: end endstop already pressed at rest -> release" },

    /* [4] Motor driver error -> safe state */
    { .from = ST_ANY, .event = iEV_ENCODER_FAULT,
      .expected_current = ST_FAULT, .expected_new = ST_FAULT,
      .kind = ENTRY_PRESERVED,
      .note = "Global: driver fault (drv_err/uv_cp) -> fault" },

    /* -- LOCAL transitions (per-state switch) -- */

    /* [5] Start homing sequence */
    { .from = ST_UNHOMED, .event = EV_CMD_HOME,
      .expected_current = ST_HOMING, .expected_new = ST_HOMING,
      .kind = ENTRY_PRESERVED,
      .note = "Unhomed -> start homing" },

    /* [6] Start calibration sequence */
    { .from = ST_UNHOMED, .event = EV_CMD_CALIBRATE,
      .expected_current = ST_CALIB_SEEK_START, .expected_new = ST_CALIB_SEEK_START,
      .kind = ENTRY_PRESERVED,
      .note = "Unhomed -> start calibration" },

    /* [7] Braking LSW_START: motor coasted to stop -> start slow release */
    { .from = ST_BRAKING_LSW_START, .event = iEV_TARGET_REACHED,
      .expected_current = ST_RELEASING_LSW_START, .expected_new = ST_RELEASING_LSW_START,
      .kind = ENTRY_PRESERVED,
      .note = "Braking start endstop: motor stopped -> begin release move" },

    /* [8] Braking LSW_END: motor coasted to stop -> start slow release */
    { .from = ST_BRAKING_LSW_END, .event = iEV_TARGET_REACHED,
      .expected_current = ST_RELEASING_LSW_END, .expected_new = ST_RELEASING_LSW_END,
      .kind = ENTRY_PRESERVED,
      .note = "Braking end endstop: motor stopped -> begin release move" },

    /* [9] LSW_START released -- normal homing path */
    { .from = ST_RELEASING_LSW_START, .event = iEV_LSW_START_RELEASED,
      .expected_current = ST_READY_AT_HOME, .expected_new = ST_READY_AT_HOME,
      .kind = ENTRY_PRESERVED, .guard_calibrating = false,
      .note = "Released start endstop (homing) -> ready at home" },

    /* [10] LSW_START released -- calibration path */
    { .from = ST_RELEASING_LSW_START, .event = iEV_LSW_START_RELEASED,
      .expected_current = ST_CALIB_SEEK_END, .expected_new = ST_CALIB_SEEK_END,
      .kind = ENTRY_PRESERVED, .guard_calibrating = true,
      .note = "Released start endstop (calibrating) -> seek end endstop" },

    /* [11] Home position confirmed, command to search syringe */
    { .from = ST_READY_AT_HOME, .event = EV_CMD_SEARCH_SYRINGE,
      .expected_current = ST_SEARCHING_SYRINGE, .expected_new = ST_SEARCHING_SYRINGE,
      .kind = ENTRY_PRESERVED,
      .note = "Ready at home -> search syringe" },

    /* [12] ADC contact voltage detected (>2.0V) */
    { .from = ST_SEARCHING_SYRINGE, .event = iEV_CONTACT_DETECTED,
      .expected_current = ST_SYRINGE_ENGAGED, .expected_new = ST_SYRINGE_ENGAGED,
      .kind = ENTRY_PRESERVED,
      .note = "Searching syringe: contact voltage -> syringe engaged" },

    /* [13] Dispense command received */
    { .from = ST_SYRINGE_ENGAGED, .event = EV_CMD_START_DISPENSE,
      .expected_current = ST_DISPENSING, .expected_new = ST_DISPENSING,
      .kind = ENTRY_PRESERVED,
      .note = "Syringe engaged -> start dispense" },

    /* [14] Motor reached dispense target, no closed-loop correction needed */
    { .from = ST_DISPENSING, .event = iEV_TARGET_REACHED,
      .expected_current = ST_DISPENSE_COMPLETED, .expected_new = ST_DISPENSE_COMPLETED,
      .kind = ENTRY_PRESERVED, .guard_cl_correction = false,
      .note = "Dispensing: target reached, CL ok -> dispense completed" },

    /* [15] Occlusion detected during dispense (ADC >3.0V) */
    { .from = ST_DISPENSING, .event = iEV_OCCLUSION_DETECTED,
      .expected_current = ST_OCCLUSION_STOPPING, .expected_new = ST_OCCLUSION_STOPPING,
      .kind = ENTRY_PRESERVED,
      .note = "Dispensing: occlusion detected -> stop for occlusion" },

    /* [16] Dispense complete, reset to home */
    { .from = ST_DISPENSE_COMPLETED, .event = EV_CMD_RESET,
      .expected_current = ST_UNHOMED, .expected_new = ST_UNHOMED,
      .kind = ENTRY_PRESERVED,
      .note = "Dispense completed -> reset to unhomed" },

    /* [17] Continue dispense with new parameters */
    { .from = ST_DISPENSE_COMPLETED, .event = EV_CMD_CONTINUE_DISPENSE,
      .expected_current = ST_SET_NEW_DISPENSE, .expected_new = ST_SET_NEW_DISPENSE,
      .kind = ENTRY_PRESERVED,
      .note = "Dispense completed -> set new dispense parameters" },

    /* [18] Search for end-of-travel after dispense */
    { .from = ST_DISPENSE_COMPLETED, .event = EV_CMD_SEARCH_EOT,
      .expected_current = ST_SEARCHING_EOT, .expected_new = ST_SEARCHING_EOT,
      .kind = ENTRY_PRESERVED,
      .note = "Dispense completed -> search end of travel" },

    /* [19] Dispense command with updated parameters */
    { .from = ST_SET_NEW_DISPENSE, .event = EV_CMD_START_DISPENSE,
      .expected_current = ST_DISPENSING, .expected_new = ST_DISPENSING,
      .kind = ENTRY_PRESERVED,
      .note = "New dispense params set -> start dispense" },

    /* [20] LSW_END released -> carriage at physical end of travel */
    { .from = ST_RELEASING_LSW_END, .event = iEV_LSW_END_RELEASED,
      .expected_current = ST_END_OF_TRAVEL, .expected_new = ST_END_OF_TRAVEL,
      .kind = ENTRY_PRESERVED,
      .note = "Released end endstop -> end of travel" },

    /* [21] Reset from end-of-travel */
    { .from = ST_END_OF_TRAVEL, .event = EV_CMD_RESET,
      .expected_current = ST_UNHOMED, .expected_new = ST_UNHOMED,
      .kind = ENTRY_PRESERVED,
      .note = "End of travel -> reset to unhomed" },

    /* [22] Operator confirms occlusion, initiate backward release move */
    { .from = ST_OCCLUSION_STOPPING, .event = EV_CMD_OCC_RELEASE,
      .expected_current = ST_OCCLUSION_RELEASE, .expected_new = ST_OCCLUSION_RELEASE,
      .kind = ENTRY_PRESERVED,
      .note = "Occlusion stopping -> begin occlusion release move" },

    /* [23] Pressure dropped below threshold (ADC <2.0V) during release */
    { .from = ST_OCCLUSION_RELEASE, .event = iEV_OCC_RELEASED,
      .expected_current = ST_OCCLUSION_PAUSED, .expected_new = ST_OCCLUSION_PAUSED,
      .kind = ENTRY_PRESERVED,
      .note = "Occlusion release: pressure cleared -> paused" },

    /* [24] Resume infusion after occlusion clearance */
    { .from = ST_OCCLUSION_PAUSED, .event = EV_CMD_RESUME_DISPENSE,
      .expected_current = ST_DISPENSING, .expected_new = ST_DISPENSING,
      .kind = ENTRY_PRESERVED,
      .note = "Occlusion paused -> resume dispensing" },

    /* [25] Manual override: reset to unhomed */
    { .from = ST_MANUAL_OVERRIDE, .event = EV_CMD_RESET,
      .expected_current = ST_UNHOMED, .expected_new = ST_UNHOMED,
      .kind = ENTRY_PRESERVED,
      .note = "Manual override -> reset to unhomed" },

    /* [26] Manual override: closed-loop correction cycle (stays in MANUAL) */
    { .from = ST_MANUAL_OVERRIDE, .event = iEV_TARGET_REACHED,
      .expected_current = ST_MANUAL_OVERRIDE, .expected_new = ST_MANUAL_OVERRIDE,
      .kind = ENTRY_PRESERVED,
      .note = "Manual override: target reached -> stay (CL correction or no-op)" },

    /* [27] Fault recovery */
    { .from = ST_FAULT, .event = EV_CMD_RESET,
      .expected_current = ST_UNHOMED, .expected_new = ST_UNHOMED,
      .kind = ENTRY_PRESERVED,
      .note = "Fault -> reset to unhomed" },

    /* -- DELIBERATE CHANGES (silent default:break today -> ST_FAULT after refactor) -- */

    /* [28] Homing: moved 105mm without hitting endstop -> sensor/wiring fault */
    { .from = ST_HOMING, .event = iEV_TARGET_REACHED,
      .expected_current = ST_HOMING,  /* today: default:break -- hangs */
      .expected_new = ST_FAULT,
      .kind = ENTRY_DELIBERATE,
      .note = "DELIBERATE: homing target reached without LSW hit -> endstop fault" },

    /* [29] EOT search: moved 105mm without hitting endstop */
    { .from = ST_SEARCHING_EOT, .event = iEV_TARGET_REACHED,
      .expected_current = ST_SEARCHING_EOT,
      .expected_new = ST_FAULT,
      .kind = ENTRY_DELIBERATE,
      .note = "DELIBERATE: EOT search target reached without LSW hit -> endstop fault" },

    /* [30] Calibration seek start: missed start endstop */
    { .from = ST_CALIB_SEEK_START, .event = iEV_TARGET_REACHED,
      .expected_current = ST_CALIB_SEEK_START,
      .expected_new = ST_FAULT,
      .kind = ENTRY_DELIBERATE,
      .note = "DELIBERATE: calib seek start target reached without LSW -> endstop fault" },

    /* [31] Calibration seek end: missed end endstop */
    { .from = ST_CALIB_SEEK_END, .event = iEV_TARGET_REACHED,
      .expected_current = ST_CALIB_SEEK_END,
      .expected_new = ST_FAULT,
      .kind = ENTRY_DELIBERATE,
      .note = "DELIBERATE: calib seek end target reached without LSW -> endstop fault" },

    /* [32] Releasing start LSW: 1M steps without switch releasing */
    { .from = ST_RELEASING_LSW_START, .event = iEV_TARGET_REACHED,
      .expected_current = ST_RELEASING_LSW_START,
      .expected_new = ST_FAULT,
      .kind = ENTRY_DELIBERATE,
      .note = "DELIBERATE: release start LSW exhausted steps without release -> fault" },

    /* [33] Releasing end LSW: 1M steps without switch releasing */
    { .from = ST_RELEASING_LSW_END, .event = iEV_TARGET_REACHED,
      .expected_current = ST_RELEASING_LSW_END,
      .expected_new = ST_FAULT,
      .kind = ENTRY_DELIBERATE,
      .note = "DELIBERATE: release end LSW exhausted steps without release -> fault" },
};

/* =========================================================================
 * Name helpers -- for readable test output.
 * ========================================================================= */
const char *golden_state_name(Core1State_t s)
{
    switch (s) {
    case ST_UNHOMED:              return "ST_UNHOMED";
    case ST_HOMING:               return "ST_HOMING";
    case ST_RELEASING_LSW_START:  return "ST_RELEASING_LSW_START";
    case ST_READY_AT_HOME:        return "ST_READY_AT_HOME";
    case ST_SEARCHING_SYRINGE:    return "ST_SEARCHING_SYRINGE";
    case ST_SYRINGE_ENGAGED:      return "ST_SYRINGE_ENGAGED";
    case ST_DISPENSING:           return "ST_DISPENSING";
    case ST_DISPENSE_COMPLETED:   return "ST_DISPENSE_COMPLETED";
    case ST_SET_NEW_DISPENSE:     return "ST_SET_NEW_DISPENSE";
    case ST_SEARCHING_EOT:        return "ST_SEARCHING_EOT";
    case ST_RELEASING_LSW_END:    return "ST_RELEASING_LSW_END";
    case ST_END_OF_TRAVEL:        return "ST_END_OF_TRAVEL";
    case ST_FAULT:                return "ST_FAULT";
    case ST_OCCLUSION_STOPPING:   return "ST_OCCLUSION_STOPPING";
    case ST_OCCLUSION_RELEASE:    return "ST_OCCLUSION_RELEASE";
    case ST_OCCLUSION_PAUSED:     return "ST_OCCLUSION_PAUSED";
    case ST_MANUAL_OVERRIDE:      return "ST_MANUAL_OVERRIDE";
    case ST_CALIB_SEEK_START:     return "ST_CALIB_SEEK_START";
    case ST_CALIB_SEEK_END:       return "ST_CALIB_SEEK_END";
    case ST_BRAKING_LSW_START:    return "ST_BRAKING_LSW_START";
    case ST_BRAKING_LSW_END:      return "ST_BRAKING_LSW_END";
    case ST_COUNT:                return "ST_ANY(global)";
    default:                      return "ST_UNKNOWN";
    }
}

const char *golden_event_name(Core1Event_t e)
{
    switch (e) {
    case EV_NONE:                  return "EV_NONE";
    case EV_CMD_HOME:              return "EV_CMD_HOME";
    case EV_CMD_SEARCH_SYRINGE:    return "EV_CMD_SEARCH_SYRINGE";
    case EV_CMD_START_DISPENSE:    return "EV_CMD_START_DISPENSE";
    case EV_CMD_SEARCH_EOT:        return "EV_CMD_SEARCH_EOT";
    case EV_CMD_RESET:             return "EV_CMD_RESET";
    case EV_CMD_CONTINUE_DISPENSE: return "EV_CMD_CONTINUE_DISPENSE";
    case EV_CMD_OCC_RELEASE:       return "EV_CMD_OCC_RELEASE";
    case EV_CMD_RESUME_DISPENSE:   return "EV_CMD_RESUME_DISPENSE";
    case EV_CMD_CALIBRATE:         return "EV_CMD_CALIBRATE";
    case iEV_LSW_START_HIT:        return "iEV_LSW_START_HIT";
    case iEV_LSW_START_RELEASED:   return "iEV_LSW_START_RELEASED";
    case iEV_LSW_END_HIT:          return "iEV_LSW_END_HIT";
    case iEV_LSW_END_RELEASED:     return "iEV_LSW_END_RELEASED";
    case iEV_CONTACT_DETECTED:     return "iEV_CONTACT_DETECTED";
    case iEV_TARGET_REACHED:       return "iEV_TARGET_REACHED";
    case iEV_ENCODER_FAULT:        return "iEV_ENCODER_FAULT";
    case iEV_OCCLUSION_DETECTED:   return "iEV_OCCLUSION_DETECTED";
    case iEV_OCC_RELEASED:         return "iEV_OCC_RELEASED";
    case iEV_LSW_END_CONTINUE:     return "iEV_LSW_END_CONTINUE(dead)";
    case EV_COUNT:                 return "EV_COUNT(sentinel)";
    default:                       return "EV_UNKNOWN";
    }
}

/* =========================================================================
 * Test runner
 * ========================================================================= */
int main(void)
{
    int preserved_pass = 0, preserved_fail = 0;
    int deliberate_documented = 0;
    int total_entries = (int)(sizeof(GOLDEN_TABLE) / sizeof(GOLDEN_TABLE[0]));

    printf("=== FSM Golden Table Test (Commit 1 -- current behavior) ===\n");
    printf("Entries: %d\n\n", total_entries);

    for (int i = 0; i < total_entries; i++) {
        const GoldenEntry_t *e = &GOLDEN_TABLE[i];

        if (e->kind == ENTRY_DELIBERATE) {
            /* Document the deliberate change -- verify current behavior is
             * "state unchanged" (the default:break path), then note the
             * intended post-refactor behavior. */
            Core1State_t result = simulate_switch(
                e->from, e->event,
                e->guard_motor_moving, e->guard_calibrating,
                e->guard_cl_correction);
            bool current_ok = (result == e->expected_current);
            printf("[%2d] DELIBERATE  %-28s + %-30s\n"
                   "     current %-24s %s  |  new -> ST_FAULT\n"
                   "     %s\n\n",
                   i,
                   golden_state_name(e->from),
                   golden_event_name(e->event),
                   golden_state_name(result),
                   current_ok ? "(confirmed default:break)" : "(UNEXPECTED - check sim!)",
                   e->note);
            deliberate_documented++;
            continue;
        }

        /* For global entries (ST_ANY), test against a representative non-fault
         * state to confirm the global handler fires. */
        Core1State_t test_from = (e->from == ST_ANY) ? ST_HOMING : e->from;

        Core1State_t result = simulate_switch(
            test_from,
            e->event,
            e->guard_motor_moving,
            e->guard_calibrating,
            e->guard_cl_correction);

        bool pass = (result == e->expected_current);

        if (pass) {
            printf("[%2d] PASS  %-28s + %-30s -> %s\n",
                   i,
                   (e->from == ST_ANY) ? "ST_ANY(global)" : golden_state_name(e->from),
                   golden_event_name(e->event),
                   golden_state_name(result));
            preserved_pass++;
        } else {
            printf("[%2d] FAIL  %-28s + %-30s\n"
                   "     expected %-22s  got %s\n"
                   "     %s\n",
                   i,
                   (e->from == ST_ANY) ? "ST_ANY(global)" : golden_state_name(e->from),
                   golden_event_name(e->event),
                   golden_state_name(e->expected_current),
                   golden_state_name(result),
                   e->note);
            preserved_fail++;
        }
    }

    printf("\n=== Summary ===\n");
    printf("Preserved  : %d PASS, %d FAIL\n", preserved_pass, preserved_fail);
    printf("Deliberate : %d documented (today default:break -> after refactor ST_FAULT)\n",
           deliberate_documented);

    if (preserved_fail == 0) {
        printf("\nRESULT: ALL PRESERVED ENTRIES PASS.\n");
        return 0;
    } else {
        printf("\nRESULT: %d PRESERVED ENTRIES FAILED.\n", preserved_fail);
        return 1;
    }
}
