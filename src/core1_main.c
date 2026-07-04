#include <math.h>
#include <pico/time.h>
#include <stdbool.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "closed_loop.h"
#include "config_manager.h"
#include "core1_main.h"
#include "crosscore_cmd.h"
#include "crosscore_logger.h"
#include "fsm_table.h"
#include "pulse_counter.pio.h"
#include "quadrature_encoder.pio.h"
#include "system_config.h"
#include "system_events.h"
#include "tmc2209.h"

#define DEBUG_MODE 1

#if DEBUG_MODE
#define LOG_DEBUG(...) logger_send_string(__VA_ARGS__)
#else
#define LOG_DEBUG(...)
#endif

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

TMC2209_t *global_motor = NULL;

/* Last calibrated max encoder count, set by both the FSM-table callback and
 * the inline LSW_END handler.  g_sys_config.calibrated_max_encoder_count holds
 * the same value and is the authoritative persistent copy (saved to Flash by
 * Core 0 when g_calibration_dirty is set).  This variable exists as a local
 * volatile mirror so Core 1 can reference it without going through the config
 * struct on every access. */
volatile int32_t calibration_max_encoder_count = 0;

// ---------------------------------------------------------------------------
// Static prototypes
// ---------------------------------------------------------------------------

static void   motor_driver_init(TMC2209_t *motor);
static void   calibration_complete_callback(int32_t max_encoder_count);
static float  measure_encoder_speed(int32_t current_count,
                                    int32_t *last_count_ptr,
                                    uint32_t *last_time_us_ptr);
static inline int32_t read_encoder_count(PIO pio, uint sm_q, uint sm_a);
static inline void    reset_encoder_counts(PIO pio, uint sm_q, uint sm_a, uint sm_b,
                                           int32_t *last_count, int32_t *last_a,
                                           int32_t *last_b, int32_t *last_speed,
                                           ClosedLoopState_t *scl);
static void tmc2209_move_linear_um_dma(TMC2209_t *motor, float target_um,
                                       float target_velocity_ums);

// ---------------------------------------------------------------------------
// core1_main — entry point for Core 1 (baremetal, no FreeRTOS)
// ---------------------------------------------------------------------------

void core1_main(void) {
    multicore_lockout_victim_init();

    uint32_t counter           = 0;
    int32_t  last_encoder_count      = 0;
    int32_t  last_encoder_a          = 0;
    int32_t  last_encoder_b          = 0;
    uint8_t  last_tmc_flags          = 0;
    int32_t  last_speed_encoder_count = 0;
    uint32_t last_speed_calc_time    = time_us_32();

    ClosedLoopState_t scl;
    closed_loop_init(&scl);

    TMC2209_t motor1;
    global_motor = &motor1;
    motor_driver_init(&motor1);

    // --- Encoder (PIO) ---
    uint sm_enc_q = 0, sm_enc_a = 0, sm_enc_b = 0;
    if (ENABLE_ENCODER) {
        if (USE_QUADRATURE_ENCODER) {
            (void)pio_add_program(pio1, &quadrature_encoder_program);
            sm_enc_q = pio_claim_unused_sm(pio1, true);
            quadrature_encoder_program_init(pio1, sm_enc_q, ENCODER_PIN_A, 0);
        } else {
            uint offset  = pio_add_program(pio1, &pulse_counter_program);
            sm_enc_a = pio_claim_unused_sm(pio1, true);
            sm_enc_b = pio_claim_unused_sm(pio1, true);
            pulse_counter_program_init(pio1, sm_enc_a, offset, ENCODER_PIN_A);
            pulse_counter_program_init(pio1, sm_enc_b, offset, ENCODER_PIN_B);
        }
    }

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_CHANNEL);

    // --- FSM setup ---
    Core1State_t current_state      = ST_UNHOMED;
    Core1Event_t active_event       = EV_NONE;
    float fsm_home_velocity_ums     = FSM_HOME_VELOCITY_UMS;
    float fsm_search_velocity_ums   = FSM_SEARCH_VELOCITY_UMS;

    FsmCtx_t fsm_ctx = {0};
    fsm_ctx.motor                      = &motor1;
    fsm_ctx.move_linear_fn             = tmc2209_move_linear_um_dma;
    fsm_ctx.pio_enc                    = pio1;
    fsm_ctx.sm_enc_q                   = sm_enc_q;
    fsm_ctx.sm_enc_a                   = sm_enc_a;
    fsm_ctx.sm_enc_b                   = sm_enc_b;
    fsm_ctx.use_quadrature             = USE_QUADRATURE_ENCODER;
    fsm_ctx.last_encoder_count         = &last_encoder_count;
    fsm_ctx.last_encoder_a             = &last_encoder_a;
    fsm_ctx.last_encoder_b             = &last_encoder_b;
    fsm_ctx.last_speed_encoder_count   = &last_speed_encoder_count;
    fsm_ctx.scl                        = &scl;
    fsm_ctx.on_calibration_complete_fn = calibration_complete_callback;
    fsm_audit_coverage();

    // Statics declared before the loop so their init is visually co-located
    // with other setup code (behavior is identical — static locals init once).
    static uint8_t      start_sw_debounce    = 0;
    static uint8_t      end_sw_debounce      = 0;
    static bool         was_moving           = false;
    static Core1State_t post_lsw_start_state = ST_READY_AT_HOME;
    const  uint8_t      DEBOUNCE_THRESHOLD   = 3;

    while (true) {
        Core1State_t prev_state = current_state;
        Core1CmdMessage_t cmd;
        active_event = EV_NONE;
        bool have_cmd = false;

        // 1. IPC — STOP_IMMEDIATE / STOP_MOTOR jump the queue (safety priority)
        {
            Core1CmdMessage_t peek;
            Core1CmdMessage_t pending[10];
            int  pending_count   = 0;
            bool emergency_found = false;
            while (queue_try_remove(&crosscore_cmd_queue, &peek)) {
                if (peek.id == CMD_STOP_IMMEDIATE || peek.id == CMD_STOP_MOTOR) {
                    cmd = peek;
                    emergency_found = true;
                    break;
                }
                if (pending_count < 10) pending[pending_count++] = peek;
                if (pending_count == 10) break;
            }
            for (int _j = 0; _j < pending_count; _j++)
                queue_try_add(&crosscore_cmd_queue, &pending[_j]);

            have_cmd = emergency_found
                       ? true
                       : queue_try_remove(&crosscore_cmd_queue, &cmd);
        }

        if (have_cmd) {
            switch (cmd.id) {
            // FSM commands — translate to FSM events; dispatch happens in step 4
            case CMD_HOME:
                active_event = EV_CMD_HOME;
                fsm_home_velocity_ums = cmd.payload.move_home.target_velocity_ums;
                break;
            case CMD_SEARCH_SYRINGE:
                active_event = EV_CMD_SEARCH_SYRINGE;
                fsm_search_velocity_ums = cmd.payload.move_home.target_velocity_ums;
                break;
            case CMD_START_DISPENSE:    active_event = EV_CMD_START_DISPENSE;    break;
            case CMD_SEARCH_EOT:        active_event = EV_CMD_SEARCH_EOT;        break;
            case CMD_RESET:             active_event = EV_CMD_RESET;             break;
            case CMD_CONTINUE_DISPENSE: active_event = EV_CMD_CONTINUE_DISPENSE; break;
            case CMD_OCC_RELEASE:       active_event = EV_CMD_OCC_RELEASE;       break;
            case CMD_RESUME_DISPENSE:   active_event = EV_CMD_RESUME_DISPENSE;   break;
            case CMD_CALIBRATE:
                active_event = EV_CMD_CALIBRATE;
                fsm_ctx.calib_move_vel_ums =
                    (cmd.payload.calibrate.move_velocity_ums > 0.0f)
                        ? cmd.payload.calibrate.move_velocity_ums
                        : CALIBRATION_MOVE_SPEED;
                fsm_ctx.calib_seek_vel_ums =
                    (cmd.payload.calibrate.seek_velocity_ums > 0.0f)
                        ? cmd.payload.calibrate.seek_velocity_ums
                        : CALIBRATION_SEEK_SPEED;
                break;

            // Raw / debug commands — bypass FSM, execute immediately
            case CMD_MOVE_LINEAR_UM:
                current_state = ST_MANUAL_OVERRIDE;
                reset_encoder_counts(pio1, sm_enc_q, sm_enc_a, sm_enc_b,
                    &last_encoder_count, &last_encoder_a, &last_encoder_b,
                    &last_speed_encoder_count, &scl);
                logger_send_motor_moving();
                tmc2209_move_linear_um_dma(global_motor,
                    cmd.payload.move_linear.target_um,
                    cmd.payload.move_linear.target_velocity_ums);
                closed_loop_init_move(&scl,
                    cmd.payload.move_linear.target_um,
                    cmd.payload.move_linear.target_velocity_ums, 0);
                break;

            case CMD_STOP_MOTOR:
                current_state = ST_UNHOMED;
                if (tmc2209_is_moving(global_motor))
                    tmc2209_abort_profile_dma(global_motor);
                else {
                    tmc2209_stop(global_motor);
                    logger_send_string("Se detiene motor en CMD_STOP_MOTOR\n");
                }
                break;

            case CMD_MOVE_2PART_PROFILE:
                current_state = ST_MANUAL_OVERRIDE;
                reset_encoder_counts(pio1, sm_enc_q, sm_enc_a, sm_enc_b,
                    &last_encoder_count, &last_encoder_a, &last_encoder_b,
                    &last_speed_encoder_count, &scl);
                logger_send_motor_moving();
                tmc2209_move_2part_profile_dma(
                    global_motor,
                    cmd.payload.move_2part.start_freq,  cmd.payload.move_2part.a_p1,
                    cmd.payload.move_2part.f_mid_accel, cmd.payload.move_2part.a_p2,
                    cmd.payload.move_2part.f_target,    cmd.payload.move_2part.c_steps,
                    cmd.payload.move_2part.d_p1,        cmd.payload.move_2part.f_mid_decel,
                    cmd.payload.move_2part.d_p2,        cmd.payload.move_2part.f_end);
                {
                    float um_per_ustep = LEAD_SCREW_PITCH_UM /
                        (MOTOR_STEPS_PER_REV * REAL_GEARBOX_RATIO * MOTOR_MICROSTEPS_VAL);
                    closed_loop_init_move(&scl, 0.0f,
                        cmd.payload.move_2part.f_target * um_per_ustep, 0);
                }
                break;

            case CMD_HOME_START:
                current_state = ST_MANUAL_OVERRIDE;
                reset_encoder_counts(pio1, sm_enc_q, sm_enc_a, sm_enc_b,
                    &last_encoder_count, &last_encoder_a, &last_encoder_b,
                    &last_speed_encoder_count, &scl);
                logger_send_motor_moving();
                tmc2209_move_linear_um_dma(global_motor, -105000.0f, 1500.0f);
                break;

            case CMD_HOME_END:
                current_state = ST_MANUAL_OVERRIDE;
                reset_encoder_counts(pio1, sm_enc_q, sm_enc_a, sm_enc_b,
                    &last_encoder_count, &last_encoder_a, &last_encoder_b,
                    &last_speed_encoder_count, &scl);
                logger_send_motor_moving();
                tmc2209_move_linear_um_dma(global_motor, 105000.0f, 1500.0f);
                break;

            case CMD_MOVE_NSTEPS:
                current_state = ST_MANUAL_OVERRIDE;
                reset_encoder_counts(pio1, sm_enc_q, sm_enc_a, sm_enc_b,
                    &last_encoder_count, &last_encoder_a, &last_encoder_b,
                    &last_speed_encoder_count, &scl);
                logger_send_motor_moving();
                if (cmd.payload.move_nsteps.nsteps < 0) {
                    tmc2209_set_direction(global_motor, false);
                    tmc2209_send_nsteps_at_freq(global_motor,
                        -cmd.payload.move_nsteps.nsteps,
                        cmd.payload.move_nsteps.freq_hz);
                } else {
                    tmc2209_set_direction(global_motor, true);
                    tmc2209_send_nsteps_at_freq(global_motor,
                        cmd.payload.move_nsteps.nsteps,
                        cmd.payload.move_nsteps.freq_hz);
                }
                break;

            case CMD_STOP_IMMEDIATE:
                current_state = ST_UNHOMED;
                tmc2209_stop(global_motor);
                break;

            default:
                break;
            }
        }

        // 2. Hardware Events (limit switches, pressure sensor, motor-stopped)
        if (global_motor->limit_switches_enabled) {
            bool start_sw_active = gpio_get(global_motor->limit_switch_start_pin);
            bool end_sw_active   = gpio_get(global_motor->limit_switch_end_pin);

            if (start_sw_active) { if (start_sw_debounce < DEBOUNCE_THRESHOLD) start_sw_debounce++; }
            else                  { if (start_sw_debounce > 0) start_sw_debounce--; }
            if (end_sw_active)   { if (end_sw_debounce < DEBOUNCE_THRESHOLD) end_sw_debounce++; }
            else                  { if (end_sw_debounce > 0) end_sw_debounce--; }

            if      (start_sw_debounce >= DEBOUNCE_THRESHOLD && !global_motor->direction &&
                     current_state != ST_BRAKING_LSW_START &&
                     current_state != ST_RELEASING_LSW_START)
                active_event = iEV_LSW_START_HIT;
            else if (end_sw_debounce >= DEBOUNCE_THRESHOLD && global_motor->direction &&
                     current_state != ST_BRAKING_LSW_END &&
                     current_state != ST_RELEASING_LSW_END)
                active_event = iEV_LSW_END_HIT;
            else if (start_sw_debounce == 0 && current_state == ST_RELEASING_LSW_START)
                active_event = iEV_LSW_START_RELEASED;
            else if (end_sw_debounce == 0 && current_state == ST_RELEASING_LSW_END)
                active_event = iEV_LSW_END_RELEASED;
        }

        if (current_state == ST_SEARCHING_SYRINGE ||
            current_state == ST_DISPENSING         ||
            current_state == ST_OCCLUSION_RELEASE) {
            uint16_t adc_val = adc_read();
            float voltage = adc_val * 3.3f / (1 << 12);

            if (counter % 50 == 0 && g_log_filter.show_adc)
                LOG_DEBUG("[ADC]: ADC Voltage (State %d): %.2f V\n", current_state, voltage);

            if      (voltage > 2.0f && current_state == ST_SEARCHING_SYRINGE)
                active_event = iEV_CONTACT_DETECTED;
            else if (voltage > 3.0f && current_state == ST_DISPENSING)
                active_event = iEV_OCCLUSION_DETECTED;
            else if (voltage < 2.0f && current_state == ST_OCCLUSION_RELEASE)
                active_event = iEV_OCC_RELEASED;
        }

        bool is_moving = tmc2209_is_moving(global_motor);
        if (was_moving && !is_moving) {
            if (active_event != iEV_LSW_START_HIT && active_event != iEV_LSW_END_HIT)
                active_event = iEV_TARGET_REACHED;
            logger_send_motor_stopped();
        }
        was_moving = is_moving;

        // 3. Global LSW Handler — old switch path only; table path uses GLOBAL[]
#ifndef ENABLE_FSM_TABLE
        if (active_event == iEV_LSW_START_HIT && current_state != ST_FAULT) {
            if (current_state == ST_CALIB_SEEK_START) {
                reset_encoder_counts(pio1, sm_enc_q, sm_enc_a, sm_enc_b,
                    &last_encoder_count, &last_encoder_a, &last_encoder_b,
                    &last_speed_encoder_count, &scl);
                post_lsw_start_state = ST_CALIB_SEEK_END;
            } else {
                post_lsw_start_state = ST_READY_AT_HOME;
            }
            if (tmc2209_is_moving(global_motor)) {
                tmc2209_abort_profile_dma(global_motor);
                current_state = ST_BRAKING_LSW_START;
            } else {
                tmc2209_stop(global_motor);
                tmc2209_send_nsteps_at_freq(global_motor, 1000000, 2000.0f);
                current_state = ST_RELEASING_LSW_START;
            }
            active_event = EV_NONE;
        }

        if (active_event == iEV_LSW_END_HIT && current_state != ST_FAULT) {
            if (current_state == ST_CALIB_SEEK_END) {
                int32_t enc = read_encoder_count(pio1, sm_enc_q, sm_enc_a);
                calibration_max_encoder_count             = enc;
                g_sys_config.calibrated_max_encoder_count = enc;
                g_sys_config.calibration_valid            = 1;
                g_calibration_dirty                       = true;
                LOG_DEBUG("[CFG]: Calibration Complete! Max Encoder Count: %d\n", enc);
            }
            if (tmc2209_is_moving(global_motor)) {
                tmc2209_abort_profile_dma(global_motor);
                current_state = ST_BRAKING_LSW_END;
            } else {
                tmc2209_stop(global_motor);
                tmc2209_send_nsteps_at_freq(global_motor, -1000000, 2000.0f);
                current_state = ST_RELEASING_LSW_END;
            }
            active_event = EV_NONE;
        }
#endif /* !ENABLE_FSM_TABLE */

        // 4. FSM Dispatch
        {
            int32_t enc_now = read_encoder_count(pio1, sm_enc_q, sm_enc_a);
            fsm_ctx.current_encoder_count = enc_now;
            fsm_ctx.motor_is_moving       = is_moving;

            // Load command payload for states that need it
            if (have_cmd) {
                switch (cmd.id) {
                case CMD_HOME:
                    fsm_ctx.cmd_velocity_ums = fsm_home_velocity_ums;
                    break;
                case CMD_SEARCH_SYRINGE:
                    fsm_ctx.cmd_velocity_ums = fsm_search_velocity_ums;
                    break;
                case CMD_START_DISPENSE:
                    fsm_ctx.cmd_target_um    = cmd.payload.start_dispense.target_um;
                    fsm_ctx.cmd_velocity_ums = cmd.payload.start_dispense.target_velocity_ums;
                    break;
                default:
                    break;
                }
            }

            // Layer 2: encoder stall — count unchanged while DMA active
            if (fsm_ctx.deadline_active && is_moving) {
                if (enc_now != fsm_ctx.last_stall_encoder_count) {
                    fsm_ctx.last_stall_encoder_count = enc_now;
                    fsm_ctx.stall_window_start_ms    = to_ms_since_boot(get_absolute_time());
                    fsm_ctx.stall_window_active      = true;
                } else if (fsm_ctx.stall_window_active) {
                    uint32_t stall_now = to_ms_since_boot(get_absolute_time());
                    if (stall_now - fsm_ctx.stall_window_start_ms > ENCODER_STALL_WINDOW_MS
                        && active_event == EV_NONE) {
                        LOG_DEBUG("[FSM]: Layer-2 stall in state %d\n", (int)current_state);
                        active_event = iEV_ENCODER_STALL;
                        fsm_ctx.stall_window_active = false;
                    }
                }
            } else {
                fsm_ctx.last_stall_encoder_count = enc_now;
                fsm_ctx.stall_window_start_ms    = to_ms_since_boot(get_absolute_time());
                fsm_ctx.stall_window_active      = false;
            }

            // Layer 3: derived deadline (calculated per-move by action functions)
            if (fsm_ctx.deadline_active && active_event == EV_NONE) {
                uint32_t dl_now = to_ms_since_boot(get_absolute_time());
                if (dl_now > fsm_ctx.deadline_ms) {
                    LOG_DEBUG("[FSM]: Layer-3 deadline exceeded in state %d\n",
                              (int)current_state);
                    active_event = iEV_TIMEOUT;
                }
            }

            // CL correction: intercept iEV_TARGET_REACHED to apply residual error
            if (active_event == iEV_TARGET_REACHED &&
                (current_state == ST_DISPENSING || current_state == ST_MANUAL_OVERRIDE)) {
                float missing_um = 0.0f;
                if (closed_loop_calculate_correction(&scl, enc_now,
                                                     USE_QUADRATURE_ENCODER, &missing_um)) {
                    LOG_DEBUG("[FSM]: CL correction %.1f um; suppressing iEV_TARGET_REACHED.\n",
                              missing_um);
                    tmc2209_move_linear_um_dma(global_motor, missing_um,
                                               scl.expected_target_velocity_ums);
                    active_event = EV_NONE;
                }
            }

            current_state = fsm_dispatch(&fsm_ctx, current_state, active_event);
        }

        // 5. TMC Driver Status
        uint32_t drv_status = tmc2209_read_drv_status(global_motor);
        uint32_t gstat      = tmc2209_read_gstat(global_motor);
#ifdef ENABLE_STALLGUARD_LOG
        uint16_t stall = tmc2209_read_sg_result(global_motor);
        (void)stall; // reserved: wire to DtoTmcStatus_t.stall_count when needed
#endif

        if (gstat & (TMC_GSTAT_DRV_ERR | TMC_GSTAT_UV_CP)) {
            if (active_event == EV_NONE && current_state != ST_FAULT)
                active_event = iEV_ENCODER_FAULT;
        }
        if (gstat & TMC_GSTAT_RESET)   tmc2209_clear_gstat(global_motor, 1);
        if (gstat & TMC_GSTAT_DRV_ERR) tmc2209_clear_gstat(global_motor, 2);
        if (gstat & TMC_GSTAT_UV_CP)   tmc2209_clear_gstat(global_motor, 4);

        uint8_t tmc_flags = 0;
        if (drv_status & TMC_DRV_OTPW)                    tmc_flags |= TMC_FLAG_OT_WARN;
        if (drv_status & TMC_DRV_OT)                      tmc_flags |= TMC_FLAG_OT_SHUT;
        if (drv_status & (TMC_DRV_S2GA | TMC_DRV_S2VSA)) tmc_flags |= TMC_FLAG_SHORT_A;
        if (drv_status & (TMC_DRV_S2GB | TMC_DRV_S2VSB)) tmc_flags |= TMC_FLAG_SHORT_B;
        if (drv_status & TMC_DRV_OLA)                     tmc_flags |= TMC_FLAG_OPEN_A;
        if (drv_status & TMC_DRV_OLB)                     tmc_flags |= TMC_FLAG_OPEN_B;
        if (gstat & TMC_GSTAT_UV_CP)                      tmc_flags |= TMC_FLAG_UV_CP;
        if (gstat & TMC_GSTAT_DRV_ERR)                    tmc_flags |= TMC_FLAG_DRV_ERR;

        if (tmc_flags != last_tmc_flags) {
            DtoTmcStatus_t dto = { .stall_count = 0, .flags = tmc_flags };
            CORE1_EMIT(EV_TMC_DRV_STATUS, tmc, dto);
            last_tmc_flags = tmc_flags;
        }

        // 6. Encoder Telemetry
        if (ENABLE_ENCODER) {
            if (USE_QUADRATURE_ENCODER) {
                int32_t current_count = quadrature_encoder_get_count(pio1, sm_enc_q);

                bool is_fsm_routine = (current_state >= ST_SEARCHING_SYRINGE &&
                                       current_state <= ST_OCCLUSION_PAUSED);
                if (counter % 50 == 0 && is_fsm_routine && is_moving) {
                    float pos_pct = tmc2209_get_move_progress_pct(global_motor);
                    if (g_log_filter.show_prg)
                        LOG_DEBUG("[PRG]: Position progress: %.1f%%\n", pos_pct);
                    logger_send_motor_progress(pos_pct);
                }

                if (counter % 10 == 0) {
                    float pps = measure_encoder_speed(current_count,
                                    &last_speed_encoder_count, &last_speed_calc_time);
                    if (is_moving) {
                        if (counter % 50 == 0)
                            logger_send_encoder_speed(fabsf(pps) * calc_um_per_pulse(true),
                                                      scl.expected_target_velocity_ums);
                        if (current_state != ST_BRAKING_LSW_START   &&
                            current_state != ST_BRAKING_LSW_END     &&
                            current_state != ST_RELEASING_LSW_START  &&
                            current_state != ST_RELEASING_LSW_END)
                            closed_loop_check_speed(&scl, pps, USE_QUADRATURE_ENCODER);
                    }
                }

                if (current_count != last_encoder_count) {
                    if (counter % 50 == 0 && is_moving)
                        logger_send_encoder_count(current_count,
                            current_count * calc_um_per_pulse(true) / 1000.0f);
                    last_encoder_count = current_count;
                }
            } else {
                int32_t a = pulse_counter_get_count(pio1, sm_enc_a);
                int32_t b = pulse_counter_get_count(pio1, sm_enc_b);

                if (counter % 10 == 0) {
                    float pps_a = measure_encoder_speed(a,
                                      &last_speed_encoder_count, &last_speed_calc_time);
                    if (is_moving) {
                        if (counter % 50 == 0)
                            logger_send_encoder_speed(fabsf(pps_a) * calc_um_per_pulse(false),
                                                      scl.expected_target_velocity_ums);
                        if (current_state != ST_BRAKING_LSW_START   &&
                            current_state != ST_BRAKING_LSW_END     &&
                            current_state != ST_RELEASING_LSW_START  &&
                            current_state != ST_RELEASING_LSW_END)
                            closed_loop_check_speed(&scl, pps_a, USE_QUADRATURE_ENCODER);
                    }
                }

                if (a != last_encoder_a || b != last_encoder_b) {
                    if (counter % 50 == 0 && is_moving)
                        logger_send_encoder_indep_counts(a, b);
                    last_encoder_a = a;
                    last_encoder_b = b;
                }
            }
        }

        if (current_state != prev_state)
            logger_send_fsm_state(current_state);

        counter++;
        sleep_ms(10);
    }
}

// ---------------------------------------------------------------------------
// Motor driver initialization
// ---------------------------------------------------------------------------

static void motor_driver_init(TMC2209_t *motor) {
    tmc2209_init(motor, MOTOR_STEP_PIN, MOTOR_DIR_PIN, MOTOR_ENA_PIN,
                 MOTOR_STEPS_PER_REV, MOTOR_MICROSTEPS,
                 TMC2209_NO_PIN, TMC2209_NO_PIN);
    sleep_ms(SENSOR_INIT_DELAY_MS);
    tmc2209_setup_limit_switches(motor, LIMIT_SW_START_PIN, LIMIT_SW_END_PIN);

    if (USE_UART_MODE) {
        LOG_DEBUG("[MTR]: Iniciando en MODO UART (Pines MS usados para direccionamiento 0)\n");
        tmc2209_set_uart_address_pins(motor, 0);
        LOG_DEBUG("[MTR]: Direccion configurada: %d\n", motor->addr);
        tmc2209_setup_uart(motor, uart1, TMC2209_UART_BAUD, 0, UART_TX_PIN, UART_RX_PIN);

        uint32_t check_uart = tmc2209_read_register(motor, 0x06);
        if (check_uart == 0) logger_send_uart_init_fail();
        else                 logger_send_uart_init_ok(check_uart);

        tmc2209_set_pdn_disable(motor, true);
        tmc2209_set_chopper_mode(motor, TMC2209_CHOPPER_DYNAMIC, 100);
        tmc2209_configure_chopconf(motor, 2, true);
        tmc2209_set_chopper_parameters(motor, 5, 0, 1);
        tmc2209_set_microstepping_uart(motor, MOTOR_MICROSTEPS);
        tmc2209_set_current_amps(motor, 0.6f, 0.3f);
        tmc2209_enable(motor, true);

        uint16_t msteps_read = tmc2209_get_microsteps(motor);
        logger_send_uart_init_microsteps_read(msteps_read);
    } else {
        logger_send_pins_init_mode();
        tmc2209_set_microstepping_by_pins(motor, MOTOR_MICROSTEPS);
    }
}

// ---------------------------------------------------------------------------
// Encoder helpers
// ---------------------------------------------------------------------------

static inline int32_t read_encoder_count(PIO pio, uint sm_q, uint sm_a) {
    if (!ENABLE_ENCODER) return 0;
    return USE_QUADRATURE_ENCODER
        ? quadrature_encoder_get_count(pio, sm_q)
        : pulse_counter_get_count(pio, sm_a);
}

static inline void reset_encoder_counts(
    PIO pio, uint sm_q, uint sm_a, uint sm_b,
    int32_t *last_count, int32_t *last_a, int32_t *last_b,
    int32_t *last_speed, ClosedLoopState_t *scl)
{
    if (!ENABLE_ENCODER) return;
    if (USE_QUADRATURE_ENCODER)
        pio_sm_exec(pio, sm_q, 0xe040);  // set y, 0
    else {
        pio_sm_exec(pio, sm_a, 0xa02b);  // mov x, ~null
        pio_sm_exec(pio, sm_b, 0xa02b);
    }
    *last_count = *last_a = *last_b = *last_speed = 0;
    scl->start_encoder_count_cl = 0;
}

// ---------------------------------------------------------------------------
// Speed measurement
// ---------------------------------------------------------------------------

static float measure_encoder_speed(int32_t current_count, int32_t *last_count_ptr,
                                   uint32_t *last_time_us_ptr) {
    uint32_t current_time = time_us_32();
    uint32_t delta_time   = current_time - *last_time_us_ptr;
    if (delta_time == 0) return 0.0f;

    float pps = ((float)(current_count - *last_count_ptr) * 1000000.0f) / (float)delta_time;
    *last_count_ptr   = current_count;
    *last_time_us_ptr = current_time;
    return pps;
}

// ---------------------------------------------------------------------------
// Calibration callback (called by act_abort_brake_end in fsm_table.c)
// ---------------------------------------------------------------------------

static void calibration_complete_callback(int32_t max_encoder_count) {
    calibration_max_encoder_count             = max_encoder_count;
    g_sys_config.calibrated_max_encoder_count = max_encoder_count;
    g_sys_config.calibration_valid            = 1;
    g_calibration_dirty                       = true;
    DtoCalibration_t dto = {
        .max_encoder_count = max_encoder_count,
        .travel_mm = max_encoder_count *
                     calc_um_per_pulse(USE_QUADRATURE_ENCODER) / 1000.0f,
        .trigger = 0,
        .success = 1
    };
    CORE1_EMIT(EV_APP_CALIBRATION, calibration, dto);
    LOG_DEBUG("[CFG]: Calibration Complete (FSM table)! Max Encoder Count: %d\n",
              max_encoder_count);
}

// ---------------------------------------------------------------------------
// Linear move abstraction — converts µm + µm/s to a DMA trapezoidal profile
// ---------------------------------------------------------------------------

static void tmc2209_move_linear_um_dma(TMC2209_t *motor, float target_um,
                                       float target_velocity_ums) {
    LOG_DEBUG("[MTR]: Abstraccion de Movimiento Lineal\n");
    if (g_log_filter.show_tgt)
        LOG_DEBUG("[TGT]: Target: %.1f um a %.1f um/s\n", target_um, target_velocity_ums);

    if (target_velocity_ums <= 0.0f || fabsf(target_um) < 1.0f) {
        LOG_DEBUG("[MTR]: Error: Velocidad cero o distancia cero.\n");
        return;
    }

    bool direction = (target_um > 0.0f);
    target_um = fabsf(target_um);

    // Drive profile lookup: microstep + chopper mode scale with velocity
    TMC2209_Microsteps_t  msteps;
    TMC2209_ChopperMode_t chop_mode;
    float run_amps;

    if (target_velocity_ums < MOTOR_THRESH_LOW_UMS) {
        msteps = TMC2209_MICROSTEPS_16; chop_mode = TMC2209_CHOPPER_STEALTHCHOP; run_amps = MOTOR_CURRENT_LOW_A;
    } else if (target_velocity_ums <= MOTOR_THRESH_MID_UMS) {
        msteps = TMC2209_MICROSTEPS_16; chop_mode = TMC2209_CHOPPER_SPREADCYCLE; run_amps = MOTOR_CURRENT_MID_A;
    } else if (target_velocity_ums <= MOTOR_THRESH_HIGH_UMS) {
        msteps = TMC2209_MICROSTEPS_8;  chop_mode = TMC2209_CHOPPER_SPREADCYCLE; run_amps = MOTOR_CURRENT_HIGH_A;
    } else {
        msteps = TMC2209_MICROSTEPS_2;  chop_mode = TMC2209_CHOPPER_SPREADCYCLE; run_amps = MOTOR_CURRENT_HIGH_A;
    }

    tmc2209_set_direction(motor, direction);
    tmc2209_stop(motor);
    sleep_ms(5);
    tmc2209_set_current_amps(motor, run_amps, 0.4f);
    tmc2209_set_chopper_mode(motor, chop_mode, 100);
    tmc2209_set_microstepping_uart(motor, msteps);

    uint16_t current_msteps_val = tmc2209_get_microsteps(motor);
    if (g_log_filter.show_cfg) {
        LOG_DEBUG("[CFG]: Configuracion OK: Microsteps=1/%d, Chopper=%s, IRUN=%.1fA\n",
            current_msteps_val,
            (chop_mode == TMC2209_CHOPPER_STEALTHCHOP) ? "StealthChop" : "SpreadCycle",
            run_amps);
    }

    // Kinematic conversion: µm → microsteps → Hz
    float um_per_full_step  = LEAD_SCREW_PITCH_UM / (MOTOR_STEPS_PER_REV * REAL_GEARBOX_RATIO);
    float um_per_microstep  = um_per_full_step / (float)current_msteps_val;
    uint32_t total_microsteps = (uint32_t)(target_um / um_per_microstep);

    if (total_microsteps < 100) {
        LOG_DEBUG("[MTR]: Advertencia: Movimiento muy corto (%u micropasos). Se enviara en burst.\n",
                  total_microsteps);
        tmc2209_send_nsteps_at_freq(motor, total_microsteps,
                                    target_velocity_ums / um_per_microstep);
        return;
    }

    float target_freq_hz = target_velocity_ums / um_per_microstep;
    float f_start        = (target_freq_hz * 0.1f < 50.0f) ? 50.0f : target_freq_hz * 0.1f;
    float f_mid_accel    = f_start + (target_freq_hz - f_start) * 0.8f;
    float f_mid_decel    = target_freq_hz - (target_freq_hz - f_start) * 0.2f;
    float f_end          = f_start;

    // Rampa por aceleración constante: N = (f_tgt² - f_start²) / (2·A)
    // A en micropasos/s², derivada de PROFILE_ACCEL_UMS2 según el microstepping activo
    float accel_steps_s2 = PROFILE_ACCEL_UMS2 / um_per_microstep;
    uint32_t pasos_aceleracion = (uint32_t)(
        (target_freq_hz * target_freq_hz - f_start * f_start) / (2.0f * accel_steps_s2));
    uint32_t pasos_frenado     = pasos_aceleracion;
    if (pasos_aceleracion < 20) { pasos_aceleracion = 20; pasos_frenado = 20; }

    uint32_t pasos_constantes = total_microsteps - (pasos_aceleracion + pasos_frenado);
    if (pasos_aceleracion * 2 >= total_microsteps) {
        pasos_aceleracion = total_microsteps / 2;
        pasos_frenado     = total_microsteps - pasos_aceleracion;
        pasos_constantes  = 0;
    }

    uint32_t a_p1 = pasos_aceleracion / 2,  a_p2 = pasos_aceleracion - a_p1;
    uint32_t d_p1 = pasos_frenado / 2,      d_p2 = pasos_frenado - d_p1;

    if (g_log_filter.show_kin)
        LOG_DEBUG("[KIN]: Cinemática => Pasos totales: %u, Freq: %.1f Hz\n",
                  total_microsteps, target_freq_hz);
    if (g_log_filter.show_prf)
        LOG_DEBUG("[PRF]: Perfil => Accel: %u (P1:%u P2:%u) | Crucero: %u | Decel: %u (P1:%u P2:%u)\n",
                  pasos_aceleracion, a_p1, a_p2, pasos_constantes, pasos_frenado, d_p1, d_p2);

    tmc2209_move_2part_profile_dma(motor, f_start, a_p1, f_mid_accel, a_p2,
                                   target_freq_hz, pasos_constantes,
                                   d_p1, f_mid_decel, d_p2, f_end);
}

// ---------------------------------------------------------------------------
// FSM state name lookup — used by serial_consumer for human-readable logging
// ---------------------------------------------------------------------------

const char *get_state_name(Core1State_t state) {
    switch (state) {
    case ST_UNHOMED:             return "ST_UNHOMED";
    case ST_HOMING:              return "ST_HOMING";
    case ST_RELEASING_LSW_START: return "ST_RELEASING_LSW_START";
    case ST_READY_AT_HOME:       return "ST_READY_AT_HOME";
    case ST_SEARCHING_SYRINGE:   return "ST_SEARCHING_SYRINGE";
    case ST_SYRINGE_ENGAGED:     return "ST_SYRINGE_ENGAGED";
    case ST_DISPENSING:          return "ST_DISPENSING";
    case ST_DISPENSE_COMPLETED:  return "ST_DISPENSE_COMPLETED";
    case ST_SET_NEW_DISPENSE:    return "ST_SET_NEW_DISPENSE";
    case ST_SEARCHING_EOT:       return "ST_SEARCHING_EOT";
    case ST_RELEASING_LSW_END:   return "ST_RELEASING_LSW_END";
    case ST_END_OF_TRAVEL:       return "ST_END_OF_TRAVEL";
    case ST_FAULT:               return "ST_FAULT";
    case ST_OCCLUSION_STOPPING:  return "ST_OCCLUSION_STOPPING";
    case ST_OCCLUSION_RELEASE:   return "ST_OCCLUSION_RELEASE";
    case ST_OCCLUSION_PAUSED:    return "ST_OCCLUSION_PAUSED";
    case ST_MANUAL_OVERRIDE:     return "ST_MANUAL_OVERRIDE";
    case ST_CALIB_SEEK_START:    return "ST_CALIB_SEEK_START";
    case ST_CALIB_SEEK_END:      return "ST_CALIB_SEEK_END";
    case ST_BRAKING_LSW_START:   return "ST_BRAKING_LSW_START";
    case ST_BRAKING_LSW_END:     return "ST_BRAKING_LSW_END";
    default:                     return "UNKNOWN_STATE";
    }
}
