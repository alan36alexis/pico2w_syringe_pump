#include "closed_loop.h"
#include "crosscore_logger.h"
#include <math.h>
#include "pico/stdlib.h"

void closed_loop_init(ClosedLoopState_t *state) {
    state->waiting_for_correction = false;
    state->expected_target_velocity_ums = 0.0f;
    state->closed_loop_target_um = 0.0f;
    state->start_encoder_count_cl = 0;
    state->correction_attempts = 0;
    state->move_start_time_us = 0;
}

void closed_loop_init_move(ClosedLoopState_t *state, float target_um, float velocity_ums, int32_t current_count) {
    state->expected_target_velocity_ums = velocity_ums;
    state->closed_loop_target_um = target_um;
    state->start_encoder_count_cl = current_count;
    state->waiting_for_correction = true;
    state->move_start_time_us = time_us_32();
    state->correction_attempts = 0;
}

void closed_loop_check_speed(ClosedLoopState_t *state, float pps_actual, bool use_quadrature) {
    if (!state->waiting_for_correction || state->expected_target_velocity_ums <= 0.0f) {
        return;
    }

    uint32_t elapsed_us = time_us_32() - state->move_start_time_us;
    if (elapsed_us > CRUISE_CHECK_DELAY_US) {
        float speed_ums = fabsf(pps_actual) * calc_um_per_pulse(use_quadrature);

        if (fabsf(speed_ums - state->expected_target_velocity_ums) > (state->expected_target_velocity_ums * SPEED_TOLERANCE_PCT / 100.0f)) {
            logger_send_speed_warning(state->expected_target_velocity_ums, speed_ums);
        }
    }
}

/**
 * @brief Calcula la correccion necesaria para alcanzar el target
 * @param state Estado del closed loop
 * @param current_count Conteo actual del encoder
 * @param use_quadrature Si se usa quadrature
 * @param missing_um_out Puntero para devolver la correccion
 * @return true si se necesita correccion, false en caso contrario
 */
bool closed_loop_calculate_correction(ClosedLoopState_t *state, int32_t current_count, bool use_quadrature, float *missing_um_out) {
    if (!state->waiting_for_correction) {
        return false;
    }

    int32_t delta_counts = current_count - state->start_encoder_count_cl;
    float actual_um_moved = (float)delta_counts * calc_um_per_pulse(use_quadrature);

    float error_um = state->closed_loop_target_um - actual_um_moved;
    float missing_um = 0.0f;

    // Solo corregir si faltan pasos en la direccion del target (no revertir overshoots)
    if (state->closed_loop_target_um > 0.0f && error_um > CORRECTION_DEADBAND_UM) {
        missing_um = error_um;
    } else if (state->closed_loop_target_um < 0.0f && error_um < -CORRECTION_DEADBAND_UM) {
        missing_um = error_um;
    }

    if (fabsf(missing_um) > CORRECTION_DEADBAND_UM && state->correction_attempts < MAX_CORRECTION_ATTEMPTS) {
        state->correction_attempts++;
        *missing_um_out = missing_um;
        return true;
    }

    state->waiting_for_correction = false;
    return false;
}
