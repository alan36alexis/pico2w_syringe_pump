#ifndef CLOSED_LOOP_H
#define CLOSED_LOOP_H

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"

typedef struct {
    bool waiting_for_correction;
    float expected_target_velocity_ums;
    float closed_loop_target_um;
    int32_t start_encoder_count_cl;
    uint8_t correction_attempts;
    uint32_t move_start_time_us;
} ClosedLoopState_t;

/**
 * @brief Initializes the closed loop state.
 */
void closed_loop_init(ClosedLoopState_t *state);

/**
 * @brief Prepares state for a new linear movement.
 * 
 * @param state Pointer to state structure
 * @param target_um Target distance in micrometers (can be negative)
 * @param velocity_ums Target velocity in micrometers per second
 * @param current_count Current encoder count at start of move
 */
void closed_loop_init_move(ClosedLoopState_t *state, float target_um, float velocity_ums, int32_t current_count);

/**
 * @brief Monitors speed during movement and sends warnings if deviation is too high.
 * 
 * @param state Pointer to state structure
 * @param pps_actual Current pulses per second from encoder
 * @param use_quadrature Whether encoder is in quadrature mode
 */
void closed_loop_check_speed(ClosedLoopState_t *state, float pps_actual, bool use_quadrature);

/**
 * @brief Calculates the missing displacement and decides if a correction is needed.
 * 
 * @param state Pointer to state structure
 * @param current_count Current encoder count at end of move
 * @param use_quadrature Whether encoder is in quadrature mode
 * @param missing_um_out Output pointer for calculated missing distance
 * @return true if a correction move should be started, false otherwise
 */
bool closed_loop_calculate_correction(ClosedLoopState_t *state, int32_t current_count, bool use_quadrature, float *missing_um_out);

#endif // CLOSED_LOOP_H
