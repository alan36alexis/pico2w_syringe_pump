#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdbool.h>

// --- System Flags ---
#define ENABLE_ENCODER true
#define USE_QUADRATURE_ENCODER true

// --- Hardware Kinematics ---
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS_VAL 16
#define MOTOR_MICROSTEPS TMC2209_MICROSTEPS_16
#define REAL_GEARBOX_RATIO 26.85124f  // From motor datasheet; can be refined via encoder calibration
#define LEAD_SCREW_PITCH_UM 2000.0f
#define ENCODER_LINES_PER_REV 892     // 223 lines/rev * 4 (quadrature)

// --- Semi-Closed Loop Settings ---
#define SPEED_TOLERANCE_PCT 15.0f
#define CORRECTION_DEADBAND_UM 10.0f
#define MAX_CORRECTION_ATTEMPTS 1
#define CRUISE_CHECK_DELAY_US 500000  // 500ms after move start before checking speed (skips accel phase)

// --- Calibration Settings ---
#define CALIBRATION_MOVE_SPEED 200.0f
#define CALIBRATION_SEEK_SPEED 100.0f
#define MAX_TRAVEL_ENCODER_COUNT 150000  // Fallback; overridden by stored calibration

// --- FSM Velocities ---
#define FSM_HOME_VELOCITY_UMS       1500.0f
#define FSM_SEARCH_VELOCITY_UMS     1200.0f

// --- Motor Drive Profile Thresholds ---
#define MOTOR_THRESH_LOW_UMS        350.0f   // < threshold: 16x microstep, StealthChop, 0.5A
#define MOTOR_THRESH_MID_UMS        550.0f   // <= threshold: 16x microstep, SpreadCycle, 1.0A
#define MOTOR_THRESH_HIGH_UMS       800.0f   // <= threshold: 8x microstep, SpreadCycle, 1.5A
#define MOTOR_CURRENT_LOW_A         0.5f
#define MOTOR_CURRENT_MID_A         1.0f
#define MOTOR_CURRENT_HIGH_A        1.5f

// --- Trapezoidal Profile Fractions ---
#define PROFILE_ACCEL_FRACTION      0.01f    // 1% of steps for acceleration
#define PROFILE_CRUISE_FRACTION     0.80f    // 80% at cruise speed
#define PROFILE_RAMP_MIN_FRACTION   0.15f    // Minimum fraction for trapezoidal vs triangular

// --- Occlusion Thresholds (mmHg) ---
#define OCC_THRESHOLD_L0_MMHG       225.0f
#define OCC_THRESHOLD_L1_MMHG       475.0f
#define OCC_THRESHOLD_L2_MMHG       725.0f
#define OCC_THRESHOLD_L3_MMHG       975.0f

// --- Clinical API Defaults ---
#define PURGE_FLOW_RATE_MLH         1000.0f
#define PURGE_VOLUME_ML             1.0f
#define KVO_FLOW_RATE_MLH           1.0f

// --- Hardware Init Constants ---
#define TMC2209_UART_BAUD           57600
#define SENSOR_INIT_DELAY_MS        200

// --- FSM Timeouts ---
#define LSW_WAIT_TIMEOUT_MS         5000
#define VIRTUAL_LSW_HYSTERESIS_COUNTS 5

// --- Kinematics Helper ---
static inline float calc_um_per_pulse(bool use_quadrature) {
    float lines = use_quadrature ? (ENCODER_LINES_PER_REV * 4.0f) : (float)ENCODER_LINES_PER_REV;
    return (LEAD_SCREW_PITCH_UM * REAL_GEARBOX_RATIO) / lines;
}

#endif // SYSTEM_CONFIG_H
