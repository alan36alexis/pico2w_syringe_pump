#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// --- System Flags ---
#define ENABLE_ENCODER true
#define USE_QUADRATURE_ENCODER true

// --- Hardware Kinematics ---
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS_VAL 16
#define MOTOR_MICROSTEPS TMC2209_MICROSTEPS_16
#define REAL_GEARBOX_RATIO 26.85124f
#define LEAD_SCREW_PITCH_UM 2000.0f
#define ENCODER_LINES_PER_REV 892

// --- Semi-Closed Loop Settings ---
#define SPEED_TOLERANCE_PCT 15.0f
#define CORRECTION_DEADBAND_UM 10.0f
#define MAX_CORRECTION_ATTEMPTS 1
#define CRUISE_CHECK_DELAY_US 500000 // 500ms ignoring acceleration

// --- Calibration Settings ---
#define CALIBRATION_MOVE_SPEED 200.0f
#define CALIBRATION_SEEK_SPEED 100.0f

#define MAX_TRAVEL_ENCODER_COUNT                                               \
  150000 // TODO: Implementar la calibracion y guardado en flash

#endif // SYSTEM_CONFIG_H
