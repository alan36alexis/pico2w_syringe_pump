#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdbool.h>

// --- System Flags ---
#define ENABLE_ENCODER true
#define USE_QUADRATURE_ENCODER true
#define ENABLE_SERIAL_ANSI_COLOR                                               \
  1 // 1 = Colores ANSI en terminal serie, 0 = Texto plano estándar

// --- Serial Terminal Throttling & UI ---
#define ENABLE_SERIAL_PROGRESS_BAR                                             \
  1 // 1 = Barra visual con \r en terminal serie, 0 = Log simple prg=...
#define SERIAL_THROTTLE_PROGRESS_MS                                            \
  300u // Intervalo mínimo de log de progreso en terminal (~3 Hz)
#define SERIAL_THROTTLE_SPEED_MS                                               \
  600u // Intervalo mínimo de log de velocidad en terminal (~1.5 Hz)
#define SERIAL_THROTTLE_ENCODER_MS                                             \
  600u // Intervalo mínimo de log de encoder en terminal (~1.5 Hz)
#define SERIAL_THROTTLE_PRESSURE_MS                                            \
  500u // Intervalo mínimo de log de presión normal en terminal (2 Hz)

// --- Hardware Kinematics ---
#define MOTOR_STEPS_PER_REV 200
#define MOTOR_MICROSTEPS_VAL 16
#define MOTOR_MICROSTEPS TMC2209_MICROSTEPS_16
#define REAL_GEARBOX_RATIO                                                     \
  26.85124f // From motor datasheet; can be refined via encoder calibration
#define LEAD_SCREW_PITCH_UM 2000.0f
#define ENCODER_LINES_PER_REV                                                  \
  892 // physical encoder lines/rev; ×4 quadrature = 3568 counts/rev (on output
      // shaft, post-gearbox)

// --- Semi-Closed Loop Settings ---
#define SPEED_TOLERANCE_PCT 15.0f
#define CORRECTION_DEADBAND_UM 10.0f
#define MAX_CORRECTION_ATTEMPTS 1
#define CRUISE_CHECK_DELAY_US                                                  \
  500000 // 500ms after move start before checking speed (skips accel phase)

// --- Calibration Settings ---
#define CALIBRATION_MOVE_SPEED 200.0f
#define CALIBRATION_SEEK_SPEED 100.0f
#define MAX_TRAVEL_ENCODER_COUNT                                               \
  150000 // Fallback; overridden by stored calibration

// --- FSM Velocities ---
#define FSM_HOME_VELOCITY_UMS 1500.0f
#define FSM_SEARCH_VELOCITY_UMS 1200.0f
#define POS_JERINGA_TEST_UM 90000.0f

// --- Motor Drive Profile Thresholds ---
#define MOTOR_THRESH_LOW_UMS                                                   \
  350.0f // < threshold: 16x microstep, StealthChop, 0.5A
#define MOTOR_THRESH_MID_UMS                                                   \
  550.0f // <= threshold: 16x microstep, SpreadCycle, 1.0A
#define MOTOR_THRESH_HIGH_UMS                                                  \
  800.0f // <= threshold: 8x microstep, SpreadCycle, 1.5A
#define MOTOR_CURRENT_LOW_A 0.5f
#define MOTOR_CURRENT_MID_A 1.0f
#define MOTOR_CURRENT_HIGH_A 1.5f

// --- Trapezoidal Profile ---
#define PROFILE_ACCEL_UMS2                                                     \
  10000.0f // Aceleración constante en µm/s² (independiente de distancia y
           // velocidad)

// --- Occlusion Thresholds (mmHg) ---
#define OCC_THRESHOLD_L0_MMHG 225.0f
#define OCC_THRESHOLD_L1_MMHG 475.0f
#define OCC_THRESHOLD_L2_MMHG 725.0f
#define OCC_THRESHOLD_L3_MMHG 975.0f

// --- Occlusion Release & Anti-Bolus Settings ---
#define OCCLUSION_RELEASE_MAX_UM                                               \
  5000.0f // Recorrido máximo de retroceso al liberar oclusión (5 mm)
#define OCCLUSION_RELEASE_SPEED_UMS                                            \
  200.0f // Velocidad de retroceso durante la liberación (200 um/s)
#define ANTIBOLUS_AUTO_ENABLED_DEFAULT                                         \
  1 // 1 = Anti-bolus automático habilitado por defecto, 0 = Manual

// --- ADC Voltage Threshold Defaults (Volts) ---
#define ADC_THRESH_SYRINGE_REACHED_V_DEFAULT                                   \
  0.3f // Umbral detección inicio de jeringa (contacto)
#define ADC_THRESH_OCCLUSION_V_DEFAULT 0.5f // Umbral detección oclusión
#define ADC_THRESH_OCC_RELEASE_V_DEFAULT                                       \
  0.3f // Umbral alivio de presión tras release

// --- Clinical API Defaults ---
#define PURGE_FLOW_RATE_MLH 1000.0f
#define PURGE_VOLUME_ML 1.0f
#define KVO_FLOW_RATE_MLH 1.0f

// --- Hardware Pins ---
#define MOTOR_STEP_PIN 3
#define MOTOR_DIR_PIN 2
#define MOTOR_ENA_PIN 8
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define LIMIT_SW_START_PIN 10
#define LIMIT_SW_END_PIN 11
#define ADC_PIN 26
#define ADC_CHANNEL 0 // GPIO28 = ADC channel 2
#define ENCODER_PIN_A 6
#define ENCODER_PIN_B 7
#define USE_UART_MODE true

// --- Shared I2C0 Bus (PCAL9555 & Future RTC DS3231) ---
#define I2C0_SDA_PIN 12
#define I2C0_SCL_PIN 13
#define I2C0_BAUDRATE_HZ 400000 // Fast-mode 400 kHz
#define I2C0_BUS_INST i2c0
#define I2C0_TIMEOUT_US 25000 // 25 ms timeout para transferencias I2C

// --- PCAL9555 GPIO Expander ---
#define PCAL9555_I2C_ADDR_DEFAULT 0x20
#define PCAL9555_POLL_INTERVAL_MS 50 // Polling a 20 Hz en Core 0
#define PCAL9555_DEBOUNCE_COUNT 2 // 2 muestras consecutivas estables (100 ms)

// Canales individuales del PCAL9555 (0 a 15)
#define EXP_PIN_SW_PLAY_STOP 0    // P0_0 (IN, Pulsador Play/Stop)
#define EXP_PIN_SW_BOLUS 1        // P0_1 (IN, Pulsador Bolo)
#define EXP_PIN_SW_DETECT_H 2     // P0_2 (IN, Deteccion de jeringa)
#define EXP_PIN_LED_ALARM 3       // P0_3 (OUT, LED Alarma)
#define EXP_PIN_LED_BATT_STATUS 4 // P0_4 (OUT, LED Estado Bateria)
#define EXP_PIN_P0_5_AUX 5        // P0_5 (NC, Auxiliar no conectado)
#define EXP_PIN_P0_6_AUX 6        // P0_6 (NC, Auxiliar no conectado)
#define EXP_PIN_P0_7_AUX 7        // P0_7 (NC, Auxiliar no conectado)
#define EXP_PIN_OFF_CNTL 8        // P1_0 (OUT, Control de auto-apagado)
#define EXP_PIN_CHRG_STATUS 9     // P1_1 (IN, Estado de carga)
#define EXP_PIN_BATT_INSTALL 10   // P1_2 (IN, Deteccion bateria)
#define EXP_PIN_P1_3_NC 11        // P1_3 (NC, No conectado)
#define EXP_PIN_P1_4_NC 12        // P1_4 (NC, No conectado)
#define EXP_PIN_P1_5_NC 13        // P1_5 (NC, No conectado)
#define EXP_PIN_BUZZER_CNTL 14    // P1_6 (OUT, Buzzer acustico)
#define EXP_PIN_FRC_SENS_CALIB 15 // P1_7 (OUT, Calibracion sensor fuerza)

// Mascaras de configuracion inicial de hardware:
// 1 = Input (P0: bits 0,1,2,5,6,7 = 0xE7; P1: bits 1,2,3,4,5 = 0x3E -> 0x3EE7)
#define PCAL9555_DIR_MASK_DEFAULT 0x3EE7u
#define PCAL9555_PULL_EN_DEFAULT                                               \
  0x3EE7u // Pull-up activo en entradas y NCs anti-flotacion
#define PCAL9555_PULL_SEL_DEFAULT 0xFFFFu // 1 = Pull-up interno 100k
#define PCAL9555_OUTPUTS_DEFAULT 0x0000u  // Todas las salidas apagadas en boot

// --- Hardware Init Constants ---
#define TMC2209_UART_BAUD 57600
#define SENSOR_INIT_DELAY_MS 200

// --- FSM Fault-detection layers (replaces the commented-out
// LSW_WAIT_TIMEOUT_MS) ---
//
// Layer 2 — Encoder stall window:
//   At the minimum operating speed (300 µm/s ≈ 0.53 encoder counts/s in
//   quadrature mode), a true stall produces Δcount == 0 within ~200 ms.
//   300 ms leaves a comfortable margin above measurement noise.
//   Only active while a DMA profile is running (motor_is_moving == true).
#define ENCODER_STALL_WINDOW_MS 300u

// Layer 3 — Derived deadline (per-move, not fixed):
//   deadline_ms = now_ms + DEADLINE_K * t_nominal_ms + DEADLINE_FLOOR_MS
//   where t_nominal_ms = |dist_um| / vel_ums * 1000.
//   DEADLINE_K = 3  ← allows 3× the expected duration before faulting.
//   DEADLINE_FLOOR_MS = 500 ← minimum timeout regardless of move duration.
//   Only covers the pathological case of a step-generator that stops pulsing
//   without emitting iEV_TARGET_REACHED or moving the encoder.
#define DEADLINE_K 3u
#define DEADLINE_FLOOR_MS 500u

// Deadlines fijos para frenado/liberación de LSW (no hay par
// distancia/velocidad del cual derivarlos). Frenar desde <=1500 um/s a
// PROFILE_ACCEL_UMS2 toma <200 ms; la liberación a 2000 Hz (~50 um/s) puede
// tardar decenas de segundos en salir de la histéresis del switch.
#define BRAKE_DEADLINE_MS 3000u
#define LSW_RELEASE_DEADLINE_MS 60000u

// Dead-time entre el freno y el movimiento inverso de release sobre un LSW.
// Evita el golpe mecánico a la caja reductora al invertir sin pausa.
// El lanzamiento diferido lo hace fsm_service_release_deadtime().
#define LSW_REVERSAL_DEAD_TIME_MS 200u

// LSW_WAIT_TIMEOUT_MS kept for reference; do NOT reuse as a fixed timeout.
// The single-value approach could not cover both homing (~350 s @ 300 µm/s)
// and braking (~2 s) simultaneously.  Use the three-layer scheme above.
#define LSW_WAIT_TIMEOUT_MS_DEPRECATED 5000

// --- Kinematics Helper ---
static inline float calc_um_per_pulse(bool use_quadrature) {
  // Encoder on output shaft (post-gearbox): gearbox ratio does not apply.
  float counts = use_quadrature ? (ENCODER_LINES_PER_REV * 4.0f)
                                : (float)ENCODER_LINES_PER_REV;
  return LEAD_SCREW_PITCH_UM / counts;
  // quadrature:     2000 / (892 * 4) = 0.5607 um/count
  // non-quadrature: 2000 / 892       = 2.242  um/count
}

#endif // SYSTEM_CONFIG_H
