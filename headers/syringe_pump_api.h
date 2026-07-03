#ifndef SYRINGE_PUMP_API_H
#define SYRINGE_PUMP_API_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// --- Data Structures ---

typedef struct {
    float internal_diameter_mm;
    float max_capacity_ml;
} SyringeProfile_t;

typedef struct {
    bool occlusion;
    bool near_end_of_infusion; // Reached last 10%
    bool end_of_infusion;      // Reached 100%
    bool bubble_in_line;
    bool syringe_empty;
    bool system_error;
} PumpAlarms_t;

typedef enum {
    PUMP_STATE_STOPPED = 0,
    PUMP_STATE_INFUSING_CONTINUOUS,
    PUMP_STATE_INFUSING_BOLUS,
    PUMP_STATE_PURGING,
    PUMP_STATE_KVO,
    PUMP_STATE_PAUSED,
    PUMP_STATE_ALARM
} PumpState_t;

typedef struct {
    float target_volume_ml;
    float infused_volume_ml;
    float current_rate_ml_h;
    float current_pressure_mmhg;
    float occlusion_threshold_mmhg;
    float elapsed_time_s;
    PumpAlarms_t alarms;
    PumpState_t state;
} PumpContext_t;

typedef enum {
    MOTOR_CMD_MOVE = 0,
    MOTOR_CMD_STOP,
    MOTOR_CMD_PAUSE
} MotorCommand_t;

// --- API Functions ---

/**
 * Initializes the API structures and sets default safe values.
 */
void Pump_Init(void);

/**
 * Validates and configures a commercial syringe based on its internal diameter (1 to 60 mL).
 * Returns false if parameters are invalid.
 */
bool Pump_SelectSyringe(float internal_diameter_mm, float max_capacity_ml);

/**
 * Resets the active syringe configuration. Must be called when changing syringes.
 * This automatically stops the pump and clears volume context.
 */
void Pump_ResetSyringe(void);

// --- Operation Modes ---

/**
 * Starts continuous infusion. 
 * Validates minimum increment of 0.01 mL/h and limits (e.g. 2000 mL/h).
 */
bool Pump_Mode_Continuous(float rate_ml_h);

/**
 * Starts a bolus dose (fixed volume at a fast rate).
 */
bool Pump_Mode_Bolus(float bolus_volume_ml, float bolus_rate_ml_h);

/**
 * Starts safe system purging before connecting to patient.
 */
bool Pump_Mode_Purge(void);

/**
 * Keep Vein Open (KVO) mode to maintain line patency after infusion.
 */
bool Pump_Mode_KVO(void);

/**
 * Stops any current motor operation.
 */
bool Pump_Stop(void);

// --- Security and Monitoring (IEC 60601-2-24) ---

/**
 * Sets the occlusion pressure threshold.
 * levels map approx from 225 up to 975 mmHg in 4 steps.
 */
void Pump_SetOcclusionThreshold(uint8_t level);

/**
 * Evaluates sensor readings and sets alarm flags.
 * Includes near end of infusion (10% remaining) validation.
 */
void Pump_CheckAlarms(void);

/**
 * Updates the current system pressure (called periodically from sensor reading updates).
 */
void Pump_UpdatePressure(float pressure_mmhg);

/**
 * Updates the infused volume (based on motor step feedback or periodic integration).
 * @param delta_ml The increment of infused volume since last update.
 */
void Pump_UpdateInfusedVolume(float delta_ml);

/**
 * Feeds a time delta to the API so it can integrate the flow rate and calculate
 * the exact infused volume and elapsed time. Must be called periodically.
 * @param delta_ms Elapsed time in milliseconds since the last call.
 */
void Pump_Tick(uint32_t delta_ms);

// --- Telemetry (IoT) ---

/**
 * Generates a JSON string with the current context for Mosquitto MQTT.
 */
void Pump_GetTelemetryJSON(char* json_buffer, size_t max_len);

/**
 * Returns a read-only pointer to the global pump context.
 */
const PumpContext_t* Pump_GetContext(void);

#endif // SYRINGE_PUMP_API_H
