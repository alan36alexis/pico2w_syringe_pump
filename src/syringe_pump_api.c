#include "syringe_pump_api.h"
#include "crosscore_cmd.h"
#include <math.h>
#include <stdio.h>

#define PI 3.1415926535f

static PumpContext_t ctx;
static SyringeProfile_t current_syringe;
static bool syringe_selected = false;
static float syringe_area_mm2 = 0.0f;

// --- Internal Math & Kinematics ---

/**
 * Translates clinical volume (mL) to linear displacement (um).
 */
static float ml_to_um(float volume_ml) {
    if (syringe_area_mm2 <= 0.0f) return 0.0f;
    // Volume in mm^3
    float volume_mm3 = volume_ml * 1000.0f;
    // Length in mm
    float length_mm = volume_mm3 / syringe_area_mm2;
    // Length in um
    return length_mm * 1000.0f;
}

/**
 * Translates clinical flow rate (mL/h) to linear speed (um/s).
 */
static float ml_h_to_um_s(float rate_ml_h) {
    // rate in mL per second
    float rate_ml_s = rate_ml_h / 3600.0f;
    return ml_to_um(rate_ml_s);
}

// --- API Implementation ---

void Pump_Init(void) {
    ctx.target_volume_ml = 0.0f;
    ctx.infused_volume_ml = 0.0f;
    ctx.current_rate_ml_h = 0.0f;
    ctx.current_pressure_mmhg = 0.0f;
    ctx.occlusion_threshold_mmhg = 975.0f; // Default highest safe limit
    ctx.elapsed_time_s = 0.0f;
    ctx.state = PUMP_STATE_STOPPED;
    
    ctx.alarms.occlusion = false;
    ctx.alarms.near_end_of_infusion = false;
    ctx.alarms.end_of_infusion = false;
    ctx.alarms.bubble_in_line = false;
    ctx.alarms.syringe_empty = false;
    ctx.alarms.system_error = false;
    
    syringe_selected = false;
}

bool Pump_SelectSyringe(float internal_diameter_mm, float max_capacity_ml) {
    if (internal_diameter_mm <= 0.0f || max_capacity_ml <= 0.0f) {
        return false;
    }
    current_syringe.internal_diameter_mm = internal_diameter_mm;
    current_syringe.max_capacity_ml = max_capacity_ml;
    
    float radius_mm = internal_diameter_mm / 2.0f;
    syringe_area_mm2 = PI * radius_mm * radius_mm;
    
    syringe_selected = true;
    return true;
}

void Pump_ResetSyringe(void) {
    if (ctx.state != PUMP_STATE_STOPPED) {
        Pump_Stop();
    }
    
    syringe_selected = false;
    current_syringe.internal_diameter_mm = 0.0f;
    current_syringe.max_capacity_ml = 0.0f;
    syringe_area_mm2 = 0.0f;
    
    ctx.target_volume_ml = 0.0f;
    ctx.infused_volume_ml = 0.0f;
    ctx.current_rate_ml_h = 0.0f;
    ctx.elapsed_time_s = 0.0f;
    ctx.state = PUMP_STATE_STOPPED;
    
    ctx.alarms.near_end_of_infusion = false;
    ctx.alarms.end_of_infusion = false;
    ctx.alarms.syringe_empty = false;
}

bool Pump_Mode_Continuous(float rate_ml_h) {
    if (!syringe_selected) return false;
    // IEC bounds validation (e.g., minimum 0.01 mL/h, maximum 2000 mL/h)
    if (rate_ml_h < 0.01f || rate_ml_h > 2000.0f) return false;
    
    float velocity_ums = ml_h_to_um_s(rate_ml_h);
    float remaining_ml = current_syringe.max_capacity_ml - ctx.infused_volume_ml;
    
    if (remaining_ml <= 0.0f) return false;
    float target_um = ml_to_um(remaining_ml);
    
    ctx.target_volume_ml = current_syringe.max_capacity_ml; // Infuse until empty
    ctx.current_rate_ml_h = rate_ml_h;
    ctx.state = PUMP_STATE_INFUSING_CONTINUOUS;
    
    // Command Core 1 to execute movement
    return cmd_send_move_linear_um(target_um, velocity_ums);
}

bool Pump_Mode_Bolus(float bolus_volume_ml, float bolus_rate_ml_h) {
    if (!syringe_selected) return false;
    if (bolus_volume_ml <= 0.0f || bolus_rate_ml_h <= 0.0f) return false;
    
    float remaining_in_syringe = current_syringe.max_capacity_ml - ctx.infused_volume_ml;
    if (bolus_volume_ml > remaining_in_syringe) return false; // Cannot bolus more than available

    float velocity_ums = ml_h_to_um_s(bolus_rate_ml_h);
    float target_um = ml_to_um(bolus_volume_ml);
    
    // In bolus mode, the target volume is just the bolus size added to current infused
    ctx.target_volume_ml = ctx.infused_volume_ml + bolus_volume_ml;
    ctx.current_rate_ml_h = bolus_rate_ml_h;
    ctx.state = PUMP_STATE_INFUSING_BOLUS;
    
    return cmd_send_move_linear_um(target_um, velocity_ums);
}

bool Pump_Mode_Purge(void) {
    if (!syringe_selected) return false;
    
    // Purging is moving the syringe fast (e.g. 1000 mL/h) for a short distance
    float purge_rate = 1000.0f; // 1000 mL/h
    float purge_volume = 1.0f;  // 1 mL safely
    
    float velocity_ums = ml_h_to_um_s(purge_rate);
    float target_um = ml_to_um(purge_volume);
    
    ctx.state = PUMP_STATE_PURGING;
    ctx.current_rate_ml_h = purge_rate;
    
    return cmd_send_move_linear_um(target_um, velocity_ums);
}

bool Pump_Mode_KVO(void) {
    if (!syringe_selected) return false;
    
    // KVO (Keep Vein Open) is usually a very small rate (e.g., 1.0 mL/h minimum)
    float kvo_rate = 1.0f; 
    
    float remaining_in_syringe = current_syringe.max_capacity_ml - ctx.infused_volume_ml;
    if (remaining_in_syringe <= 0.0f) return false; // Syringe is completely empty
    
    float velocity_ums = ml_h_to_um_s(kvo_rate);
    float target_um = ml_to_um(remaining_in_syringe);
    
    ctx.state = PUMP_STATE_KVO;
    ctx.current_rate_ml_h = kvo_rate;
    
    return cmd_send_move_linear_um(target_um, velocity_ums);
}

bool Pump_Stop(void) {
    ctx.state = PUMP_STATE_STOPPED;
    ctx.current_rate_ml_h = 0.0f;
    return cmd_send_stop_motor();
}

// --- Security and Monitoring ---

void Pump_SetOcclusionThreshold(uint8_t level) {
    // 4 levels mapping 225 to 975 mmHg
    switch(level) {
        case 0: ctx.occlusion_threshold_mmhg = 225.0f; break;
        case 1: ctx.occlusion_threshold_mmhg = 475.0f; break;
        case 2: ctx.occlusion_threshold_mmhg = 725.0f; break;
        case 3: 
        default: ctx.occlusion_threshold_mmhg = 975.0f; break;
    }
}

void Pump_CheckAlarms(void) {
    // 1. Occlusion validation
    if (ctx.current_pressure_mmhg >= ctx.occlusion_threshold_mmhg) {
        if (!ctx.alarms.occlusion) {
            ctx.alarms.occlusion = true;
            Pump_Stop(); // Critical safety: stop immediately
            ctx.state = PUMP_STATE_ALARM;
        }
    } else {
        ctx.alarms.occlusion = false;
    }
    
    // 2. Near end of infusion (Last 10%)
    if (ctx.target_volume_ml > 0.0f) {
        float remaining_ml = ctx.target_volume_ml - ctx.infused_volume_ml;
        if (remaining_ml > 0.0f && remaining_ml <= (ctx.target_volume_ml * 0.10f)) {
            ctx.alarms.near_end_of_infusion = true;
        } else {
            ctx.alarms.near_end_of_infusion = false;
        }
        
        // 3. End of infusion
        if (ctx.infused_volume_ml >= ctx.target_volume_ml) {
            if (!ctx.alarms.end_of_infusion) {
                ctx.alarms.end_of_infusion = true;
                // IEC 60601-2-24: Drop to KVO automatically when infusion ends
                if (ctx.state != PUMP_STATE_KVO && ctx.state != PUMP_STATE_ALARM) {
                    Pump_Mode_KVO();
                }
            }
        } else {
            ctx.alarms.end_of_infusion = false;
        }
    }
    
    // 4. Syringe exactly empty safety check
    if (ctx.infused_volume_ml >= current_syringe.max_capacity_ml && syringe_selected) {
        ctx.alarms.syringe_empty = true;
        Pump_Stop(); // Must stop if physical limit reached
        ctx.state = PUMP_STATE_ALARM;
    }
}

void Pump_UpdatePressure(float pressure_mmhg) {
    ctx.current_pressure_mmhg = pressure_mmhg;
    Pump_CheckAlarms();
}

void Pump_UpdateInfusedVolume(float delta_ml) {
    ctx.infused_volume_ml += delta_ml;
    Pump_CheckAlarms();
}

void Pump_Tick(uint32_t delta_ms) {
    if (delta_ms == 0) return;
    
    // Only accumulate volume and time if we are actually pumping
    if (ctx.state == PUMP_STATE_INFUSING_CONTINUOUS ||
        ctx.state == PUMP_STATE_INFUSING_BOLUS ||
        ctx.state == PUMP_STATE_PURGING ||
        ctx.state == PUMP_STATE_KVO) {
        
        float delta_s = (float)delta_ms / 1000.0f;
        ctx.elapsed_time_s += delta_s;
        
        // current_rate_ml_h is in mL/h. Convert to mL/s.
        float rate_ml_s = ctx.current_rate_ml_h / 3600.0f;
        float delta_ml = rate_ml_s * delta_s;
        
        Pump_UpdateInfusedVolume(delta_ml);
    }
}

// --- Telemetry (IoT) ---

void Pump_GetTelemetryJSON(char* json_buffer, size_t max_len) {
    float remaining_ml = ctx.target_volume_ml - ctx.infused_volume_ml;
    if (remaining_ml < 0.0f) remaining_ml = 0.0f;
    
    float time_remaining_h = 0.0f;
    if (ctx.current_rate_ml_h > 0.0f && 
       (ctx.state == PUMP_STATE_INFUSING_CONTINUOUS || 
        ctx.state == PUMP_STATE_INFUSING_BOLUS || 
        ctx.state == PUMP_STATE_KVO)) {
        time_remaining_h = remaining_ml / ctx.current_rate_ml_h;
    }
    
    snprintf(json_buffer, max_len,
        "{\"vol_inf\":%.2f,"
        "\"vol_tgt\":%.2f,"
        "\"rate\":%.2f,"
        "\"t_ela_s\":%.1f,"
        "\"t_rem_h\":%.2f,"
        "\"pres\":%.1f,"
        "\"st\":%d,"
        "\"alm\":{\"occ\":%d,\"near\":%d,\"end\":%d,\"bub\":%d,\"emp\":%d,\"err\":%d}}",
        ctx.infused_volume_ml, 
        ctx.target_volume_ml, 
        ctx.current_rate_ml_h, 
        ctx.elapsed_time_s,
        time_remaining_h, 
        ctx.current_pressure_mmhg,
        (int)ctx.state,
        (int)ctx.alarms.occlusion, 
        (int)ctx.alarms.near_end_of_infusion, 
        (int)ctx.alarms.end_of_infusion,
        (int)ctx.alarms.bubble_in_line, 
        (int)ctx.alarms.syringe_empty, 
        (int)ctx.alarms.system_error);
}

const PumpContext_t* Pump_GetContext(void) {
    return &ctx;
}
