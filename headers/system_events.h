#pragma once

#include <stdint.h>
#include "pico/util/queue.h"

// ---------------------------------------------------------------------------
// Forward declaration of the crosscore queue (defined in system_queues.c).
// Intentionally declared here (not in system_queues.h) so that Core 1
// (baremetal, no FreeRTOS) can include only this header to use CORE1_EMIT.
// ---------------------------------------------------------------------------
extern queue_t g_crosscore_event_q;

// ---------------------------------------------------------------------------
// Event IDs
// typedef uint16_t instead of "enum : uint16_t" (C23 feature not in C11+GCC14)
// ---------------------------------------------------------------------------
typedef uint16_t SystemEventID_t;

// TMC2209 Driver — internal motor registers [0x0100–0x01FF]
#define EV_TMC_DRV_STATUS    ((SystemEventID_t)0x0100)  // DtoTmcStatus_t
#define EV_TMC_STALL         ((SystemEventID_t)0x0101)  // DtoTmcStatus_t
#define EV_TMC_OVERTEMP      ((SystemEventID_t)0x0102)  // DtoTmcStatus_t

// Actuators and process sensors [0x0200–0x02FF]
#define EV_ACT_PRESSURE      ((SystemEventID_t)0x0200)  // DtoForce_t: psi, mmhg
#define EV_ACT_PRESSURE_OCC  ((SystemEventID_t)0x0201)  // DtoForce_t: occlusion threshold exceeded
#define EV_ACT_LSW_END       ((SystemEventID_t)0x0202)  // payload.param = 0
#define EV_ACT_SYRINGE_DET   ((SystemEventID_t)0x0203)  // payload.param: 1=engaged, 0=removed

// Motor and kinematics [0x0300–0x03FF]
#define EV_MOT_ENCODER       ((SystemEventID_t)0x0300)  // DtoMotion_t: count, position_mm
#define EV_MOT_SPEED         ((SystemEventID_t)0x0301)  // DtoMotion_t: speed fields
#define EV_MOT_PROGRESS      ((SystemEventID_t)0x0302)  // DtoMotion_t: progress_pct
#define EV_MOT_CORRECTION    ((SystemEventID_t)0x0303)  // DtoMotion_t: correction_um
#define EV_MOT_SPEED_WARN    ((SystemEventID_t)0x0304)  // DtoMotion_t: expected vs actual

// Network and connectivity [0x0400–0x04FF]
#define EV_NET_WIFI_CONN       ((SystemEventID_t)0x0400)  // DtoWifi_t: rssi_dbm, channel
#define EV_NET_WIFI_DISC       ((SystemEventID_t)0x0401)  // payload.param: reason code
#define EV_NET_MQTT_CONN       ((SystemEventID_t)0x0402)  // payload.param = 0
#define EV_NET_MQTT_DISC       ((SystemEventID_t)0x0403)  // payload.param = 0
#define EV_NET_MQTT_TX_DROP    ((SystemEventID_t)0x0404)  // payload.param: cumulative drops
#define EV_NET_WIFI_CONNECTING ((SystemEventID_t)0x0405)  // payload.param = 0

// Application / FSM [0x0500–0x05FF]
#define EV_APP_FSM_STATE     ((SystemEventID_t)0x0500)  // DtoFsm_t
#define EV_APP_SESSION_START ((SystemEventID_t)0x0501)  // DtoSession_t
#define EV_APP_SESSION_END   ((SystemEventID_t)0x0502)  // DtoSession_t
#define EV_APP_SESSION_UPD   ((SystemEventID_t)0x0503)  // DtoSession_t: periodic update (2s)
#define EV_APP_CMD_EXECUTED  ((SystemEventID_t)0x0504)  // DtoManualOp_t
#define EV_APP_CALIBRATION       ((SystemEventID_t)0x0505)  // DtoCalibration_t
#define EV_SYS_CALIBRATION_SAVED ((SystemEventID_t)0x0506)  // payload.param = 0

// Alarms [0x0600–0x06FF]
// IMPORTANT: these are logging notifications, NOT control signals.
// The safety action (stop motor, alert UI) already happened in Core 1 / FSM.
// These events only inform consumers (serial, MQTT, HMI) of the fact.
#define EV_ALARM_OCCLUSION   ((SystemEventID_t)0x0600)  // DtoAlarm_t
#define EV_ALARM_EOT         ((SystemEventID_t)0x0601)  // DtoAlarm_t
#define EV_ALARM_DRV_FAULT   ((SystemEventID_t)0x0602)  // DtoAlarm_t
#define EV_ALARM_BATTERY_LOW ((SystemEventID_t)0x0603)  // DtoAlarm_t
#define EV_ALARM_MAINS_LOST  ((SystemEventID_t)0x0604)  // DtoAlarm_t

// Power and supply [0x0700–0x07FF]
#define EV_PWR_BATTERY_UPD   ((SystemEventID_t)0x0700)  // DtoPower_t
#define EV_PWR_MAINS_DETECT  ((SystemEventID_t)0x0701)  // payload.param: 1=present, 0=absent

// System health [0x0800–0x08FF]
#define EV_SYS_HEAP_UPD      ((SystemEventID_t)0x0800)  // DtoSysHealth_t
#define EV_SYS_HEARTBEAT     ((SystemEventID_t)0x0801)  // payload.param: counter (Core 1)
#define EV_SYS_CLI_READY     ((SystemEventID_t)0x0802)  // payload.param = 0
#define EV_DBG_STRING        ((SystemEventID_t)0x0803)  // DtoDebugStr_t: Core 1 debug log

// Domain range markers for consumer filtering
#define EV_DOMAIN_ALARM_MIN  ((SystemEventID_t)0x0600)
#define EV_DOMAIN_ALARM_MAX  ((SystemEventID_t)0x06FF)

// Consumer-side alarm filter — the broker never uses this
#define EV_IS_ALARM(id)  ((id) >= EV_DOMAIN_ALARM_MIN && (id) <= EV_DOMAIN_ALARM_MAX)

// ---------------------------------------------------------------------------
// DTOs — Data Transfer Objects
// General invariant: no char[] fields — strings never travel through queues.
// Exception: DtoDebugStr_t carries a fixed-size string for Core 1 debug logs
// emitted via LOG_DEBUG (EV_DBG_STRING). Payload is 56 bytes to keep
// sizeof(SystemEvent_t) == 64.
// ---------------------------------------------------------------------------

typedef struct {
    float psi;
    float mmhg;
} DtoForce_t;

typedef struct {
    int32_t encoder_count;
    float   position_mm;
    float   encoder_speed_ums;
    float   target_speed_ums;
    float   progress_pct;
    float   correction_um;
} DtoMotion_t;

// Bit positions for DtoTmcStatus_t.flags (parsed from GSTAT + DRV_STATUS)
#define TMC_FLAG_OT_WARN (1u << 0)  // drv_status: OTPW  — prewarning ~120°C
#define TMC_FLAG_OT_SHUT (1u << 1)  // drv_status: OT    — shutdown activo
#define TMC_FLAG_SHORT_A (1u << 2)  // drv_status: S2GA | S2VSA
#define TMC_FLAG_SHORT_B (1u << 3)  // drv_status: S2GB | S2VSB
#define TMC_FLAG_OPEN_A  (1u << 4)  // drv_status: OLA
#define TMC_FLAG_OPEN_B  (1u << 5)  // drv_status: OLB
#define TMC_FLAG_UV_CP   (1u << 6)  // gstat: uv_cp — undervoltage charge pump
#define TMC_FLAG_DRV_ERR (1u << 7)  // gstat: drv_err — driver apagado

typedef struct {
    uint16_t stall_count;  // solo válido con ENABLE_STALLGUARD_LOG
    uint8_t  flags;        // bitmask TMC_FLAG_*
} DtoTmcStatus_t;

typedef struct {
    int8_t  rssi_dbm;
    uint8_t channel;
} DtoWifi_t;

typedef struct {
    uint16_t state_from;   // Core1State_t cast to uint16_t
    uint16_t state_to;
    uint32_t session_id;
} DtoFsm_t;

typedef struct {
    uint32_t session_id;
    float    target_volume_ml;
    float    infused_volume_ml;
    float    rate_ml_h;
    uint32_t elapsed_s;
    float    syringe_diam_mm;
} DtoSession_t;

typedef struct {
    uint32_t session_id;
    uint16_t alarm_id;     // SystemEventID_t of the triggering event
    uint8_t  severity;     // 1=LOW, 2=MEDIUM, 3=HIGH (IEC 60601-1-8)
    uint16_t fsm_state;    // FSM state at the moment of the alarm
    float    param_f;      // contextual numeric value (e.g. pressure in mmHg)
} DtoAlarm_t;

typedef struct {
    uint16_t source;       // 0=CLI, 1=MQTT, 2=TFT
    uint16_t fsm_from;
    uint16_t fsm_to;
    uint8_t  accepted;
    uint8_t  reject_code;  // 0=ok, 1=INVALID_STATE, 2=PARAM_OOB
    uint8_t  action_id;
} DtoManualOp_t;

typedef struct {
    float   battery_voltage_v;
    uint8_t battery_pct;
    uint8_t flags;         // bit 0: MAINS_PRESENT, 1: CHARGING, 2: ON_BATTERY
} DtoPower_t;

typedef struct {
    uint32_t free_heap_bytes;
    uint32_t min_ever_heap_bytes;
    uint8_t  mqtt_tx_drops;
    uint8_t  mqtt_rx_drops;
} DtoSysHealth_t;

typedef struct {
    int32_t max_encoder_count;
    float   travel_mm;
    uint8_t trigger;       // 0=AUTO, 1=CLI, 2=MQTT
    uint8_t success;
} DtoCalibration_t;

typedef struct {
    char buf[56];
} DtoDebugStr_t;

// ---------------------------------------------------------------------------
// SystemEvent_t — universal event structure
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t        timestamp_ms;
    // Core 1: always 0 — broker stamps on receive with FreeRTOS tick
    // Core 0: emitter stamps with xTaskGetTickCount() * portTICK_PERIOD_MS
    // Rationale: Core 1 is baremetal, has no FreeRTOS tick access.
    // Stamping latency (<=2ms) is irrelevant for logging.

    SystemEventID_t id;

    // NO priority field. The broker is a uniform router — it does not interpret
    // content. Consumers are responsible for prioritizing via EV_IS_ALARM() etc.

    union {
        uint32_t         param;        // for simple events without a full DTO
        DtoForce_t       force;
        DtoMotion_t      motion;
        DtoTmcStatus_t   tmc;
        DtoWifi_t        wifi;
        DtoFsm_t         fsm;
        DtoSession_t     session;
        DtoAlarm_t       alarm;
        DtoManualOp_t    manual_op;
        DtoPower_t       power;
        DtoSysHealth_t   sys_health;
        DtoCalibration_t calibration;
        DtoDebugStr_t    dbg_str;
    } payload;
} SystemEvent_t;

// RAM budget (heap_3, ~264 KB SRAM available on RP2350):
//   sizeof(SystemEvent_t) x queue capacities:
//   ~32 x (32 + 24 + 32 + 32 + 16) = 32 x 136 = 4352 bytes total
_Static_assert(sizeof(SystemEvent_t) <= 64,
    "SystemEvent_t too large — review DTOs to reduce size");

// ---------------------------------------------------------------------------
// Emission macros
// ---------------------------------------------------------------------------

// Core 1 (baremetal): timestamp = 0, broker stamps on receive.
// Depends only on Pico SDK queue_t — no FreeRTOS headers required.
// Non-blocking: drops silently if queue full (acceptable for logging).
#define CORE1_EMIT(event_id, dto_field, dto_value) do {  \
    SystemEvent_t _ev = {                                  \
        .timestamp_ms      = 0,                            \
        .id                = (event_id),                   \
        .payload.dto_field = (dto_value)                   \
    };                                                     \
    queue_try_add(&g_crosscore_event_q, &_ev);             \
} while (0)
