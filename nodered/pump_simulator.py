#!/usr/bin/env python3
"""
pump_simulator.py — Simulador de Bomba de Infusión a Jeringa (Arquitectura EDA)
=============================================================================
Replica el comportamiento del firmware pico2w_syringe_pump bajo el contrato MQTT
moderno multi-rate (EDA):
  - bj/{device_id}/telemetry/sensors  (QoS 0, 500 ms)
  - bj/{device_id}/telemetry/motion   (QoS 0, 1000 ms)
  - bj/{device_id}/telemetry/session  (QoS 0, 2000 ms)
  - bj/{device_id}/event              (QoS 1, por disparo: FSM transitions y alarmas)
  - bj/{device_id}/status             (QoS 1, retain: online/offline)
  - bj/{device_id}/cmd                (QoS 1, comandos con envelope {"cid":N,"cmd":"..."})
  - bj/{device_id}/cmd/ack            (QoS 1, ACK {"cid":N,"result":"accepted"|"rejected"})

Fuentes de verdad portadas:
  - headers/core1_main.h     → Core1State_t (21 estados FSM de Core 1)
  - src/mqtt_consumer.c      → Esquemas JSON multi-rate y eventos
  - src/cmd_dispatcher.c     → Pipeline de comandos agnóstico
  - headers/system_config.h  → Constantes clínicas y umbrales de oclusión

Uso rápido:
    python pump_simulator.py --broker localhost --id bj-001
    python pump_simulator.py --broker 192.168.1.100 --id bj-002 --diam 19.05 --cap 20.0

Comandos vía MQTT (tópico: bj/{device_id}/cmd):
    {"cid": 1, "cmd": "fsm_home,1500"}
    {"cid": 2, "cmd": "fsm_dispense,30000,500"}
    {"cid": 3, "cmd": "stop"}

Inyección de fallas / pruebas (tópico: bj/{device_id}/sim_fault):
    fault_occ          →  simula rampa de oclusión hidráulica
    fault_bubble       →  dispara alarma de burbuja
    fault_clear        →  limpia alarmas y restablece presión
    infuse,<rate>,<vol>→  inicia infusión continua (mL/h, mL)
    bolus,<vol>,<rate> →  dispara bolo (mL, mL/h)
    purge              →  purga rápida
"""

import argparse
import json
import math
import random
import threading
import time
import logging
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Tuple

import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("BombaSim")

# ---------------------------------------------------------------------------
# Constantes clínicas y cinemáticas
# ---------------------------------------------------------------------------
PI = 3.1415926535

# Umbrales de oclusión (mmHg)
OCC_THRESHOLDS = [225.0, 475.0, 725.0, 975.0]

PURGE_FLOW_RATE_MLH = 1000.0
PURGE_VOLUME_ML = 1.0
KVO_FLOW_RATE_MLH = 1.0

# Períodos de telemetría multi-rate EDA (ms)
PERIOD_SENSORS_MS = 500
PERIOD_MOTION_MS  = 1000
PERIOD_SESSION_MS = 2000
SIM_TICK_MS       = 100

MMHG_TO_PSI = 0.0193368

# ---------------------------------------------------------------------------
# Estados FSM de Core 1 — 1:1 con headers/core1_main.h (Core1State_t)
# ---------------------------------------------------------------------------
class Core1State(IntEnum):
    ST_UNHOMED             = 0
    ST_HOMING              = 1
    ST_RELEASING_LSW_START = 2
    ST_READY_AT_HOME       = 3
    ST_SEARCHING_SYRINGE   = 4
    ST_SYRINGE_ENGAGED     = 5
    ST_DISPENSING          = 6
    ST_DISPENSE_COMPLETED  = 7
    ST_SET_NEW_DISPENSE    = 8
    ST_SEARCHING_EOT       = 9
    ST_RELEASING_LSW_END   = 10
    ST_END_OF_TRAVEL       = 11
    ST_FAULT               = 12
    ST_OCCLUSION_STOPPING  = 13
    ST_OCCLUSION_RELEASE   = 14
    ST_OCCLUSION_PAUSED    = 15
    ST_MANUAL_OVERRIDE     = 16
    ST_CALIB_SEEK_START    = 17
    ST_CALIB_SEEK_END      = 18
    ST_BRAKING_LSW_START   = 19
    ST_BRAKING_LSW_END     = 20

CORE1_STATE_NAMES = {
    Core1State.ST_UNHOMED:             "ST_UNHOMED",
    Core1State.ST_HOMING:              "ST_HOMING",
    Core1State.ST_RELEASING_LSW_START: "ST_RELEASING_LSW_START",
    Core1State.ST_READY_AT_HOME:       "ST_READY_AT_HOME",
    Core1State.ST_SEARCHING_SYRINGE:   "ST_SEARCHING_SYRINGE",
    Core1State.ST_SYRINGE_ENGAGED:     "ST_SYRINGE_ENGAGED",
    Core1State.ST_DISPENSING:          "ST_DISPENSING",
    Core1State.ST_DISPENSE_COMPLETED:  "ST_DISPENSE_COMPLETED",
    Core1State.ST_SET_NEW_DISPENSE:    "ST_SET_NEW_DISPENSE",
    Core1State.ST_SEARCHING_EOT:       "ST_SEARCHING_EOT",
    Core1State.ST_RELEASING_LSW_END:   "ST_RELEASING_LSW_END",
    Core1State.ST_END_OF_TRAVEL:       "ST_END_OF_TRAVEL",
    Core1State.ST_FAULT:               "ST_FAULT",
    Core1State.ST_OCCLUSION_STOPPING:  "ST_OCCLUSION_STOPPING",
    Core1State.ST_OCCLUSION_RELEASE:   "ST_OCCLUSION_RELEASE",
    Core1State.ST_OCCLUSION_PAUSED:    "ST_OCCLUSION_PAUSED",
    Core1State.ST_MANUAL_OVERRIDE:     "ST_MANUAL_OVERRIDE",
    Core1State.ST_CALIB_SEEK_START:    "ST_CALIB_SEEK_START",
    Core1State.ST_CALIB_SEEK_END:      "ST_CALIB_SEEK_END",
    Core1State.ST_BRAKING_LSW_START:   "ST_BRAKING_LSW_START",
    Core1State.ST_BRAKING_LSW_END:     "ST_BRAKING_LSW_END",
}

# ---------------------------------------------------------------------------
# Estructuras de datos
# ---------------------------------------------------------------------------
@dataclass
class SyringeProfile:
    internal_diameter_mm: float = 19.05   # BD 20 mL por defecto
    max_capacity_ml: float       = 20.0

@dataclass
class PumpAlarms:
    occlusion:     bool = False
    bubble_in_line: bool = False
    eot:           bool = False
    syringe_empty: bool = False
    drv_fault:     bool = False
    battery_low:   bool = False

@dataclass
class PumpContext:
    session_id:               int        = 1
    target_volume_ml:         float      = 0.0
    infused_volume_ml:        float      = 0.0
    current_rate_ml_h:        float      = 0.0
    current_pressure_mmhg:    float      = 10.0
    current_pressure_psi:     float      = 0.193
    occlusion_threshold_mmhg: float      = OCC_THRESHOLDS[3]
    elapsed_time_s:           float      = 0.0
    position_mm:              float      = 0.0
    encoder_count:            int        = 0
    speed_ums:                float      = 0.0
    progress_pct:             float      = 0.0
    fsm_state:                Core1State = Core1State.ST_READY_AT_HOME
    alarms:                   PumpAlarms = field(default_factory=PumpAlarms)

# ---------------------------------------------------------------------------
# Modelo de Presión
# ---------------------------------------------------------------------------
class PressureModel:
    RAMP_UP_RATE_MMHG_S = 100.0
    RAMP_DOWN_TAU_S     = 3.0
    BASELINE_MEAN_MMHG  = 10.0
    BASELINE_NOISE_MMHG = 2.5
    FLOW_COEFF_MMHG_MLH = 0.05

    def __init__(self):
        self._fault_active = False
        self._current_p = self.BASELINE_MEAN_MMHG

    def inject_occlusion(self):
        self._fault_active = True

    def release(self):
        self._fault_active = False

    def update(self, delta_s: float, rate_ml_h: float, threshold_mmhg: float) -> Tuple[float, float]:
        noise = random.gauss(0, self.BASELINE_NOISE_MMHG * 0.3)
        flow_component = rate_ml_h * self.FLOW_COEFF_MMHG_MLH
        baseline = self.BASELINE_MEAN_MMHG + flow_component

        if self._fault_active:
            target = threshold_mmhg * 1.05
            self._current_p += self.RAMP_UP_RATE_MMHG_S * delta_s
            self._current_p = min(self._current_p, target)
        else:
            diff = self._current_p - baseline
            self._current_p -= diff * (1.0 - math.exp(-delta_s / self.RAMP_DOWN_TAU_S))
            self._current_p = max(self._current_p, 0.0)

        mmhg = max(0.0, self._current_p + noise)
        psi = mmhg * MMHG_TO_PSI
        return mmhg, psi

# ---------------------------------------------------------------------------
# Simulador Principal
# ---------------------------------------------------------------------------
class SyringePumpSimulator:
    def __init__(self, device_id: str, diam_mm: float, cap_ml: float):
        self.device_id = device_id
        self._lock = threading.Lock()

        self._ctx = PumpContext()
        self._syringe = SyringeProfile(diam_mm, cap_ml)
        self._syringe_area_mm2 = self._calc_area(diam_mm)
        self._pressure_model = PressureModel()

        self._event_queue: list = []
        self._state_timer: float = 0.0
        self._pending_fsm_transition: Optional[Core1State] = None

        log.info(f"[{self.device_id}] Inicializado. Jeringa: Ø{diam_mm} mm, {cap_ml} mL, Área={self._syringe_area_mm2:.2f} mm²")

    @staticmethod
    def _calc_area(diam_mm: float) -> float:
        r = diam_mm / 2.0
        return PI * r * r

    def _ml_to_mm(self, volume_ml: float) -> float:
        if self._syringe_area_mm2 <= 0:
            return 0.0
        return (volume_ml * 1000.0) / self._syringe_area_mm2

    def _ml_h_to_um_s(self, rate_ml_h: float) -> float:
        mm_h = self._ml_to_mm(rate_ml_h)
        return (mm_h / 3600.0) * 1000.0

    def pop_events(self) -> list:
        with self._lock:
            evs = list(self._event_queue)
            self._event_queue.clear()
            return evs

    def _set_fsm_state(self, new_state: Core1State):
        if self._ctx.fsm_state != new_state:
            prev = self._ctx.fsm_state
            self._ctx.fsm_state = new_state
            self._event_queue.append({
                "type": "state",
                "from": int(prev),
                "to":   int(new_state)
            })
            log.info(f"[{self.device_id}] FSM: {CORE1_STATE_NAMES[prev]} → {CORE1_STATE_NAMES[new_state]}")

    def _check_alarms(self):
        ctx = self._ctx
        alm = ctx.alarms

        # 1. Oclusión
        if ctx.current_pressure_mmhg >= ctx.occlusion_threshold_mmhg:
            if not alm.occlusion:
                alm.occlusion = True
                self._set_fsm_state(Core1State.ST_OCCLUSION_STOPPING)
                ctx.current_rate_ml_h = 0.0
                ctx.speed_ums = 0.0
                self._event_queue.append({"type": "alarm", "code": "occ", "level": 2})
                log.warning(f"[{self.device_id}] ALARMA OCLUSIÓN: {ctx.current_pressure_mmhg:.1f} mmHg ≥ {ctx.occlusion_threshold_mmhg:.1f} mmHg")
                # Auto anti-bolus programado en 0.5s
                self._state_timer = 0.5
                self._pending_fsm_transition = Core1State.ST_OCCLUSION_RELEASE
        else:
            alm.occlusion = False

        # 2. Fin de infusión
        if ctx.target_volume_ml > 0.0 and ctx.infused_volume_ml >= ctx.target_volume_ml:
            if ctx.fsm_state == Core1State.ST_DISPENSING:
                self._set_fsm_state(Core1State.ST_DISPENSE_COMPLETED)
                ctx.current_rate_ml_h = 0.0
                ctx.speed_ums = 0.0
                self._event_queue.append({"type": "info", "msg": "dispense_finished"})
                log.info(f"[{self.device_id}] Infusión objetivo completada")

        # 3. Límite de jeringa vacía / EOT
        if ctx.infused_volume_ml >= self._syringe.max_capacity_ml:
            if not alm.syringe_empty:
                alm.syringe_empty = True
                self._set_fsm_state(Core1State.ST_END_OF_TRAVEL)
                ctx.current_rate_ml_h = 0.0
                ctx.speed_ums = 0.0
                self._event_queue.append({"type": "alarm", "code": "eot", "level": 3})
                log.warning(f"[{self.device_id}] ALARMA FIN DE RECORRIDO / JERINGA VACÍA")

    def tick(self, delta_ms: int):
        if delta_ms <= 0:
            return
        with self._lock:
            ctx = self._ctx
            delta_s = delta_ms / 1000.0

            # Manejo de transiciones diferidas (simula movimiento mecánico real)
            if self._pending_fsm_transition is not None:
                self._state_timer -= delta_s
                if self._state_timer <= 0:
                    next_st = self._pending_fsm_transition
                    self._pending_fsm_transition = None
                    self._set_fsm_state(next_st)

                    # Encadenamientos de estados
                    if next_st == Core1State.ST_HOMING:
                        self._state_timer = 1.0
                        self._pending_fsm_transition = Core1State.ST_RELEASING_LSW_START
                    elif next_st == Core1State.ST_RELEASING_LSW_START:
                        self._state_timer = 0.5
                        self._pending_fsm_transition = Core1State.ST_READY_AT_HOME
                    elif next_st == Core1State.ST_SEARCHING_SYRINGE:
                        self._state_timer = 1.2
                        self._pending_fsm_transition = Core1State.ST_SYRINGE_ENGAGED
                    elif next_st == Core1State.ST_OCCLUSION_RELEASE:
                        self._state_timer = 0.8
                        self._pending_fsm_transition = Core1State.ST_OCCLUSION_PAUSED

            # Actualizar modelo de presión
            ctx.current_pressure_mmhg, ctx.current_pressure_psi = self._pressure_model.update(
                delta_s, ctx.current_rate_ml_h, ctx.occlusion_threshold_mmhg
            )

            # Integración de volumen y cinemática en estado DISPENSING
            if ctx.fsm_state == Core1State.ST_DISPENSING:
                ctx.elapsed_time_s += delta_s
                rate_ml_s = ctx.current_rate_ml_h / 3600.0
                delta_ml  = rate_ml_s * delta_s
                ctx.infused_volume_ml += delta_ml

                # Posición y encoder (suponiendo 1000 pulsos/mm)
                ctx.position_mm = self._ml_to_mm(ctx.infused_volume_ml)
                ctx.encoder_count = int(ctx.position_mm * 1000)
                ctx.speed_ums = self._ml_h_to_um_s(ctx.current_rate_ml_h)

                if ctx.target_volume_ml > 0:
                    ctx.progress_pct = min(100.0, (ctx.infused_volume_ml / ctx.target_volume_ml) * 100.0)
                else:
                    ctx.progress_pct = 0.0

                self._check_alarms()
            else:
                ctx.speed_ums = 0.0

    # -----------------------------------------------------------------------
    # Generadores de Telemetría EDA multi-rate (byte-exactos al firmware)
    # -----------------------------------------------------------------------
    def get_sensors_json(self) -> str:
        with self._lock:
            ctx = self._ctx
            return f'{{"pressure_psi":{ctx.current_pressure_psi:.3f},"pressure_mmhg":{ctx.current_pressure_mmhg:.1f}}}'

    def get_motion_json(self) -> str:
        with self._lock:
            ctx = self._ctx
            return (
                f'{{"encoder_count":{ctx.encoder_count},'
                f'"position_mm":{ctx.position_mm:.2f},'
                f'"speed_ums":{ctx.speed_ums:.1f},'
                f'"progress_pct":{ctx.progress_pct:.1f}}}'
            )

    def get_session_json(self) -> str:
        with self._lock:
            ctx = self._ctx
            return (
                f'{{"session_id":{ctx.session_id},'
                f'"infused_ml":{ctx.infused_volume_ml:.2f},'
                f'"target_ml":{ctx.target_volume_ml:.2f},'
                f'"rate_ml_h":{ctx.current_rate_ml_h:.1f},'
                f'"elapsed_s":{int(ctx.elapsed_time_s)},'
                f'"fsm_state":{int(ctx.fsm_state)}}}'
            )

    # -----------------------------------------------------------------------
    # Parser de Comandos (Pipeline idéntico a cmd_dispatcher.c)
    # -----------------------------------------------------------------------
    def parse_and_execute(self, cmd_str: str) -> Tuple[str, Optional[str]]:
        cmd = cmd_str.strip()
        log.debug(f"[{self.device_id}] CMD recibido: {cmd}")

        with self._lock:
            ctx = self._ctx

            # STOP inmediato o suave
            if cmd.lower() in ("stop", "stop_imm"):
                ctx.current_rate_ml_h = 0.0
                ctx.speed_ums = 0.0
                self._pending_fsm_transition = None
                self._set_fsm_state(Core1State.ST_READY_AT_HOME)
                return ("accepted", None)

            # FSM HOME
            if cmd.startswith("fsm_home"):
                self._set_fsm_state(Core1State.ST_HOMING)
                self._state_timer = 1.0
                self._pending_fsm_transition = Core1State.ST_RELEASING_LSW_START
                return ("accepted", None)

            # FSM SEARCH SYRINGE
            if cmd.startswith("fsm_search,") or cmd == "fsm_search":
                self._set_fsm_state(Core1State.ST_SEARCHING_SYRINGE)
                self._state_timer = 1.2
                self._pending_fsm_transition = Core1State.ST_SYRINGE_ENGAGED
                return ("accepted", None)

            # FSM DISPENSE (fsm_dispense,TARGET_UM,VELOCITY_UMS)
            if cmd.startswith("fsm_dispense,"):
                parts = cmd[13:].split(",")
                if len(parts) == 2:
                    try:
                        target_um = float(parts[0])
                        vel_ums   = float(parts[1])
                        if self._syringe_area_mm2 > 0:
                            target_ml = (target_um / 1000.0) * self._syringe_area_mm2 / 1000.0
                            rate_ml_h = vel_ums * 3600.0 * self._syringe_area_mm2 / 1e6
                            ctx.target_volume_ml  = min(ctx.infused_volume_ml + target_ml, self._syringe.max_capacity_ml)
                            ctx.current_rate_ml_h = rate_ml_h
                            ctx.session_id += 1
                            self._set_fsm_state(Core1State.ST_DISPENSING)
                            return ("accepted", None)
                    except ValueError:
                        pass
                return ("rejected", "bad_format")

            # FSM SEARCH EOT
            if cmd == "fsm_search_eot":
                self._set_fsm_state(Core1State.ST_SEARCHING_EOT)
                self._state_timer = 1.5
                self._pending_fsm_transition = Core1State.ST_END_OF_TRAVEL
                return ("accepted", None)

            # FSM RESET
            if cmd == "fsm_reset":
                ctx.infused_volume_ml = 0.0
                ctx.target_volume_ml  = 0.0
                ctx.current_rate_ml_h = 0.0
                ctx.elapsed_time_s    = 0.0
                ctx.position_mm       = 0.0
                ctx.encoder_count     = 0
                ctx.progress_pct      = 0.0
                ctx.alarms            = PumpAlarms()
                self._pressure_model.release()
                self._pending_fsm_transition = None
                self._set_fsm_state(Core1State.ST_READY_AT_HOME)
                return ("accepted", None)

            # FSM CONTINUE / RESUME
            if cmd in ("fsm_cont", "fsm_resume"):
                if ctx.fsm_state in (Core1State.ST_OCCLUSION_PAUSED, Core1State.ST_SET_NEW_DISPENSE):
                    if ctx.current_rate_ml_h <= 0:
                        ctx.current_rate_ml_h = 50.0 # valor nominal de respaldo
                    self._set_fsm_state(Core1State.ST_DISPENSING)
                    return ("accepted", None)
                return ("rejected", "invalid_state")

            # FSM OCCLUSION RELEASE
            if cmd == "fsm_occ_rel":
                self._pressure_model.release()
                ctx.alarms.occlusion = False
                self._set_fsm_state(Core1State.ST_OCCLUSION_RELEASE)
                self._state_timer = 0.6
                self._pending_fsm_transition = Core1State.ST_OCCLUSION_PAUSED
                return ("accepted", None)

            # FSM CALIBRATE
            if cmd.startswith("fsm_calibrate"):
                self._set_fsm_state(Core1State.ST_CALIB_SEEK_START)
                self._state_timer = 1.0
                self._pending_fsm_transition = Core1State.ST_CALIB_SEEK_END
                return ("accepted", None)

            # Fallback cinemático crudo: TARGET_UM,VELOCITY_UMS
            if "," in cmd:
                parts = cmd.split(",")
                if len(parts) == 2:
                    try:
                        target_um = float(parts[0])
                        vel_ums   = float(parts[1])
                        target_ml = (target_um / 1000.0) * self._syringe_area_mm2 / 1000.0
                        rate_ml_h = vel_ums * 3600.0 * self._syringe_area_mm2 / 1e6
                        ctx.target_volume_ml  = min(ctx.infused_volume_ml + target_ml, self._syringe.max_capacity_ml)
                        ctx.current_rate_ml_h = rate_ml_h
                        self._set_fsm_state(Core1State.ST_DISPENSING)
                        return ("accepted", None)
                    except ValueError:
                        pass

            # Comandos de configuración aceptados silenciosamente
            if any(cmd.startswith(prefix) for prefix in (
                "config_wifi,", "config_mqtt,", "config_thresholds,", "set_occ_thresh,",
                "set_syringe_thresh,", "set_occ_rel_thresh,", "config_antibolus,",
                "set_antibolus_auto,", "config_save", "config_info")):
                log.info(f"[{self.device_id}] Config aplicada: {cmd}")
                return ("accepted", None)

            log.warning(f"[{self.device_id}] Comando no reconocido: {cmd}")
            return ("rejected", "unknown_cmd")

    # -----------------------------------------------------------------------
    # Inyección de fallas / atajos de simulación (tópico sim_fault)
    # -----------------------------------------------------------------------
    def parse_sim_command(self, payload: str):
        cmd = payload.strip()

        if cmd == "fault_occ":
            with self._lock:
                self._pressure_model.inject_occlusion()
            log.info(f"[{self.device_id}] SIM: Inyección de oclusión activada")

        elif cmd == "fault_bubble":
            with self._lock:
                self._ctx.alarms.bubble_in_line = True
                self._set_fsm_state(Core1State.ST_FAULT)
                self._ctx.current_rate_ml_h = 0.0
                self._event_queue.append({"type": "alarm", "code": "bub", "level": 2})
            log.warning(f"[{self.device_id}] SIM: Alarma de burbuja inyectada")

        elif cmd == "fault_clear":
            with self._lock:
                self._ctx.alarms = PumpAlarms()
                self._pressure_model.release()
                if self._ctx.fsm_state in (Core1State.ST_FAULT, Core1State.ST_OCCLUSION_PAUSED, Core1State.ST_OCCLUSION_STOPPING):
                    self._set_fsm_state(Core1State.ST_READY_AT_HOME)
            log.info(f"[{self.device_id}] SIM: Fallas limpiadas y presión normalizada")

        elif cmd.startswith("infuse,"):
            parts = cmd[7:].split(",")
            if len(parts) == 2:
                rate_ml_h = float(parts[0])
                target_ml = float(parts[1])
                with self._lock:
                    self._ctx.target_volume_ml = min(self._ctx.infused_volume_ml + target_ml, self._syringe.max_capacity_ml)
                    self._ctx.current_rate_ml_h = rate_ml_h
                    self._ctx.session_id += 1
                    self._set_fsm_state(Core1State.ST_DISPENSING)
                log.info(f"[{self.device_id}] SIM: Infundiendo {rate_ml_h:.1f} mL/h hasta {target_ml:.1f} mL")

        elif cmd.startswith("bolus,"):
            parts = cmd[6:].split(",")
            if len(parts) == 2:
                vol_ml, rate_ml_h = float(parts[0]), float(parts[1])
                with self._lock:
                    self._ctx.target_volume_ml = min(self._ctx.infused_volume_ml + vol_ml, self._syringe.max_capacity_ml)
                    self._ctx.current_rate_ml_h = rate_ml_h
                    self._set_fsm_state(Core1State.ST_DISPENSING)
                log.info(f"[{self.device_id}] SIM: Bolo {vol_ml:.1f} mL @ {rate_ml_h:.1f} mL/h")

        elif cmd == "purge":
            with self._lock:
                self._ctx.target_volume_ml = min(self._ctx.infused_volume_ml + PURGE_VOLUME_ML, self._syringe.max_capacity_ml)
                self._ctx.current_rate_ml_h = PURGE_FLOW_RATE_MLH
                self._set_fsm_state(Core1State.ST_DISPENSING)
            log.info(f"[{self.device_id}] SIM: Purga")

        else:
            log.warning(f"[{self.device_id}] SIM: Comando desconocido: {cmd}")

# ---------------------------------------------------------------------------
# Cliente MQTT Multi-rate EDA
# ---------------------------------------------------------------------------
class PumpMQTTClient:
    RECONNECT_BACKOFF_S = [2, 5, 10, 30]

    def __init__(self, sim: SyringePumpSimulator, broker: str, port: int):
        self.sim    = sim
        self.broker = broker
        self.port   = port
        self._id    = sim.device_id

        # Tópicos modernos EDA
        self.topic_sensors   = f"bj/{self._id}/telemetry/sensors"
        self.topic_motion    = f"bj/{self._id}/telemetry/motion"
        self.topic_session   = f"bj/{self._id}/telemetry/session"
        self.topic_event     = f"bj/{self._id}/event"
        self.topic_status    = f"bj/{self._id}/status"
        self.topic_cmd       = f"bj/{self._id}/cmd"
        self.topic_cmd_ack   = f"bj/{self._id}/cmd/ack"
        self.topic_sim_fault = f"bj/{self._id}/sim_fault"

        self._client = mqtt.Client(client_id=f"sim_{self._id}", clean_session=True, protocol=mqtt.MQTTv311)
        self._client.on_connect    = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message    = self._on_message

        # LWT idéntico al firmware (bj/{id}/status retain=true)
        self._client.will_set(
            self.topic_status,
            payload=json.dumps({"state": "offline", "id": self._id}),
            qos=1,
            retain=True
        )

        self._connected   = False
        self._backoff_idx = 0
        self._stop_event  = threading.Event()

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected   = True
            self._backoff_idx = 0
            log.info(f"[{self._id}] Conectado a {self.broker}:{self.port}")

            # Online con retain
            client.publish(
                self.topic_status,
                json.dumps({"state": "online", "id": self._id, "fw": "v1.0.0-eda"}),
                qos=1, retain=True
            )

            client.subscribe(self.topic_cmd,       qos=1)
            client.subscribe(self.topic_sim_fault, qos=0)
            log.info(f"[{self._id}] Suscrito a comandos: {self.topic_cmd} | {self.topic_sim_fault}")
        else:
            log.error(f"[{self._id}] Error conexión MQTT rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        if rc != 0:
            log.warning(f"[{self._id}] Desconexión inesperada (rc={rc})")

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="replace").strip()

        if msg.topic == self.topic_sim_fault:
            self.sim.parse_sim_command(payload)
            return

        if msg.topic == self.topic_cmd:
            cid = None
            try:
                obj = json.loads(payload)
                cid     = obj.get("cid")
                cmd_str = obj.get("cmd", payload)
            except (json.JSONDecodeError, KeyError):
                cmd_str = payload

            result, reason = self.sim.parse_and_execute(cmd_str)

            if cid is not None:
                ack = {"cid": cid, "result": result}
                if reason:
                    ack["reason"] = reason
                client.publish(self.topic_cmd_ack, json.dumps(ack), qos=1)

    def _telemetry_thread(self):
        """Publicador multi-rate de telemetría (500 ms sensors, 1000 ms motion, 2000 ms session)"""
        t_sensors = 0.0
        t_motion  = 0.0
        t_session = 0.0

        while not self._stop_event.is_set():
            now = time.time()

            if self._connected:
                # Sensors (500 ms)
                if now - t_sensors >= (PERIOD_SENSORS_MS / 1000.0):
                    self._client.publish(self.topic_sensors, self.sim.get_sensors_json(), qos=0)
                    t_sensors = now

                # Motion (1000 ms)
                if now - t_motion >= (PERIOD_MOTION_MS / 1000.0):
                    self._client.publish(self.topic_motion, self.sim.get_motion_json(), qos=0)
                    t_motion = now

                # Session (2000 ms)
                if now - t_session >= (PERIOD_SESSION_MS / 1000.0):
                    self._client.publish(self.topic_session, self.sim.get_session_json(), qos=0)
                    t_session = now

            time.sleep(0.05)

    def _sim_tick_thread(self):
        while not self._stop_event.is_set():
            self.sim.tick(SIM_TICK_MS)
            time.sleep(SIM_TICK_MS / 1000.0)

    def _event_thread(self):
        while not self._stop_event.is_set():
            if self._connected:
                for ev in self.sim.pop_events():
                    self._client.publish(self.topic_event, json.dumps(ev), qos=1)
            time.sleep(0.05)

    def run(self):
        # Conexión inicial
        while not self._stop_event.is_set():
            try:
                self._client.connect(self.broker, self.port, keepalive=60)
                self._client.loop_start()
                break
            except Exception as e:
                delay = self.RECONNECT_BACKOFF_S[min(self._backoff_idx, len(self.RECONNECT_BACKOFF_S) - 1)]
                log.error(f"[{self._id}] No se pudo conectar ({e}). Reintento en {delay}s...")
                self._backoff_idx += 1
                time.sleep(delay)

        threading.Thread(target=self._sim_tick_thread,  daemon=True, name="tick").start()
        threading.Thread(target=self._telemetry_thread, daemon=True, name="telem").start()
        threading.Thread(target=self._event_thread,     daemon=True, name="event").start()

        log.info(f"[{self._id}] Simulador EDA corriendo.")
        log.info(f"[{self._id}] Telemetría Sensors → {self.topic_sensors} (500ms)")
        log.info(f"[{self._id}] Telemetría Motion  → {self.topic_motion} (1000ms)")
        log.info(f"[{self._id}] Telemetría Session → {self.topic_session} (2000ms)")
        log.info(f"[{self._id}] Eventos FSM       → {self.topic_event}")
        log.info(f"[{self._id}] Canal Comandos     ← {self.topic_cmd}")

        try:
            while True:
                time.sleep(5)
        except KeyboardInterrupt:
            log.info(f"[{self._id}] Deteniendo simulador...")
        finally:
            self._stop_event.set()
            self._client.publish(
                self.topic_status,
                json.dumps({"state": "offline", "id": self._id}),
                qos=1, retain=True
            )
            self._client.loop_stop()
            self._client.disconnect()

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Simulador de Bomba de Infusión (pico2w_syringe_pump - EDA)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  python pump_simulator.py --broker localhost --id bj-001
  python pump_simulator.py --broker 192.168.1.100 --id bj-002 --diam 14.5 --cap 10.0

Comandos con mosquitto_pub:
  mosquitto_pub -h localhost -t bj/bj-001/cmd -m '{"cid":1,"cmd":"fsm_home,1500"}'
  mosquitto_pub -h localhost -t bj/bj-001/cmd -m '{"cid":2,"cmd":"fsm_dispense,30000,500"}'
  mosquitto_pub -h localhost -t bj/bj-001/sim_fault -m "fault_occ"
  mosquitto_pub -h localhost -t bj/bj-001/sim_fault -m "fault_clear"
        """
    )
    p.add_argument("--broker", default="localhost",       help="IP/host broker MQTT")
    p.add_argument("--port",   type=int, default=1883,    help="Puerto broker MQTT")
    p.add_argument("--id",     default="bj-001",          help="Device ID de la bomba")
    p.add_argument("--diam",   type=float, default=19.05, help="Diámetro interno émbolo (mm)")
    p.add_argument("--cap",    type=float, default=20.0,  help="Capacidad máxima (mL)")
    p.add_argument("--debug",  action="store_true",       help="Log DEBUG")
    return p

def main():
    args = build_parser().parse_args()
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    sim = SyringePumpSimulator(args.id, args.diam, args.cap)
    client = PumpMQTTClient(sim, args.broker, args.port)
    client.run()

if __name__ == "__main__":
    main()
