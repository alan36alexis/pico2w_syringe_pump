#!/usr/bin/env python3
"""
pump_simulator.py — Simulador de Bomba de Infusión a Jeringa
=============================================================
Replica el comportamiento del firmware pico2w_syringe_pump (rama tft_integration)
con telemetría JSON byte-exacta a la de Pump_GetTelemetryJSON().

Fuentes de verdad portadas:
  - src/syringe_pump_api.c   → modelo de estado, cinemática, alarmas, JSON
  - src/crosscore_cmd.c      → parser de comandos MQTT/CLI
  - src/system_config.h      → constantes clínicas y umbrales de oclusión

Extras del simulador (no están en el firmware):
  - Modelo de presión físicamente razonable (ruido basal + rampa de oclusión)
  - Inyección de fallas vía tópico MQTT separado (sin contaminar el canal de cmd)
  - Modo multi-instancia: --id permite correr N bombas en paralelo
  - Reconexión automática al broker con back-off

Uso rápido:
    pip install paho-mqtt
    python pump_simulator.py --broker 192.168.1.100 --id bj-001
    python pump_simulator.py --broker 192.168.1.100 --id bj-002 --diam 19.05 --cap 20.0

Comandos vía MQTT (tópico: bj/{device_id}/cmd):
    Payload JSON del contrato: {"cid": 17, "cmd": "stop"}
    También acepta string crudo por compatibilidad: stop

Inyección de fallas (tópico: bj/{device_id}/sim_fault):
    fault_occ          →  simula oclusión (presión sube hasta umbral)
    fault_bubble       →  dispara alarma de burbuja
    fault_clear        →  limpia todas las fallas inyectadas
    select_syringe,<diam_mm>,<cap_ml>   →  reconfigura jeringa en caliente
    infuse,<rate_ml_h>,<vol_ml>         →  modo continuo con volumen objetivo
    bolus,<vol_ml>,<rate_ml_h>          →  bolo
    purge                               →  purga
    kvo                                 →  modo KVO
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
# Constantes — portadas de system_config.h y syringe_pump_api.h
# ---------------------------------------------------------------------------
PI = 3.1415926535

# Umbrales de oclusión (mmHg) — OCC_THRESHOLD_Lx_MMHG
OCC_THRESHOLDS = [225.0, 475.0, 725.0, 975.0]

# Modos clínicos por defecto
PURGE_FLOW_RATE_MLH = 1000.0
PURGE_VOLUME_ML = 1.0
KVO_FLOW_RATE_MLH = 1.0

# Período de telemetría (ms) — coincide con task_pump_telemetry del firmware
TELEMETRY_PERIOD_MS = 2000

# Tick interno del simulador (ms) — resolución de la integración de volumen
SIM_TICK_MS = 100

# ---------------------------------------------------------------------------
# Enumeraciones — portadas de syringe_pump_api.h
# ---------------------------------------------------------------------------
class PumpState(IntEnum):
    STOPPED               = 0
    INFUSING_CONTINUOUS   = 1
    INFUSING_BOLUS        = 2
    PURGING               = 3
    KVO                   = 4
    PAUSED                = 5
    ALARM                 = 6

# Nombres legibles para el log
STATE_NAMES = {
    PumpState.STOPPED:             "STOPPED",
    PumpState.INFUSING_CONTINUOUS: "INFUSING_CONTINUOUS",
    PumpState.INFUSING_BOLUS:      "INFUSING_BOLUS",
    PumpState.PURGING:             "PURGING",
    PumpState.KVO:                 "KVO",
    PumpState.PAUSED:              "PAUSED",
    PumpState.ALARM:               "ALARM",
}

# ---------------------------------------------------------------------------
# Estructuras de datos — portadas de syringe_pump_api.h
# ---------------------------------------------------------------------------
@dataclass
class SyringeProfile:
    internal_diameter_mm: float = 19.13   # 20 mL Becton-Dickinson por defecto
    max_capacity_ml: float       = 20.0

@dataclass
class PumpAlarms:
    occlusion:             bool = False
    near_end_of_infusion:  bool = False   # último 10%
    end_of_infusion:       bool = False
    bubble_in_line:        bool = False
    syringe_empty:         bool = False
    system_error:          bool = False

@dataclass
class PumpContext:
    target_volume_ml:         float = 0.0
    infused_volume_ml:        float = 0.0
    current_rate_ml_h:        float = 0.0
    current_pressure_mmhg:    float = 0.0
    occlusion_threshold_mmhg: float = OCC_THRESHOLDS[3]
    elapsed_time_s:           float = 0.0
    alarms:                   PumpAlarms = field(default_factory=PumpAlarms)
    state:                    PumpState  = PumpState.STOPPED

# ---------------------------------------------------------------------------
# Modelo de presión — no existe en el firmware (el hardware lo mide directo)
# ---------------------------------------------------------------------------
class PressureModel:
    """
    Simula la presión hidráulica en línea de forma físicamente razonable.

    Comportamiento:
      - Basalino ruidoso en reposo: 5–15 mmHg
      - Infusión normal: presión leve proporcional al caudal
      - Fault de oclusión activo: rampa lineal hasta el umbral configurado
        a ~100 mmHg/s (ocluyendo a 450 mL/h en una tubería de 4 Fr es ~90 mmHg/s)
      - Al liberar oclusión (fsm_occ_rel): caída exponencial de regreso al basal
    """
    RAMP_UP_RATE_MMHG_S   = 100.0   # pendiente de subida al ocluyir
    RAMP_DOWN_TAU_S       = 3.0     # constante de tiempo de caída (exponencial)
    BASELINE_MEAN_MMHG    = 10.0
    BASELINE_NOISE_MMHG   = 3.0
    FLOW_COEFF_MMHG_MLH   = 0.05    # 0.05 mmHg por (mL/h) → 50 mL/h ≈ 2.5 mmHg extra

    def __init__(self):
        self._fault_active = False
        self._current_p = self.BASELINE_MEAN_MMHG

    def inject_occlusion(self):
        self._fault_active = True

    def release(self):
        self._fault_active = False

    def update(self, delta_s: float, rate_ml_h: float, threshold_mmhg: float) -> float:
        noise = random.gauss(0, self.BASELINE_NOISE_MMHG * 0.3)
        flow_component = rate_ml_h * self.FLOW_COEFF_MMHG_MLH
        baseline = self.BASELINE_MEAN_MMHG + flow_component

        if self._fault_active:
            # Sube lineal hasta el umbral + 5% (reproducir el overshoot del sensor)
            target = threshold_mmhg * 1.05
            self._current_p += self.RAMP_UP_RATE_MMHG_S * delta_s
            self._current_p = min(self._current_p, target)
        else:
            # Decaimiento exponencial hacia el basalino
            diff = self._current_p - baseline
            self._current_p -= diff * (1.0 - math.exp(-delta_s / self.RAMP_DOWN_TAU_S))
            self._current_p = max(self._current_p, 0.0)

        return self._current_p + noise

# ---------------------------------------------------------------------------
# Simulador principal — porta syringe_pump_api.c + cmd_parse_and_execute()
# ---------------------------------------------------------------------------
class SyringePumpSimulator:
    """
    Réplica del PumpContext_t + lógica de syringe_pump_api.c en Python.
    Thread-safe vía un único lock que protege ctx + syringe + event_queue.
    """

    def __init__(self, device_id: str, diam_mm: float, cap_ml: float):
        self.device_id = device_id
        self._lock = threading.Lock()

        self._ctx = PumpContext()
        self._syringe = SyringeProfile(diam_mm, cap_ml)
        self._syringe_area_mm2 = self._calc_area(diam_mm)
        self._syringe_selected = True
        self._pressure_model = PressureModel()

        self._event_queue: list = []

        log.info(f"[{self.device_id}] Jeringa configurada: Ø{diam_mm} mm, {cap_ml} mL")

    # -----------------------------------------------------------------------
    # Event queue — drenado por PumpMQTTClient para publicar al tópico event
    # -----------------------------------------------------------------------
    def pop_events(self) -> list:
        with self._lock:
            events = list(self._event_queue)
            self._event_queue.clear()
            return events

    # -----------------------------------------------------------------------
    # Cinemática interna — portada de syringe_pump_api.c
    # -----------------------------------------------------------------------
    @staticmethod
    def _calc_area(diam_mm: float) -> float:
        r = diam_mm / 2.0
        return PI * r * r

    def _ml_to_um(self, volume_ml: float) -> float:
        """Volumen (mL) → desplazamiento lineal (µm). Idéntico al C."""
        if self._syringe_area_mm2 <= 0.0:
            return 0.0
        volume_mm3 = volume_ml * 1000.0
        length_mm  = volume_mm3 / self._syringe_area_mm2
        return length_mm * 1000.0

    def _ml_h_to_um_s(self, rate_ml_h: float) -> float:
        """Caudal (mL/h) → velocidad lineal (µm/s). Idéntico al C."""
        return self._ml_to_um(rate_ml_h / 3600.0)

    # -----------------------------------------------------------------------
    # Pump_CheckAlarms() — portada exacta + generación de eventos
    # -----------------------------------------------------------------------
    def _check_alarms(self):
        ctx = self._ctx
        alm = ctx.alarms

        # 1. Oclusión
        if ctx.current_pressure_mmhg >= ctx.occlusion_threshold_mmhg:
            if not alm.occlusion:
                alm.occlusion = True
                self._stop_motor()
                ctx.state = PumpState.ALARM
                self._event_queue.append({"type": "alarm", "code": "occ", "level": 2})
                log.warning(f"[{self.device_id}] ALARMA: Oclusión detectada "
                            f"({ctx.current_pressure_mmhg:.1f} mmHg ≥ "
                            f"{ctx.occlusion_threshold_mmhg:.1f} mmHg)")
        else:
            alm.occlusion = False

        # 2. Proximidad al final de infusión (último 10%)
        if ctx.target_volume_ml > 0.0:
            remaining = ctx.target_volume_ml - ctx.infused_volume_ml
            if 0.0 < remaining <= ctx.target_volume_ml * 0.10:
                if not alm.near_end_of_infusion:
                    alm.near_end_of_infusion = True
                    self._event_queue.append({"type": "info", "msg": "near_end"})
                    log.info(f"[{self.device_id}] Aviso: último 10% de infusión")
            else:
                alm.near_end_of_infusion = False

            # 3. Fin de infusión
            if ctx.infused_volume_ml >= ctx.target_volume_ml:
                if not alm.end_of_infusion:
                    alm.end_of_infusion = True
                    self._event_queue.append({"type": "info", "msg": "kvo_start"})
                    log.info(f"[{self.device_id}] Fin de infusión → KVO")
                    if ctx.state not in (PumpState.KVO, PumpState.ALARM):
                        self._mode_kvo()
            else:
                alm.end_of_infusion = False

        # 4. Jeringa vacía (límite físico)
        if (self._syringe_selected and
                ctx.infused_volume_ml >= self._syringe.max_capacity_ml):
            if not alm.syringe_empty:
                alm.syringe_empty = True
                self._stop_motor()
                ctx.state = PumpState.ALARM
                self._event_queue.append({"type": "alarm", "code": "emp", "level": 2})
                log.warning(f"[{self.device_id}] ALARMA: Jeringa vacía")

    # -----------------------------------------------------------------------
    # Motor helpers internos (sin acceso al hardware real)
    # -----------------------------------------------------------------------
    def _stop_motor(self):
        """Soft stop — reproduce cmd_send_stop_motor()."""
        self._ctx.current_rate_ml_h = 0.0

    def _stop_immediate(self):
        """Hard stop — reproduce cmd_send_stop_immediate()."""
        self._stop_motor()

    # -----------------------------------------------------------------------
    # Modos de operación — portados de Pump_Mode_*
    # -----------------------------------------------------------------------
    def _mode_continuous(self, rate_ml_h: float) -> bool:
        ctx = self._ctx
        if not self._syringe_selected:                     return False
        if rate_ml_h < 0.01 or rate_ml_h > 2000.0:        return False
        remaining = self._syringe.max_capacity_ml - ctx.infused_volume_ml
        if remaining <= 0.0:                               return False

        ctx.target_volume_ml  = self._syringe.max_capacity_ml
        ctx.current_rate_ml_h = rate_ml_h
        ctx.state             = PumpState.INFUSING_CONTINUOUS
        log.info(f"[{self.device_id}] Modo continuo: {rate_ml_h:.2f} mL/h")
        return True

    def _mode_continuous_with_target(self, rate_ml_h: float, target_ml: float) -> bool:
        """Extensión del simulador: continuo con volumen objetivo explícito."""
        ctx = self._ctx
        if not self._syringe_selected:                     return False
        if rate_ml_h < 0.01 or rate_ml_h > 2000.0:        return False
        remaining = self._syringe.max_capacity_ml - ctx.infused_volume_ml
        if remaining <= 0.0:                               return False

        ctx.target_volume_ml  = min(ctx.infused_volume_ml + target_ml,
                                    self._syringe.max_capacity_ml)
        ctx.current_rate_ml_h = rate_ml_h
        ctx.state             = PumpState.INFUSING_CONTINUOUS
        log.info(f"[{self.device_id}] Modo continuo: {rate_ml_h:.2f} mL/h "
                 f"→ target {ctx.target_volume_ml:.2f} mL")
        return True

    def _mode_bolus(self, bolus_vol_ml: float, bolus_rate_ml_h: float) -> bool:
        ctx = self._ctx
        if not self._syringe_selected:         return False
        if bolus_vol_ml <= 0 or bolus_rate_ml_h <= 0: return False
        remaining = self._syringe.max_capacity_ml - ctx.infused_volume_ml
        if bolus_vol_ml > remaining:           return False

        ctx.target_volume_ml  = ctx.infused_volume_ml + bolus_vol_ml
        ctx.current_rate_ml_h = bolus_rate_ml_h
        ctx.state             = PumpState.INFUSING_BOLUS
        log.info(f"[{self.device_id}] Bolo: {bolus_vol_ml:.2f} mL "
                 f"@ {bolus_rate_ml_h:.2f} mL/h")
        return True

    def _mode_purge(self) -> bool:
        if not self._syringe_selected: return False
        self._ctx.state             = PumpState.PURGING
        self._ctx.current_rate_ml_h = PURGE_FLOW_RATE_MLH
        self._ctx.target_volume_ml  = (self._ctx.infused_volume_ml + PURGE_VOLUME_ML)
        log.info(f"[{self.device_id}] Purga iniciada")
        return True

    def _mode_kvo(self) -> bool:
        if not self._syringe_selected: return False
        remaining = self._syringe.max_capacity_ml - self._ctx.infused_volume_ml
        if remaining <= 0.0: return False
        self._ctx.target_volume_ml  = self._syringe.max_capacity_ml
        self._ctx.current_rate_ml_h = KVO_FLOW_RATE_MLH
        self._ctx.state             = PumpState.KVO
        log.info(f"[{self.device_id}] Modo KVO")
        return True

    def _pump_stop(self) -> bool:
        self._ctx.state = PumpState.STOPPED
        self._stop_motor()
        log.info(f"[{self.device_id}] Stop")
        return True

    # -----------------------------------------------------------------------
    # Pump_Tick() — motor de integración de volumen (portado exacto)
    # -----------------------------------------------------------------------
    def tick(self, delta_ms: int):
        """Llamar periódicamente desde el hilo de simulación."""
        if delta_ms <= 0:
            return
        with self._lock:
            ctx = self._ctx
            delta_s = delta_ms / 1000.0
            prev_state = ctx.state

            # Actualizar presión con el modelo físico
            ctx.current_pressure_mmhg = self._pressure_model.update(
                delta_s,
                ctx.current_rate_ml_h,
                ctx.occlusion_threshold_mmhg
            )

            # Integrar volumen solo si el motor está activo (= misma lógica que el C)
            active_states = (
                PumpState.INFUSING_CONTINUOUS,
                PumpState.INFUSING_BOLUS,
                PumpState.PURGING,
                PumpState.KVO,
            )
            if ctx.state in active_states:
                ctx.elapsed_time_s += delta_s
                rate_ml_s = ctx.current_rate_ml_h / 3600.0
                delta_ml  = rate_ml_s * delta_s
                ctx.infused_volume_ml += delta_ml
                self._check_alarms()

            # Detectar transición de estado (incluyendo cambios de _check_alarms)
            if ctx.state != prev_state:
                self._event_queue.append({
                    "type": "state",
                    "from": int(prev_state),
                    "to":   int(ctx.state)
                })

    # -----------------------------------------------------------------------
    # Pump_GetTelemetryJSON() — byte-exacto al snprintf del firmware
    # -----------------------------------------------------------------------
    def get_telemetry_json(self) -> str:
        with self._lock:
            ctx = self._ctx
            alm = ctx.alarms

            remaining_ml = max(ctx.target_volume_ml - ctx.infused_volume_ml, 0.0)
            time_rem_h = 0.0
            if (ctx.current_rate_ml_h > 0.0 and
                    ctx.state in (PumpState.INFUSING_CONTINUOUS,
                                  PumpState.INFUSING_BOLUS,
                                  PumpState.KVO)):
                time_rem_h = remaining_ml / ctx.current_rate_ml_h

            # Mismo orden de campos y misma precisión decimal que snprintf del C
            return (
                f'{{"vol_inf":{ctx.infused_volume_ml:.2f},'
                f'"vol_tgt":{ctx.target_volume_ml:.2f},'
                f'"rate":{ctx.current_rate_ml_h:.2f},'
                f'"t_ela_s":{ctx.elapsed_time_s:.1f},'
                f'"t_rem_h":{time_rem_h:.2f},'
                f'"pres":{ctx.current_pressure_mmhg:.1f},'
                f'"st":{int(ctx.state)},'
                f'"alm":{{'
                f'"occ":{int(alm.occlusion)},'
                f'"near":{int(alm.near_end_of_infusion)},'
                f'"end":{int(alm.end_of_infusion)},'
                f'"bub":{int(alm.bubble_in_line)},'
                f'"emp":{int(alm.syringe_empty)},'
                f'"err":{int(alm.system_error)}'
                f'}}}}'
            )

    # -----------------------------------------------------------------------
    # cmd_parse_and_execute() — portado exacto de crosscore_cmd.c
    # Retorna ("accepted", None) o ("rejected", reason) para el ACK MQTT.
    # -----------------------------------------------------------------------
    def parse_and_execute(self, payload: str) -> Tuple[str, Optional[str]]:
        cmd = payload.strip()
        log.debug(f"[{self.device_id}] CMD: {cmd}")

        with self._lock:
            # --- Stop inmediato ---
            if cmd.lower() == "stop_imm":
                self._stop_immediate()
                self._ctx.state = PumpState.STOPPED
                return ("accepted", None)

            # --- Stop suave ---
            if cmd.lower() == "stop":
                self._pump_stop()
                return ("accepted", None)

            # --- FSM: Homing ---
            if cmd.startswith("fsm_home,"):
                self._ctx.state = PumpState.STOPPED
                log.info(f"[{self.device_id}] FSM: HOME simulado (velocidad {cmd[9:]} µm/s)")
                return ("accepted", None)

            if cmd.startswith("fsm_search,"):
                self._ctx.state = PumpState.STOPPED
                log.info(f"[{self.device_id}] FSM: SEARCH SYRINGE simulado")
                return ("accepted", None)

            if cmd.startswith("fsm_dispense,"):
                parts = cmd[13:].split(",")
                if len(parts) == 2:
                    try:
                        target_um = float(parts[0])
                        vel_ums   = float(parts[1])
                        if self._syringe_area_mm2 > 0:
                            target_ml = (target_um / 1000.0) * self._syringe_area_mm2 / 1000.0
                            rate_ml_h = vel_ums * 3600.0 * self._syringe_area_mm2 / (1e6)
                            ok = self._mode_continuous_with_target(rate_ml_h, target_ml)
                            return ("accepted", None) if ok else ("rejected", "invalid_state")
                    except ValueError:
                        pass
                return ("rejected", "invalid_params")

            if cmd == "fsm_search_eot":
                log.info(f"[{self.device_id}] FSM: SEARCH EOT simulado")
                return ("accepted", None)

            if cmd == "fsm_reset":
                self._ctx.state             = PumpState.STOPPED
                self._ctx.infused_volume_ml = 0.0
                self._ctx.target_volume_ml  = 0.0
                self._ctx.elapsed_time_s    = 0.0
                self._ctx.alarms            = PumpAlarms()
                self._pressure_model.release()
                log.info(f"[{self.device_id}] FSM: RESET")
                return ("accepted", None)

            if cmd == "fsm_cont":
                if self._ctx.state == PumpState.PAUSED:
                    self._ctx.state = PumpState.INFUSING_CONTINUOUS
                return ("accepted", None)

            if cmd == "fsm_occ_rel":
                self._pressure_model.release()
                self._ctx.alarms.occlusion = False
                self._ctx.state = PumpState.STOPPED
                log.info(f"[{self.device_id}] Oclusión liberada")
                return ("accepted", None)

            if cmd == "fsm_resume":
                if self._ctx.state in (PumpState.PAUSED, PumpState.ALARM):
                    self._ctx.state = PumpState.INFUSING_CONTINUOUS
                return ("accepted", None)

            if cmd == "fsm_calibrate":
                log.info(f"[{self.device_id}] FSM: CALIBRATE simulado")
                return ("accepted", None)

            # --- Home manual ---
            if cmd.startswith("home_start,") or cmd.startswith("home_end,"):
                log.info(f"[{self.device_id}] Homing manual simulado")
                return ("accepted", None)

            # --- Pausa ---
            if cmd == "pause":
                if self._ctx.state in (PumpState.INFUSING_CONTINUOUS,
                                       PumpState.INFUSING_BOLUS):
                    self._ctx.state = PumpState.PAUSED
                    self._stop_motor()
                return ("accepted", None)

            # --- Fallback: "target_um,velocity_ums" ---
            if "," in cmd:
                parts = cmd.split(",")
                if len(parts) == 2:
                    try:
                        target_um = float(parts[0])
                        vel_ums   = float(parts[1])
                        if self._syringe_area_mm2 > 0:
                            target_ml = (target_um / 1000.0) * self._syringe_area_mm2 / 1000.0
                            rate_ml_h = vel_ums * 3600.0 * self._syringe_area_mm2 / 1e6
                            ok = self._mode_continuous_with_target(rate_ml_h, target_ml)
                            return ("accepted", None) if ok else ("rejected", "invalid_state")
                    except ValueError:
                        pass

            log.warning(f"[{self.device_id}] Comando no reconocido: {cmd}")
            return ("rejected", "unknown_cmd")

    # -----------------------------------------------------------------------
    # Comandos exclusivos del simulador (tópico sim_fault)
    # -----------------------------------------------------------------------
    def parse_sim_command(self, payload: str):
        """
        Comandos de inyección de fallas y configuración de la simulación.
        No existen en el firmware — viven en un tópico separado para no
        contaminar el canal de comandos reales.
        """
        cmd = payload.strip()

        if cmd == "fault_occ":
            with self._lock:
                self._pressure_model.inject_occlusion()
            log.info(f"[{self.device_id}] SIM: Inyección de oclusión activada")

        elif cmd == "fault_bubble":
            with self._lock:
                self._ctx.alarms.bubble_in_line = True
                self._ctx.state = PumpState.ALARM
                self._stop_motor()
                self._event_queue.append({"type": "alarm", "code": "bub", "level": 2})
            log.warning(f"[{self.device_id}] SIM: Alarma de burbuja inyectada")

        elif cmd == "fault_clear":
            with self._lock:
                self._ctx.alarms         = PumpAlarms()
                self._ctx.state          = PumpState.STOPPED
                self._pressure_model.release()
            log.info(f"[{self.device_id}] SIM: Fallas limpiadas")

        elif cmd.startswith("select_syringe,"):
            parts = cmd[15:].split(",")
            if len(parts) == 2:
                diam, cap = float(parts[0]), float(parts[1])
                with self._lock:
                    self._syringe = SyringeProfile(diam, cap)
                    self._syringe_area_mm2 = self._calc_area(diam)
                    self._syringe_selected = True
                log.info(f"[{self.device_id}] SIM: Jeringa reconfigurada Ø{diam} mm, {cap} mL")

        elif cmd.startswith("infuse,"):
            parts = cmd[7:].split(",")
            if len(parts) == 2:
                rate_ml_h  = float(parts[0])
                target_ml  = float(parts[1])
                with self._lock:
                    self._mode_continuous_with_target(rate_ml_h, target_ml)

        elif cmd.startswith("bolus,"):
            parts = cmd[6:].split(",")
            if len(parts) == 2:
                vol_ml, rate_ml_h = float(parts[0]), float(parts[1])
                with self._lock:
                    self._mode_bolus(vol_ml, rate_ml_h)

        elif cmd == "purge":
            with self._lock:
                self._mode_purge()

        elif cmd == "kvo":
            with self._lock:
                self._mode_kvo()

        else:
            log.warning(f"[{self.device_id}] SIM: Comando desconocido: {cmd}")

    def get_state_name(self) -> str:
        with self._lock:
            return STATE_NAMES.get(self._ctx.state, "UNKNOWN")

# ---------------------------------------------------------------------------
# Cliente MQTT — integra el simulador con el broker
# ---------------------------------------------------------------------------
class PumpMQTTClient:
    RECONNECT_BACKOFF_S = [2, 5, 10, 30, 60]

    def __init__(self, sim: SyringePumpSimulator, broker: str, port: int):
        self.sim    = sim
        self.broker = broker
        self.port   = port
        self._id    = sim.device_id

        # Tópicos — jerarquía bj/{device_id}/... del MQTT_CONTRACT
        self.topic_telemetry = f"bj/{self._id}/telemetry"
        self.topic_cmd       = f"bj/{self._id}/cmd"
        self.topic_cmd_ack   = f"bj/{self._id}/cmd/ack"
        self.topic_event     = f"bj/{self._id}/event"
        self.topic_status    = f"bj/{self._id}/status"
        self.topic_sim_fault = f"bj/{self._id}/sim_fault"

        self._client = mqtt.Client(client_id=f"sim_{self._id}",
                                   clean_session=True,
                                   protocol=mqtt.MQTTv311)
        self._client.on_connect    = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message    = self._on_message

        # LWT: cuando el proceso muere, el broker publica offline
        self._client.will_set(
            self.topic_status,
            payload=json.dumps({"state": "offline", "id": self._id}),
            qos=1,
            retain=True
        )

        self._connected     = False
        self._backoff_idx   = 0
        self._stop_event    = threading.Event()

    # -----------------------------------------------------------------------
    # Callbacks MQTT
    # -----------------------------------------------------------------------
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected   = True
            self._backoff_idx = 0
            log.info(f"[{self._id}] Conectado al broker {self.broker}:{self.port}")

            # Publicar presencia
            client.publish(
                self.topic_status,
                json.dumps({"state": "online", "id": self._id, "fw": "sim-1.0"}),
                qos=1, retain=True
            )
            # Suscribir a cmd y al canal de fallas del simulador
            client.subscribe(self.topic_cmd,       qos=1)
            client.subscribe(self.topic_sim_fault, qos=0)
            log.info(f"[{self._id}] Suscrito a: {self.topic_cmd} | {self.topic_sim_fault}")
        else:
            log.error(f"[{self._id}] Error de conexión MQTT rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        if rc != 0:
            log.warning(f"[{self._id}] Desconectado inesperadamente (rc={rc}), reintentando...")

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="replace").strip()

        if msg.topic == self.topic_sim_fault:
            self.sim.parse_sim_command(payload)
            return

        if msg.topic == self.topic_cmd:
            # Intentar parsear el envelope JSON del contrato: {"cid": N, "cmd": "..."}
            cid = None
            try:
                obj = json.loads(payload)
                cid     = obj.get("cid")
                cmd_str = obj["cmd"]
            except (json.JSONDecodeError, KeyError):
                cmd_str = payload   # backward compat: string crudo sin envelope

            result, reason = self.sim.parse_and_execute(cmd_str)

            if cid is not None:
                ack = {"cid": cid, "result": result}
                if reason:
                    ack["reason"] = reason
                client.publish(self.topic_cmd_ack, json.dumps(ack), qos=1)

    # -----------------------------------------------------------------------
    # Loop principal con reconexión automática
    # -----------------------------------------------------------------------
    def _reconnect_loop(self):
        while not self._stop_event.is_set():
            try:
                self._client.connect(self.broker, self.port, keepalive=60)
                self._client.loop_start()
                break
            except Exception as e:
                delay = self.RECONNECT_BACKOFF_S[
                    min(self._backoff_idx, len(self.RECONNECT_BACKOFF_S) - 1)
                ]
                log.error(f"[{self._id}] No se pudo conectar ({e}). Reintento en {delay}s")
                self._backoff_idx += 1
                time.sleep(delay)

    def _telemetry_loop(self):
        """Publica telemetría cada TELEMETRY_PERIOD_MS ms."""
        while not self._stop_event.is_set():
            if self._connected:
                payload = self.sim.get_telemetry_json()
                self._client.publish(self.topic_telemetry, payload, qos=0)
            time.sleep(TELEMETRY_PERIOD_MS / 1000.0)

    def _sim_tick_loop(self):
        """Avanza la simulación a SIM_TICK_MS de resolución."""
        while not self._stop_event.is_set():
            self.sim.tick(SIM_TICK_MS)
            time.sleep(SIM_TICK_MS / 1000.0)

    def _event_loop(self):
        """Drena la cola de eventos del simulador y los publica al broker."""
        while not self._stop_event.is_set():
            if self._connected:
                for ev in self.sim.pop_events():
                    self._client.publish(self.topic_event, json.dumps(ev), qos=1)
            time.sleep(0.1)

    def run(self):
        """Arranca todos los hilos y bloquea hasta Ctrl+C."""
        self._reconnect_loop()

        t_tick      = threading.Thread(target=self._sim_tick_loop,  daemon=True, name="tick")
        t_telemetry = threading.Thread(target=self._telemetry_loop, daemon=True, name="telemetry")
        t_event     = threading.Thread(target=self._event_loop,     daemon=True, name="event")

        t_tick.start()
        t_telemetry.start()
        t_event.start()

        log.info(f"[{self._id}] Simulador corriendo. Ctrl+C para detener.")
        log.info(f"[{self._id}] Telemetría → {self.topic_telemetry}  ({TELEMETRY_PERIOD_MS}ms)")
        log.info(f"[{self._id}] Comandos   ← {self.topic_cmd}")
        log.info(f"[{self._id}] Ack        → {self.topic_cmd_ack}")
        log.info(f"[{self._id}] Eventos    → {self.topic_event}")
        log.info(f"[{self._id}] Fallas sim ← {self.topic_sim_fault}")

        try:
            while True:
                time.sleep(5)
                if not self._connected:
                    self._reconnect_loop()
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
        description="Simulador de Bomba de Infusión a Jeringa (pico2w_syringe_pump)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  # Simulador básico contra broker local
  python pump_simulator.py

  # ID personalizado y broker remoto
  python pump_simulator.py --broker 192.168.1.100 --id bj-002

  # Jeringas de distintos volúmenes (diámetro Ø según BD Plastipak)
  python pump_simulator.py --diam 14.50 --cap 10.0   # jeringa 10 mL
  python pump_simulator.py --diam 19.05 --cap 20.0   # jeringa 20 mL (default)
  python pump_simulator.py --diam 26.70 --cap 50.0   # jeringa 50 mL

  # Tres bombas en paralelo (abrir tres terminales)
  python pump_simulator.py --id bj-001 &
  python pump_simulator.py --id bj-002 &
  python pump_simulator.py --id bj-003 &

Comandos rápidos con mosquitto_pub:
  mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "infuse,50.0,20.0"
  mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "fault_occ"
  mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "fault_clear"
  mosquitto_pub -h localhost -t bj/bj-deadbeef/cmd -m '{"cid":1,"cmd":"stop"}'
        """
    )
    p.add_argument("--broker", default="localhost",       help="IP/host del broker MQTT")
    p.add_argument("--port",   type=int, default=1883,    help="Puerto del broker MQTT")
    p.add_argument("--id",     default="bj-001",          help="Device ID de la bomba")
    p.add_argument("--diam",   type=float, default=19.05, help="Diámetro interno jeringa (mm)")
    p.add_argument("--cap",    type=float, default=20.0,  help="Capacidad máx. jeringa (mL)")
    p.add_argument("--debug",  action="store_true",       help="Log nivel DEBUG")
    return p


def main():
    args = build_parser().parse_args()
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    sim    = SyringePumpSimulator(args.id, args.diam, args.cap)
    client = PumpMQTTClient(sim, args.broker, args.port)
    client.run()


if __name__ == "__main__":
    main()
