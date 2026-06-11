# Contrato MQTT — Bomba de Infusión a Jeringa IoT

Documento de referencia para el firmware (Pico W2) y el dashboard (Node-RED).
Todo mensaje MQTT del sistema debe conformar los esquemas de este documento.

---

## Jerarquía de tópicos

| Tópico | QoS | Retain | Período / Disparo | Dirección |
|--------|-----|--------|-------------------|-----------|
| `bj/{device_id}/telemetry` | 0 | No | 2000 ms | Firmware → Dashboard |
| `bj/{device_id}/event` | 1 | No | Por disparo | Firmware → Dashboard |
| `bj/{device_id}/status` | 1 | **Sí** | Conexión / LWT | Firmware / Broker → Dashboard |
| `bj/{device_id}/cmd` | 1 | No | Por disparo | Dashboard → Firmware |
| `bj/{device_id}/cmd/ack` | 1 | No | Por disparo | Firmware → Dashboard |

`{device_id}` tiene el formato `bj-XXXXXXXX` (8 hex chars derivados del ID único del chip RP2350).

---

## Esquemas JSON

### `bj/{device_id}/telemetry`

Publicado cada 2 s. QoS 0 — perder una muestra es aceptable; la siguiente reemplaza.

```json
{
  "vol_inf": 10.83,
  "vol_tgt": 20.00,
  "rate":    50.00,
  "t_ela_s": 780.0,
  "t_rem_h": 0.18,
  "pres":    12.2,
  "st":      1,
  "alm": {
    "occ":  0,
    "near": 0,
    "end":  0,
    "bub":  0,
    "emp":  0,
    "err":  0
  }
}
```

| Campo | Tipo | Unidad | Descripción |
|-------|------|--------|-------------|
| `vol_inf` | float | mL | Volumen infundido (±0.01 mL) |
| `vol_tgt` | float | mL | Volumen objetivo |
| `rate` | float | mL/h | Caudal actual |
| `t_ela_s` | float | s | Tiempo transcurrido (1 decimal) |
| `t_rem_h` | float | h | Tiempo restante estimado |
| `pres` | float | mmHg | Presión de línea (1 decimal) |
| `st` | int | — | Estado FSM: 0=STOPPED 1=CONT 2=BOLUS 3=PURGE 4=KVO 5=PAUSED 6=ALARM |
| `alm.occ` | 0/1 | — | Oclusión detectada |
| `alm.near` | 0/1 | — | Último 10% de infusión |
| `alm.end` | 0/1 | — | Fin de infusión |
| `alm.bub` | 0/1 | — | Burbuja en línea |
| `alm.emp` | 0/1 | — | Jeringa vacía |
| `alm.err` | 0/1 | — | Error de sistema |

---

### `bj/{device_id}/status`

LWT configurado en el broker; publicado también por el firmware al conectar. Retain = true.

**Online** (publicado por el firmware tras conexión exitosa):
```json
{"state": "online", "id": "bj-abc12345", "fw": "v1.0.0"}
```

**Offline** (publicado por el broker al vencer el keepalive — LWT):
```json
{"state": "offline", "id": "bj-abc12345"}
```

---

### `bj/{device_id}/event`

Publicado con QoS 1 ante cambios de estado o alarmas. No hay retain.

```json
{"type": "alarm",  "code": "occ",    "level": 2}
{"type": "state",  "from": 1,        "to": 6}
{"type": "info",   "msg": "kvo_start"}
```

| Campo `type` | Descripción |
|---|---|
| `alarm` | Alarma activada — `code` identifica el campo `alm.*`, `level` 0–3 |
| `state` | Transición de estado FSM |
| `info` | Evento informativo de texto libre |

---

### `bj/{device_id}/cmd`

Enviado por el dashboard con QoS 1. El campo `cid` (correlation ID) permite correlacionar el ACK.

```json
{"cid": 17, "cmd": "fsm_dispense,10000.0,450.0"}
```

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `cid` | int | ID de correlación (generado por el dashboard, devuelto en el ack) |
| `cmd` | string | Payload de comando — misma sintaxis que el parser CLI |

**Comandos disponibles:**

| Comando | Descripción |
|---------|-------------|
| `stop` | Detener motor suavemente |
| `stop_imm` | Parada inmediata |
| `fsm_dispense,<vol_um>,<vel_ums>` | Iniciar infusión |
| `fsm_home,<vel>` | Ir a home |
| `fsm_search,<vel>` | Buscar jeringa |
| `fsm_reset` | Reset FSM |
| `fsm_occ_rel` | Liberar oclusión |
| `fsm_calibrate` | Calibrar recorrido |
| `config_wifi,<ssid>,<pass>` | Cambiar credenciales WiFi |
| `config_mqtt,<ip>,<port>` | Cambiar broker MQTT |
| `config_save` | Guardar config en flash |
| `config_info` | Imprimir config actual (vía log) |

---

### `bj/{device_id}/cmd/ack`

Respuesta del firmware por QoS 1. El `cid` coincide con el del comando.

```json
{"cid": 17, "result": "accepted"}
{"cid": 17, "result": "rejected", "reason": "no_syringe"}
```

| `result` | Descripción |
|----------|-------------|
| `accepted` | Comando recibido y encolado para Core 1 |
| `rejected` | Comando inválido, estado incompatible, o cola llena |

---

## Diagrama de secuencia: Comando con ACK

```
Dashboard                    Broker                    Firmware
    │                           │                           │
    │── PUBLISH cmd QoS 1 ─────►│                           │
    │   {"cid":17,"cmd":"stop"} │── DELIVER ───────────────►│
    │                           │                           │── parsea cmd
    │                           │                           │── encola Core1
    │                           │◄── PUBLISH ack QoS 1 ─────│
    │◄── DELIVER ───────────────│  {"cid":17,"result":"accepted"}
    │                           │                           │
```

---

## Reglas de suscripción para Node-RED

| Tópico | QoS | Notas |
|--------|-----|-------|
| `bj/+/telemetry` | 0 | Stream — pérdida aceptable |
| `bj/+/event` | 1 | No perder alarmas |
| `bj/+/status` | 1 | Con retain para recuperar estado al reconectar |
| `bj/+/cmd/ack` | 1 | Correlacionar con `cid` del comando enviado |

Publicar a `bj/{id}/cmd` con QoS 1.

---

## Keepalive y detección de desconexión

- Keepalive configurado: **60 s**
- El broker detecta dispositivo caído tras ~90 s sin actividad (1.5× keepalive)
- La telemetría cada 2 s actúa como heartbeat implícito
- El topic `status` con retain permite conocer el último estado conocido al reconectar

---

## Versiones

| Campo | Valor |
|-------|-------|
| Firmware target | v1.0.0 |
| `SystemConfig_t` magic | `0xA1B2C3D5` |
| Esquema telemetría | v1 (compatible con `Pump_GetTelemetryJSON`) |
