# Contrato MQTT — Bomba de Infusión a Jeringa IoT (Arquitectura EDA)

Documento oficial de referencia para el firmware (Pico 2 W / RP2350), el simulador en Python y el dashboard (Node-RED).
Todo mensaje MQTT del sistema debe conformar los esquemas y políticas definidos en este documento.

> **Nota sobre el tópico legado:** El antiguo tópico monolítico `bj/{device_id}/telemetry` ha sido reemplazado formalmente por la arquitectura multi-rate EDA dividida en sensores, cinemática y sesión clínica.

---

## 1. Jerarquía de Tópicos

| Tópico | QoS | Retain | Período / Disparo | Dirección |
|--------|-----|--------|-------------------|-----------|
| `bj/{device_id}/telemetry/sensors` | 0 | No | 500 ms | Firmware / Sim → Dashboard |
| `bj/{device_id}/telemetry/motion`  | 0 | No | 1000 ms | Firmware / Sim → Dashboard |
| `bj/{device_id}/telemetry/session` | 0 | No | 2000 ms | Firmware / Sim → Dashboard |
| `bj/{device_id}/event`             | 1 | No | Por disparo | Firmware / Sim → Dashboard |
| `bj/{device_id}/status`            | 1 | **Sí** | Conexión / LWT | Firmware / Broker → Dashboard |
| `bj/{device_id}/cmd`               | 1 | No | Por disparo | Dashboard → Firmware / Sim |
| `bj/{device_id}/cmd/ack`           | 1 | No | Por disparo | Firmware / Sim → Dashboard |

`{device_id}` tiene el formato `bj-XXXXXXXX` (8 caracteres hexadecimales derivados del identificador único del chip RP2350 o prefijo de simulación).

---

## 2. Esquemas JSON de Telemetría (Multi-rate EDA)

### A. Sensores de Presión: `bj/{device_id}/telemetry/sensors`
Publicado cada **500 ms** (2 Hz) con QoS 0. Proporciona datos de alta frecuencia para gráficos de presión de línea y detección visual de oclusiones.

```json
{
  "pressure_psi": 0.235,
  "pressure_mmhg": 12.2
}
```

| Campo | Tipo | Unidad | Descripción |
|-------|------|--------|-------------|
| `pressure_psi` | float | psi | Presión hidráulica en línea (3 decimales) |
| `pressure_mmhg` | float | mmHg | Presión hidráulica en línea (1 decimal) |

---

### B. Cinemática y Motor: `bj/{device_id}/telemetry/motion`
Publicado cada **1000 ms** (1 Hz) con QoS 0. Informa la posición mecánica del empujador, velocidad y conteo de encoder en cuadratura.

```json
{
  "encoder_count": 15200,
  "position_mm": 15.20,
  "speed_ums": 450.0,
  "progress_pct": 50.0
}
```

| Campo | Tipo | Unidad | Descripción |
|-------|------|--------|-------------|
| `encoder_count` | int | pulsos | Conteo acumulado del encoder por hardware (PIO) |
| `position_mm` | float | mm | Desplazamiento lineal del émbolo (2 decimales) |
| `speed_ums` | float | µm/s | Velocidad lineal instantánea medida |
| `progress_pct` | float | % | Porcentaje completado respecto al objetivo (0.0 a 100.0) |

---

### C. Sesión Clínica e Infusión: `bj/{device_id}/telemetry/session`
Publicado cada **2000 ms** (0.5 Hz) con QoS 0. Estado de la infusión, volúmenes acumulados y estado de la FSM.

```json
{
  "session_id": 1,
  "infused_ml": 5.25,
  "target_ml": 10.00,
  "rate_ml_h": 50.0,
  "elapsed_s": 378,
  "fsm_state": 6
}
```

| Campo | Tipo | Unidad | Descripción |
|-------|------|--------|-------------|
| `session_id` | int | — | Identificador secuencial de la sesión de infusión |
| `infused_ml` | float | mL | Volumen real infundido al paciente |
| `target_ml` | float | mL | Volumen objetivo programado (VTBI) |
| `rate_ml_h` | float | mL/h | Caudal configurado |
| `elapsed_s` | int | s | Tiempo transcurrido de infusión activa |
| `fsm_state` | int | — | Estado actual de la FSM de Core 1 (0 a 20) |

---

## 3. Eventos y Alarmas: `bj/{device_id}/event`

Publicado con **QoS 1** ante cambios de estado de la máquina de estados o disparo de alarmas. No utiliza flag retain.

### A. Transición de Estado FSM:
```json
{"type": "state", "from": 3, "to": 6}
```
* `from`: ID del estado anterior (`Core1State_t`).
* `to`: ID del estado nuevo (`Core1State_t`).

### B. Disparo de Alarmas (IEC 60601-1-8):
```json
{"type": "alarm", "code": "occ", "level": 2}
```
* `code`: Código de la alarma:
  - `occ`: Oclusión hidráulica en línea.
  - `eot`: Fin de recorrido mecánico / jeringa vacía.
  - `drv`: Falla del driver de potencia TMC2209.
  - `bat`: Batería baja.
  - `mains`: Pérdida de alimentación de red (220V).
  - `bub`: Detección de burbuja de aire en línea.
* `level`: Severidad clínica:
  - `1`: Baja prioridad / advertencia.
  - `2`: Media prioridad (requiere atención del operador).
  - `3`: Alta prioridad (parada obligatoria inmediata del motor).

### C. Eventos Informativos:
```json
{"type": "info", "msg": "dispense_finished"}
```

---

## 4. Estados FSM de Core 1 (`Core1State_t`)

El sistema cuenta con 21 estados físicos y de control en Core 1:

| ID | Enumeración C (`Core1State_t`) | Etiqueta UI | Descripción |
|:--:|--------------------------------|-------------|-------------|
| `0` | `ST_UNHOMED` | ESPERANDO (UNHOMED) | Dispositivo encendido sin referencia de home |
| `1` | `ST_HOMING` | HOMING | Carro retrocediendo hacia sensor de inicio |
| `2` | `ST_RELEASING_LSW_START` | REL_LSW_INI | Despegue suave del microswitch inicial |
| `3` | `ST_READY_AT_HOME` | LISTO EN HOME | Referenciado en origen, listo para operar |
| `4` | `ST_SEARCHING_SYRINGE` | BUSCANDO JERINGA | Avance lento buscando contacto con el émbolo |
| `5` | `ST_SYRINGE_ENGAGED` | JERINGA ACOPLADA | Contacto detectado por celda de carga |
| `6` | `ST_DISPENSING` | DISPENSANDO | Infusión continua o bolo en curso |
| `7` | `ST_DISPENSE_COMPLETED` | FIN DE DISPENSADO | Objetivo de volumen alcanzado |
| `8` | `ST_SET_NEW_DISPENSE` | NUEVO DISPENSADO | Espera de nuevos parámetros de dosis |
| `9` | `ST_SEARCHING_EOT` | BUSCANDO EOT | Movimiento hacia límite de final de carrera |
| `10` | `ST_RELEASING_LSW_END` | REL_LSW_FIN | Despegue del microswitch final |
| `11` | `ST_END_OF_TRAVEL` | FIN DE RECORRIDO | Límite físico máximo alcanzado |
| `12` | `ST_FAULT` | FALLA MOTOR | Error crítico de hardware o bloqueo |
| `13` | `ST_OCCLUSION_STOPPING` | OCLUSIÓN FRENANDO | Frenado inmediato ante sobrepresión |
| `14` | `ST_OCCLUSION_RELEASE` | LIBERANDO OCLUSIÓN | Retroceso anti-bolus para aliviar presión |
| `15` | `ST_OCCLUSION_PAUSED` | PAUSA POR OCLUSIÓN | En pausa con presión aliviada |
| `16` | `ST_MANUAL_OVERRIDE` | MANUAL OVERRIDE | Control por joystick o pasos manuales |
| `17` | `ST_CALIB_SEEK_START` | CALIB INI (BUSCA 0) | Calibración de recorrido hacia inicio |
| `18` | `ST_CALIB_SEEK_END` | CALIB FIN (BUSCA MAX) | Calibración hacia fin y conteo de pasos |
| `19` | `ST_BRAKING_LSW_START` | FRENANDO LSW INI | Frenado por golpe de switch inicial |
| `20` | `ST_BRAKING_LSW_END` | FRENANDO LSW FIN | Frenado por golpe de switch final |

---

## 5. Control y Comandos: `bj/{device_id}/cmd` y `ack`

Enviado por el dashboard con **QoS 1**. Cada comando incluye un Correlation ID (`cid`) para vincular la respuesta unívoca en `ack`.

### A. Solicitud de Comando:
```json
{"cid": 17, "cmd": "fsm_dispense,30000.0,500.0"}
```

### B. Acuse de Recibo (ACK): `bj/{device_id}/cmd/ack`
```json
{"cid": 17, "result": "accepted"}
```
o en caso de rechazo:
```json
{"cid": 17, "result": "rejected", "reason": "invalid_state"}
```

### C. Catálogo de Comandos Admitidos:

| Comando | Parámetros | Descripción |
|---------|------------|-------------|
| `stop` | Ninguno | Detención suave del motor |
| `stop_imm` | Ninguno | Parada de emergencia inmediata (invariante de seguridad) |
| `fsm_home` | `[<vel_um_s>]` | Iniciar rutina de homing |
| `fsm_search` | `[<vel_um_s>]` | Buscar acople con émbolo de jeringa |
| `fsm_dispense` | `<vol_um>,<vel_ums>` | Iniciar dispensado con distancia y velocidad |
| `fsm_cont` | Ninguno | Continuar infusión |
| `fsm_occ_rel` | Ninguno | Liberar oclusión y aliviar presión hidráulica |
| `fsm_resume` | Ninguno | Reanudar infusión tras oclusión liberada |
| `fsm_reset` | Ninguno | Restablecer fallas y poner FSM en estado inicial |
| `fsm_calibrate` | `[<vel_move>,<vel_seek>]` | Iniciar calibración automática de longitud total |
| `fsm_search_eot`| Ninguno | Búsqueda controlada de fin de carrera |
| `config_thresholds` | `<v_syr>,<v_occ>,<v_rel>` | Configurar umbrales ADC (V) |
| `set_occ_thresh` | `<volts>` | Configurar umbral de oclusión (V) |
| `config_antibolus`| `<auto\|manual>` | Habilitar o deshabilitar retroceso anti-bolus |
| `config_wifi` | `<ssid>,<password>` | Configurar red WiFi en caliente |
| `config_mqtt` | `<ip>,<port>` | Configurar broker MQTT |
| `config_save` | Ninguno | Persistir configuración en memoria Flash |

---

## 6. Presencia y Keepalive (`bj/{device_id}/status`)

Configurado con **Retain = true** y **QoS 1**.

* **Online** (emitido al conectar exitosamente):
```json
{"state": "online", "id": "bj-XXXXXXXX", "fw": "v1.0.0-eda"}
```

* **Offline** (emitido automáticamente por el broker mediante Last Will and Testament al vencer el keepalive de 60 s):
```json
{"state": "offline", "id": "bj-XXXXXXXX"}
```
