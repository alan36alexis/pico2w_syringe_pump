# Contrato MQTT — Bomba de Infusión a Jeringa IoT (Arquitectura EDA)

Documento oficial de referencia para el firmware (Pico 2 W / RP2350), el simulador en Python y el dashboard (Node-RED).
Todo mensaje MQTT del sistema debe conformar los esquemas, tipos y políticas definidos en este documento.

> **Nota sobre el tópico legado:** El antiguo tópico monolítico `bj/{device_id}/telemetry` ha sido reemplazado formalmente por la arquitectura multi-rate EDA dividida en sensores, hardware en tiempo real, cinemática y sesión clínica.

---

## 1. Jerarquía de Tópicos

| Tópico | QoS | Retain | Período / Disparo | Dirección | Propósito |
|--------|:---:|:------:|:-----------------:|:---------:|-----------|
| `bj/{device_id}/telemetry/sensors`  | 0 | No | 500 ms (2 Hz) | Pico / Sim → Dashboard | Presión de línea y oclusión clínica |
| `bj/{device_id}/telemetry/realtime` | 0 | No | 500 ms (2 Hz) | Pico / Sim → Dashboard | Monitoreo bajo nivel de sensores, ADC, PCAL9555 y TMC2209 |
| `bj/{device_id}/telemetry/motion`   | 0 | No | 1000 ms (1 Hz) | Pico / Sim → Dashboard | Cinemática del carro, pulsos PIO y velocidad lineal |
| `bj/{device_id}/telemetry/session` | 0 | No | 2000 ms (0.5 Hz) | Pico / Sim → Dashboard | Volumen VTBI, volumen infundido y tiempo de sesión |
| `bj/{device_id}/event`             | 1 | No | Por evento | Pico / Sim → Dashboard | Transiciones de la FSM (Core 1) y alarmas IEC 60601-1-8 |
| `bj/{device_id}/status`            | 1 | **Sí** | Conexión / LWT | Pico / Broker → Dashboard | Presencia del dispositivo (`online` / `offline`) |
| `bj/{device_id}/cmd`               | 1 | No | Por disparo | Dashboard → Pico / Sim | Comandos de control clínico y configuración con `cid` |
| `bj/{device_id}/cmd/ack`           | 1 | No | Por disparo | Pico / Sim → Dashboard | Acuses de recibo vinculados por `cid` (`accepted` / `rejected`) |
| `bj/{device_id}/sim_fault`         | 0 | No | Por disparo | Dashboard / Test → Sim | Inyección de fallas físicas y sobreescrituras en simulación |

`{device_id}` tiene el formato canónico `bj-XXXXXXXX` (8 caracteres hexadecimales derivados del identificador de hardware único del chip RP2350 o prefijo de simulación como `bj-deadbeef`).

---

## 2. Esquemas JSON de Telemetría (Multi-rate EDA)

### A. Sensores de Presión: `bj/{device_id}/telemetry/sensors`
Publicado cada **500 ms** (2 Hz) con QoS 0. Proporciona datos de alta frecuencia para gráficos de presión de línea y detección visual inmediata de aumentos de resistencia.

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

### B. Telemetría de Hardware en Tiempo Real: `bj/{device_id}/telemetry/realtime`
Publicado cada **500 ms** (2 Hz) con QoS 0. Expone el estado eléctrico y físico directo de los sensores, finales de carrera, canales del convertidor ADC, entradas del expansor I2C PCAL9555 y pines de potencia del driver TMC2209.

```json
{
  "lsw": {
    "start": 1,
    "end": 0
  },
  "encoder": {
    "count": 14500,
    "position_mm": 14.50
  },
  "adc": {
    "frc_sens_v": 0.735,
    "frc_sens_raw": 911,
    "bub_sens_v": 3.105,
    "bub_sens_raw": 3852,
    "vin_sens_v": 15.58,
    "vin_sens_raw": 1757
  },
  "pcal9555": {
    "sw_play_stop": 1,
    "sw_bolus": 1,
    "sw_detect_h": 0,
    "chrg_status": 0,
    "batt_install": 0
  },
  "motor": {
    "ena": 1,
    "dir": 1,
    "dir_str": "FORWARD",
    "stallguard": 240
  }
}
```

#### Descripción Detallada de Campos:

| Objeto / Campo | Tipo | Unidad / Rango | Descripción de Hardware |
|----------------|------|----------------|--------------------------|
| **`lsw.start`** | int (0/1) | 0 = libre, 1 = activo | Final de carrera inicio / retracción / Home (`LIMIT_SW_START_PIN`, GP10, Pull-up interno) |
| **`lsw.end`** | int (0/1) | 0 = libre, 1 = activo | Final de carrera avance máximo / EOT (`LIMIT_SW_END_PIN`, GP11, Pull-up interno) |
| **`encoder.count`** | int | pulsos | Pulsos acumulados en cuadratura (máquina de estados PIO GP6/GP7) |
| **`encoder.position_mm`** | float | mm | Desplazamiento lineal del émbolo respecto a origen (2 decimales) |
| **`adc.frc_sens_v`** | float | 0.0 – 3.3 V | Tensión en canal ADC0 (`FRC_SENS`, GP26). Sensor piezo/celda de carga |
| **`adc.frc_sens_raw`** | int | 0 – 4095 | Lectura cruda de 12 bits de ADC0 |
| **`adc.bub_sens_v`** | float | 0.0 – 3.3 V | Tensión en canal ADC1 (`BUB_SENS`, GP27). **Sensor Óptico de Burbuja**: ~3.1V continuo, <1.5V aire |
| **`adc.bub_sens_raw`** | int | 0 – 4095 | Lectura cruda de 12 bits de ADC1 |
| **`adc.vin_sens_v`** | float | 10.0 – 26.0 V | Tensión real de entrada del cargador / batería Li-Ion 4S calculada tras divisor 11:1 |
| **`adc.vin_sens_raw`** | int | 0 – 4095 | Lectura cruda de 12 bits en pin ADC2 (`VIN_SENS`, GP28). $V_{\text{pin}} = V_{\text{IN}} / 11$ |
| **`pcal9555.sw_play_stop`** | int (0/1) | 0 = pulsado, 1 = reposo | P0_0: Pulsador de Arranque / Pausa |
| **`pcal9555.sw_bolus`** | int (0/1) | 0 = pulsado, 1 = reposo | P0_1: Pulsador de Bolo / Purga manual |
| **`pcal9555.sw_detect_h`** | int (0/1) | 0 = ON, 1 = OFF | P0_2: **Switch principal ON / OFF** del equipo (`0 = Encendido`, `1 = Apagado`) |
| **`pcal9555.chrg_status`** | int (0/1) | 0 = cargando, 1 = standby | P1_1: Indicador de estado del cargador de litio (Open-drain activo bajo) |
| **`pcal9555.batt_install`** | int (0/1) | 0 = instalada, 1 = desconectada | P1_2: Detección de presencia física del pack de batería |
| **`motor.ena`** | int (0/1) | 0 = libre, 1 = habilitado | Señal de habilitación hacia driver TMC2209 (`MOTOR_ENA_PIN`, GP8) |
| **`motor.dir`** | int (0/1) | 1 = avance, 0 = retroceso | Señal de dirección hacia TMC2209 (`MOTOR_DIR_PIN`, GP2) |
| **`motor.dir_str`** | string | `"FORWARD"`, `"REVERSE"`, `"IDLE"` | Dirección física en formato legible |
| **`motor.stallguard`** | int | 0 – 1023 | Registro de esfuerzo mecánico `SG_RESULT` del TMC2209 (< 60 indica bloqueo/stall) |

#### Calibración y Modelado Matemático de `VIN_SENS`:
El circuito cuenta con un divisor resistivo de precisión:
* Resistencia en serie ($R_1$): $100\text{ k}\Omega \pm 1\%$
* Resistencia a tierra ($R_2$): $10\text{ k}\Omega \pm 1\%$
* **Relación de Atenuación:**
  $$\text{Ratio} = \frac{R_2}{R_1 + R_2} = \frac{10}{100 + 10} = \frac{1}{11} \implies V_{\text{ADC}} = \frac{V_{\text{IN}}}{11}$$
* **Fórmula de Conversión en Firmware / Simulador:**
  $$V_{\text{ADC}} = \frac{\text{adc\_raw}}{4095.0} \times 3.3\text{ V}$$
  $$V_{\text{IN}} = V_{\text{ADC}} \times 11.0 = \frac{\text{adc\_raw} \times 36.3\text{ V}}{4095.0}$$
* **Niveles Operativos para Pack Li-Ion 4S:**
  * **$V_{\text{IN}} \ge 16.8\text{ V}$:** Batería cargada al 100% (4.2V / celda) o cargador de pared conectado.
  * **$12.0\text{ V} \le V_{\text{IN}} < 16.8\text{ V}$:** Rango óptimo de descarga ($0\% \text{ a } 100\%$).
  * **$V_{\text{IN}} < 12.0\text{ V}$:** Batería descargada críticamente ($< 3.0\text{V}$ / celda) $\rightarrow$ Advertencia en dashboard y preparación de apagado seguro.

---

### C. Cinemática y Motor: `bj/{device_id}/telemetry/motion`
Publicado cada **1000 ms** (1 Hz) con QoS 0. Informa la cinemática de infusión para monitoreo de estabilidad mecánica.

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
| `encoder_count` | int | pulsos | Conteo acumulado de cuadratura por hardware (PIO) |
| `position_mm` | float | mm | Desplazamiento lineal del émbolo (2 decimales) |
| `speed_ums` | float | µm/s | Velocidad lineal instantánea medida |
| `progress_pct` | float | % | Porcentaje completado respecto al objetivo de sesión (0.0 a 100.0) |

---

### D. Sesión Clínica e Infusión: `bj/{device_id}/telemetry/session`
Publicado cada **2000 ms** (0.5 Hz) con QoS 0. Estado consolidado de la terapia para registro histórico hospitalario.

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
| `target_ml` | float | mL | Volumen total a infundir programado (VTBI) |
| `rate_ml_h` | float | mL/h | Caudal configurado |
| `elapsed_s` | int | s | Tiempo transcurrido acumulado de infusión activa |
| `fsm_state` | int | — | Estado actual de la FSM de Core 1 (0 a 20) |

---

## 3. Eventos y Alarmas: `bj/{device_id}/event`

Publicado con **QoS 1** ante cambios discretos de estado de la FSM o disparo de alarmas clínicas. No utiliza flag retain.

### A. Transición de Estado FSM:
```json
{"type": "state", "from": 3, "to": 6}
```
* `from`: ID del estado anterior (`Core1State_t`).
* `to`: ID del estado nuevo (`Core1State_t`).

### B. Disparo de Alarmas Clínicas (Norma IEC 60601-1-8):
```json
{"type": "alarm", "code": "occ", "level": 2}
```
* `code`: Código normalizado de la alarma:
  - `occ`: Oclusión hidráulica en línea de infusión.
  - `bub`: Detección de burbuja de aire en línea por sensor óptico.
  - `eot`: Fin de recorrido mecánico alcanzado / jeringa vacía.
  - `drv`: Falla del driver de potencia TMC2209 (térmica o sobrecorriente).
  - `bat`: Batería baja / crítica (< 12.0 V).
  - `mains`: Pérdida de alimentación externa de cargador.
* `level`: Severidad clínica:
  - `1`: Baja prioridad / advertencia informativa.
  - `2`: Media prioridad (pausa y requiere atención del operador).
  - `3`: Alta prioridad (parada obligatoria instantánea del motor por seguridad).

### C. Eventos Informativos:
```json
{"type": "info", "msg": "dispense_finished"}
```

---

## 4. Estados FSM de Core 1 (`Core1State_t`)

El sistema cuenta con 21 estados físicos y de control en el procesador de control de movimiento (Core 1):

| ID | Enumeración C (`Core1State_t`) | Etiqueta UI | Descripción de Comportamiento |
|:--:|--------------------------------|-------------|-------------------------------|
| `0` | `ST_UNHOMED` | ESPERANDO (UNHOMED) | Dispositivo encendido sin referencia física de posición |
| `1` | `ST_HOMING` | HOMING | Carro retrocediendo hacia sensor de inicio (`LSW_START`) |
| `2` | `ST_RELEASING_LSW_START` | REL_LSW_INI | Despegue suave del microswitch inicial para buscar borde |
| `3` | `ST_READY_AT_HOME` | LISTO EN HOME | Referenciado en origen mecánico, listo para operar |
| `4` | `ST_SEARCHING_SYRINGE` | BUSCANDO JERINGA | Avance lento buscando contacto con el émbolo |
| `5` | `ST_SYRINGE_ENGAGED` | JERINGA ACOPLADA | Contacto detectado y confirmado por celda de carga |
| `6` | `ST_DISPENSING` | DISPENSANDO | Infusión continua o administración de bolo en curso |
| `7` | `ST_DISPENSE_COMPLETED` | FIN DE DISPENSADO | Objetivo de volumen alcanzado exitosamente |
| `8` | `ST_SET_NEW_DISPENSE` | NUEVO DISPENSADO | Espera de nuevos parámetros de dosis |
| `9` | `ST_SEARCHING_EOT` | BUSCANDO EOT | Movimiento hacia límite de final de carrera (`LSW_END`) |
| `10` | `ST_RELEASING_LSW_END` | REL_LSW_FIN | Despegue del microswitch final |
| `11` | `ST_END_OF_TRAVEL` | FIN DE RECORRIDO | Límite físico máximo alcanzado (jeringa totalmente vacía) |
| `12` | `ST_FAULT` | FALLA MOTOR | Error crítico de hardware, burbuja de aire o bloqueo |
| `13` | `ST_OCCLUSION_STOPPING` | OCLUSIÓN FRENANDO | Frenado inmediato ante sobrepresión detectada |
| `14` | `ST_OCCLUSION_RELEASE` | LIBERANDO OCLUSIÓN | Retroceso anti-bolus para aliviar presión acumulada |
| `15` | `ST_OCCLUSION_PAUSED` | PAUSA POR OCLUSIÓN | Pausa segura con presión aliviada en espera de reanudación |
| `16` | `ST_MANUAL_OVERRIDE` | MANUAL OVERRIDE | Control por joystick o pasos manuales de servicio |
| `17` | `ST_CALIB_SEEK_START` | CALIB INI (BUSCA 0) | Calibración de recorrido hacia inicio |
| `18` | `ST_CALIB_SEEK_END` | CALIB FIN (BUSCA MAX) | Calibración hacia fin y conteo de pasos totales |
| `19` | `ST_BRAKING_LSW_START` | FRENANDO LSW INI | Frenado por impacto involuntario de switch inicial |
| `20` | `ST_BRAKING_LSW_END` | FRENANDO LSW FIN | Frenado por impacto involuntario de switch final |

---

## 5. Control y Comandos: `bj/{device_id}/cmd` y `ack`

Enviado por el dashboard con **QoS 1**. Cada comando incluye un Correlation ID (`cid`) para vincular la respuesta unívoca en `ack`.

### A. Solicitud de Comando con Envoltorio JSON:
```json
{"cid": 17, "cmd": "fsm_dispense,30000.0,500.0"}
```
*(Para compatibilidad con terminales manuales, se admiten también comandos en texto plano como `"stop_imm"`).*

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
| `fsm_home` | `[<vel_um_s>]` | Iniciar rutina de homing hacia origen |
| `fsm_search` | `[<vel_um_s>]` | Buscar acople con émbolo de jeringa |
| `fsm_dispense` | `<vol_um>,<vel_ums>` | Iniciar dispensado con volumen y velocidad |
| `fsm_cont` | Ninguno | Continuar infusión |
| `fsm_occ_rel` | Ninguno | Liberar oclusión y aliviar presión hidráulica |
| `fsm_resume` | Ninguno | Reanudar infusión preservando el caudal programado tras oclusión |
| `fsm_reset` | Ninguno | Restablecer acumuladores, limpiar fallas y volver a `ST_READY_AT_HOME` |
| `fsm_calibrate` | `[<vel_move>,<vel_seek>]` | Iniciar calibración automática de longitud total |
| `fsm_search_eot`| Ninguno | Búsqueda controlada de fin de carrera |
| `config_thresholds` | `<v_syr>,<v_occ>,<v_rel>` | Configurar umbrales ADC de fuerza (V) |
| `set_occ_thresh` | `<volts>` | Configurar umbral de oclusión (V) |
| `config_antibolus`| `<auto\|manual>` | Habilitar o deshabilitar retroceso anti-bolus |
| `config_wifi` | `<ssid>,<password>` | Configurar credenciales WiFi en caliente |
| `config_mqtt` | `<ip>,<port>` | Configurar broker MQTT |
| `config_save` | Ninguno | Persistir parámetros en memoria Flash |

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

---

## 7. Inyección de Fallas y Simulación de Hardware: `bj/{device_id}/sim_fault`

Publicado con **QoS 0** durante pruebas unitarias, de integración o validación clínica desde consola o dashboard hacia el simulador / banco de pruebas HIL (Hardware-in-the-Loop).

### Comandos de Inyección Soportados:

| Comando | Formato | Efecto en la Telemetría y Máquina de Estados |
|---------|---------|----------------------------------------------|
| `fault_occ` | `fault_occ` | Provoca un pico súbito de presión (> 2.5V, > 300 mmHg) disparando la secuencia de oclusión (`ST_OCCLUSION_STOPPING` $\rightarrow$ `ST_OCCLUSION_RELEASE` $\rightarrow$ `ST_OCCLUSION_PAUSED`). |
| `fault_bubble` | `fault_bubble` | Hace caer el voltaje del sensor óptico `BUB_SENS` a < 0.5V, emitiendo alarma `{"type":"alarm","code":"bub","level":2}` y pasando a `ST_FAULT`. |
| `fault_clear` | `fault_clear` | Restablece las presiones normales, limpia las alarmas activas y cancela todos los overrides de sensores. |
| `sim_vin,<volts>` | `sim_vin,11.5` | Sobreescribe la lectura de batería/cargador en voltios reales. Permite verificar advertencias de batería baja (<12V) o carga completa (16.8V). |
| `sim_pcal,<pin>,<val>` | `sim_pcal,sw_detect_h,1` | Sobreescribe una entrada del expansor PCAL9555 (`sw_play_stop`, `sw_bolus`, `sw_detect_h`, `chrg_status`, `batt_install`). Permite probar el switch ON/OFF principal. |
| `sim_lsw,<pin>,<val>` | `sim_lsw,start,1` | Sobreescribe los finales de carrera físicos (`start` o `end`) con 0 (libre) o 1 (activado). |

---

## 8. Políticas de Latencia, Dimensionamiento de Búferes y Prevención de Bufferbloat

Para garantizar una experiencia de monitoreo verdaderamente en tiempo real en el dashboard sin retardos percibidos al manipular señales analógicas (por ejemplo, potenciómetros en GP26/GP27) o entradas digitales, el sistema implementa las siguientes directivas obligatorias:

### A. Dimensión de Búferes de Transporte (lwIP)
* **`MQTT_OUTPUT_RINGBUF_SIZE`:** Configurado estrictamente en **$\ge 1024$ bytes** en `lwipopts.h` (el valor por defecto de lwIP es 256 B). Esto evita el descarte silencioso (`ERR_MEM`) del paquete JSON consolidado de `realtime` (~310 B + encabezados).
* **`MQTT_MAX_PAYLOAD`:** Fijado en **384 bytes** en la cola de salida del microcontrolador.

### B. Prevención de Bufferbloat (Anti-Stale Queuing)
* En telemetría periódica QoS 0 (`realtime`, `sensors`, `motion`), **el dato nuevo siempre prevalece sobre el dato viejo**.
* Si la cola de transmisión (`mqtt_tx_queue`) se encuentra llena (por ejemplo, ante congestión de radio Wi-Fi o contención de red), la rutina de encolado descarta automáticamente el paquete más antiguo almacenado e inserta la muestra actual fresca (*drop-oldest*).
* Esto garantiza que la latencia punta a punta (medición física $\rightarrow$ broker MQTT $\rightarrow$ widget en Node-RED) nunca supere el período de refresco configurado (500 ms), eliminando el fenómeno de acumulación o "desembalse diferido" de muestras viejas.

### C. Desacoplamiento Temporal en Core 1 (Control de Movimiento y ADC)
* La telemetría de Core 1 (`EV_HW_CORE1_STATUS`) se rige por marcas de tiempo absolutas (`absolute_time_t` con `make_timeout_time_ms(500)`), garantizando una frecuencia de muestreo de 2 Hz exacta e independiente del número de iteraciones o demoras dinámicas del ciclo de control.
* En ausencia de hardware de driver PAP / TMC2209 físico (pruebas sobre Pico 2 W pelada), el canal de polling UART se desactiva de forma automática en el arranque tras detectar fallo de comunicación (`check_uart == 0`), previniendo timeouts bloqueantes que degraden la cadencia de la FSM.

### D. Valores por Defecto en Banco de Pruebas (Hardware Mocks)
* **`vin_sens`:** En placas sin el circuito divisor resistivo conectado en ADC2 (GP28), se reporta un valor nominal seguro de batería de 4 celdas Li-Ion ($15.58\text{ V}$, cuenta raw 1757) para permitir la evaluación clínica en dashboard.
* **`pcal9555`:** Administrado mediante la tarea modular `mock_pcal9555` en Core 0 cuando no se detecta el chip I2C0 en dirección `0x20`, manteniendo las señales nominales de reposo y permitiendo inyecciones vía `sim_fault`.
