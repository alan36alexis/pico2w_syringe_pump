# Contexto del Proyecto — Bomba de Infusión a Jeringa IoT
## Handoff para Claude Code / VSCode

> **Propósito de este documento:** Transferir todo el contexto de diseño, decisiones arquitectónicas
> y tareas pendientes discutidas en la sesión de arquitectura previa. Leer completo antes de tocar
> cualquier archivo del repositorio.

---

## 1. Proyecto

**Nombre:** Bomba de Infusión a Jeringa con Monitoreo y Control IoT
**Institución:** UTN FRA — Proyecto Final, Ingeniería Electrónica
**Equipo:** Beherens Braian, Fernandez Pablo, Gallo Alejandro, Velazquez Alan
**Norma de referencia:** IEC 60601-2-24 (bombas de infusión)

### Descripción en una línea
Sistema embebido sobre Raspberry Pi Pico W2 que controla una bomba jeringa de precisión
con motor PAP (NEMA 17 + caja reductora + TMC2209), detección de oclusión por presión,
detección de burbujas, y conectividad IoT vía MQTT/WiFi con dashboard en Node-RED.

### Stack tecnológico definido
| Capa | Tecnología | Estado |
|------|-----------|--------|
| MCU | RP2350 (Pico W2), FreeRTOS v11, Pico SDK v2.2.0 | En desarrollo |
| Driver motor | TMC2209 UART, perfil trapezoidal, semi-lazo cerrado con encoder | Implementado |
| Broker MQTT | Mosquitto (local / RPi / VM) | A configurar |
| Dashboard | Node-RED + Dashboard 2.0 (FlowFuse) | **Próximo trabajo** |
| Simulador | Python + paho-mqtt | **Entregado — ver sección 5** |
| Persistencia | SQLite vía node-red-node-sqlite | Pendiente |
| Notificaciones | PWA Web Push | Pendiente |

---

## 2. Repositorio

```
URL:    https://github.com/alan36alexis/pico2w_syringe_pump
Rama activa: tft_integration
```

### Estructura relevante del repo (rama tft_integration)
```
src/
  core0_main.c          ← Tareas FreeRTOS: telemetría, CLI, reconexión, system_health
  core1_main.c          ← Control de motor baremetal (hard real-time, sin RTOS)
  syringe_pump_api.c/h  ← Modelo clínico: PumpContext_t, Pump_Tick(), Pump_GetTelemetryJSON()
  mqtt_client.c/h       ← Cliente MQTT sobre lwIP, colas TX/RX FreeRTOS-safe
  crosscore_cmd.c/h     ← Parser de comandos + queue Pico SDK entre cores
  crosscore_logger.c/h  ← Logger asíncrono Core1→Core0 via queue no-bloqueante
  config_manager.c/h    ← Persistencia en flash: SystemConfig_t con CRC + magic
  system_config.h       ← Constantes de hardware y clínicas (#define centralizados)
lib/
  tft_touch_module/     ← Submódulo git (repo AleeGallo/ProyectoFinal) — Display TFT 4"
```

### Arquitectura dual-core (IMPORTANTE para entender el firmware)
```
Core 0  ──── FreeRTOS ────  WiFi/MQTT · CLI · Telemetría · Logger · Config
                │
         queue_t (Pico SDK, non-blocking, thread-safe)
                │
Core 1  ──── Baremetal ───  Motor PAP · Encoder · Sensor fuerza · FSM motion
```
**Razón de diseño:** `printf()` desde Core 1 bloquea spinlocks compartidos con el driver CYW43.
Todo logging de Core 1 va por `crosscore_logger` (queue no-bloqueante) hacia Core 0.

---

## 3. Problemas identificados en el firmware (estado por PR)

### 3.1 Sin identidad de dispositivo — ✅ IMPLEMENTADO (PR1)
`device_id[17]` agregado a `SystemConfig_t` en `config_manager.h`.
`config_manager.c` lo popula con `pico_get_unique_board_id_string()` en el bloque de defaults.
`CONFIG_MAGIC_WORD` bumpeado a `0xA1B2C3D5` → flash vieja se detecta y se regeneran defaults.
`mqtt_client.c` usa `g_sys_config.device_id` como `ci.client_id`.

### 3.2 Tópicos MQTT planos sin jerarquía de ID — ⚠️ PARCIALMENTE IMPLEMENTADO
Módulo `src/mqtt_topics.h/.c` creado (PR1). Suscripción a `topic_cmd()` activa.
**Pendiente (PR2):** migrar `"syringe_pump/telemetry"` en `core0_main.c` a `topic_telemetry()`
y los 8 literales `"syringe_pump/log/*"` del `task_logger`.

Estructura activa:
```
bj/{id}/status           QoS 1, retain=true  — ACTIVO (PR1)
bj/{id}/cmd              QoS 1, sin retain   — ACTIVO (PR1)
bj/{id}/telemetry        QoS 0, sin retain   — PENDIENTE PR2
bj/{id}/event            QoS 1, sin retain   — PENDIENTE PR2
bj/{id}/cmd/ack          QoS 1, sin retain   — PENDIENTE PR3
```
Tópicos legacy `syringe_pump/*` siguen activos hasta PR2.

### 3.3 Sin Last Will Testament (LWT) — ✅ IMPLEMENTADO (PR1)
`mqtt_client.c` configura `ci.will_topic/will_msg/will_qos/will_retain` antes de conectar.
Al conectar exitosamente publica `{"state":"online","id":"bj-XXXXXXXX","fw":"v1.0.0"}` con retain=1.
Verificar: `mosquitto_sub -t "bj/+/status" -v` debe mostrar el online al arrancar.

### 3.4 Sin módulo de tópicos — ✅ IMPLEMENTADO (PR1)
`src/mqtt_topics.h` y `src/mqtt_topics.c` creados. `topics_init()` + 5 accessors `topic_*()`.
Agregado a `CMakeLists.txt`. Llamado desde `mqtt_client_task()` antes del loop de conexión.

### 3.5 Sin ACK de comandos + bug en recepción de tópico
**Bug topic RX — ✅ CORREGIDO (PR1):**
`mqtt_client.c` ahora guarda el topic en `static char s_rx_topic[64]` dentro de
`mqtt_incoming_publish_cb`. El `mqtt_incoming_data_cb` lo copia a `rx_msg.topic` y
también respeta el flag `MQTT_DATA_FLAG_LAST` para ignorar payloads fragmentados.

**ACK de comandos — PENDIENTE (PR3):** El dashboard no sabe si un comando fue aceptado.
Solución: capa `cmd_envelope` sobre `cmd_parse_and_execute()` sin modificarlo:
```
Dashboard → bj/{id}/cmd  →  {"cid":17,"cmd":"fsm_dispense,10000.0,450.0"}
Firmware  → bj/{id}/cmd/ack  →  {"cid":17,"result":"accepted"}
```
El `cid` (correlation ID) permite request/response sobre pub/sub puro.

### 3.6 Buffer de payload de 128 bytes insuficiente
**Problema:** `char payload[128]` se queda corto cuando el JSON de telemetría crezca.
El truncamiento es un bug silencioso: JSON inválido que el dashboard descarta sin aviso.

**Solución:** `#define MQTT_MAX_PAYLOAD 512` en el header del contrato.

### 3.7 QoS 0 fijo en todos los publish
**Regla a implementar:**
- Telemetría → QoS 0 (perder una muestra de 2 s es aceptable, la próxima reemplaza)
- Eventos/alarmas/ACKs → QoS 1 (no se repiten, no deben perderse)

---

## 4. Plan de PRs para el firmware (secuencia recomendada)

Cada PR es funcional y testeable de forma independiente. **No mezclar.**

```
✅ MQTT_CONTRACT.md creado en raíz del repo (pre-condición cumplida)

✅ PR1: device_id + mqtt_topics + status/LWT  [COMPLETADO]
     → device_id en SystemConfig_t, magic 0xA1B2C3D5
     → src/mqtt_topics.h/.c con topics_init() + 5 accessors
     → LWT configurado, online publicado con retain al conectar
     → Suscripción a topic_cmd() QoS 1
     → Fix s_rx_topic en callbacks RX + MQTT_DATA_FLAG_LAST

⏳ PR2: Migrar telemetría y eventos a bj/{id}/...  [SIGUIENTE]
     → core0_main.c: "syringe_pump/telemetry" → topic_telemetry()
     → core0_main.c: 8 literales "syringe_pump/log/*" → topic_event() o subtópicos
     → core0_main.c: json_buf 256 → 512
     → Agregar tools/simulator/pump_simulator.py al repo
     → Actualizar simulador con nuevos tópicos bj/{id}/*

⬜ PR3: cmd_envelope + ACK
     → Parsear {"cid":N,"cmd":"..."} en mqtt_rx_task
     → Publicar {"cid":N,"result":"accepted"} en topic_cmd_ack() QoS 1

⬜ PR4: Buffers MQTT_MAX_PAYLOAD=512 + QoS diferenciado
     → #define MQTT_MAX_PAYLOAD 512 en mqtt_client.h
     → mqtt_client_publish_reliable() para QoS 1 (eventos, ACKs, status)
     → Telemetría → QoS 0, resto → QoS 1
```

---

## 5. Simulador Python — YA IMPLEMENTADO

**Archivo:** `pump_simulator.py` (disponible para agregar al repo)

### Qué hace exactamente
- Porta `syringe_pump_api.c` línea por línea a Python
- Genera JSON **byte-exacto** al `Pump_GetTelemetryJSON()` del firmware:
  ```json
  {"vol_inf":0.83,"vol_tgt":10.00,"rate":50.00,"t_ela_s":60.0,
   "t_rem_h":0.18,"pres":12.2,"st":1,
   "alm":{"occ":0,"near":0,"end":0,"bub":0,"emp":0,"err":0}}
  ```
- Implementa todos los estados de `PumpState_t` con transiciones correctas
- Implementa `Pump_CheckAlarms()` completo (oclusión, último 10%, fin→KVO, vacía)
- Acepta todos los comandos de `cmd_parse_and_execute()`
- Modelo de presión físico: ruido basal + rampa de oclusión a 100 mmHg/s + decaimiento exponencial
- LWT configurado: broker publica `offline` si el proceso muere

### Instalación y uso
```bash
pip install paho-mqtt

# Bomba simple contra broker local
python pump_simulator.py

# Con ID y broker remoto
python pump_simulator.py --broker 192.168.1.100 --id bj-001

# Jeringas según diámetro BD Plastipak
python pump_simulator.py --diam 14.50 --cap 10.0   # 10 mL
python pump_simulator.py --diam 19.05 --cap 20.0   # 20 mL (default)
python pump_simulator.py --diam 26.70 --cap 50.0   # 50 mL

# Tres bombas en paralelo (multi-instancia)
python pump_simulator.py --id bj-001 &
python pump_simulator.py --id bj-002 &
python pump_simulator.py --id bj-003 &
```

### Comandos via mosquitto_pub
```bash
# Inyectar escenarios de prueba (tópico exclusivo del simulador)
mosquitto_pub -t syringe_pump/sim_fault -m "infuse,50.0,20.0"    # arrancar infusión
mosquitto_pub -t syringe_pump/sim_fault -m "bolus,5.0,200.0"     # bolo 5mL a 200mL/h
mosquitto_pub -t syringe_pump/sim_fault -m "fault_occ"           # inyectar oclusión
mosquitto_pub -t syringe_pump/sim_fault -m "fault_bubble"        # alarma de burbuja
mosquitto_pub -t syringe_pump/sim_fault -m "fault_clear"         # limpiar fallas
mosquitto_pub -t syringe_pump/sim_fault -m "select_syringe,19.05,20.0"

# Comandos reales del firmware (mismo canal que el hardware)
mosquitto_pub -t syringe_pump/cmd -m "stop"
mosquitto_pub -t syringe_pump/cmd -m "fsm_reset"
mosquitto_pub -t syringe_pump/cmd -m "fsm_occ_rel"
```

### Tópicos actuales del simulador (actualizar en PR2)
```
syringe_pump/telemetry   ← publica JSON cada 2 s (= firmware actual)
syringe_pump/cmd         ← suscribe comandos reales
syringe_pump/status      ← online/offline con retain
syringe_pump/sim_fault   ← inyección de fallas (exclusivo simulador)
```

---

## 6. Dashboard Node-RED — Arquitectura definida

### Por qué Node-RED y no Python + Flask + WebSockets
- El browser no habla MQTT TCP nativo → siempre hay un bridge MQTT↔WebSocket
- Node-RED **es** ese bridge ya hecho + servidor web + UI + persistencia
- Python requeriría: paho + Flask/FastAPI + WebSockets + HTML/JS + SQLite + Web Push
- Para un proyecto donde el diferencial técnico está en el firmware (no en el web), minimizar el esfuerzo en dashboard es la decisión correcta

### Capas del dashboard en Node-RED
```
[Bombas / Simulador]
        ↓  MQTT pub/sub
[Broker Mosquitto]
        ↓
[Node-RED — servidor central]
  ├── Ingesta          → parseo y validación de JSON entrante
  ├── Comandos         → publicación cmd + espera ack
  ├── Estado global    → contexto por bomba en flow context
  ├── Motor de alarmas → priorización, deduplicación, silenciado
  ├── Persistencia     → SQLite / histórico
  ├── Push             → Web Push API → PWA del personal
  └── Páginas UI       → Dashboard 2.0 (FlowFuse)
        ↓
[Navegador / PWA]
```

### Páginas del dashboard a implementar
1. **Overview / Sala** — estado de todas las bombas en grid (tarjeta por bomba)
2. **Detalle bomba** — gráfico de caudal en tiempo real, presión, volumen, progreso
3. **Panel de alarmas** — lista priorizada, silenciado, histórico
4. **Control remoto** — envío de comandos con confirmación ACK
5. **Datalog** — descarga de histórico de infusión por bomba/paciente
6. **Config** — parámetros del sistema, umbral de oclusión, perfil de jeringa

### Nodos de Node-RED requeridos
```
npm install node-red-dashboard          ← UI Dashboard 2.0
npm install node-red-node-sqlite        ← persistencia
npm install node-red-contrib-web-push   ← notificaciones PWA
```

### Regla de QoS para flows Node-RED
- Suscribirse a `bj/+/telemetry` con QoS 0 (stream de datos)
- Suscribirse a `bj/+/event` con QoS 1 (alarmas, no perder)
- Suscribirse a `bj/+/status` con QoS 1 + `retain=true` (estado online/offline)
- Publicar a `bj/{id}/cmd` con QoS 1 (confirmar con ack correlacionado)

---

## 7. Contrato MQTT — A crear como MQTT_CONTRACT.md

Este es el primer entregable antes de cualquier código de dashboard o PR de firmware.

### Jerarquía de tópicos
```
bj/{device_id}/telemetry     QoS 0  retain 0   período: 2000 ms
bj/{device_id}/event         QoS 1  retain 0   por disparo
bj/{device_id}/status        QoS 1  retain 1   online/offline + LWT
bj/{device_id}/cmd           QoS 1  retain 0   dashboard → firmware
bj/{device_id}/cmd/ack       QoS 1  retain 0   firmware → dashboard
```

### Esquema JSON telemetry (portado de Pump_GetTelemetryJSON)
```json
{
  "vol_inf": 10.83,      // mL infundidos (float ±0.01)
  "vol_tgt": 20.00,      // mL objetivo (float)
  "rate":    50.00,      // mL/h actual (float)
  "t_ela_s": 780.0,      // segundos transcurridos (float, 1 decimal)
  "t_rem_h": 0.18,       // horas restantes estimadas (float)
  "pres":    12.2,       // presión línea mmHg (float, 1 decimal)
  "st":      1,          // PumpState: 0=STOPPED 1=CONT 2=BOLUS 3=PURGE 4=KVO 5=PAUSED 6=ALARM
  "alm": {
    "occ":   0,          // oclusión detectada
    "near":  0,          // último 10% de infusión
    "end":   0,          // fin de infusión
    "bub":   0,          // burbuja en línea
    "emp":   0,          // jeringa vacía
    "err":   0           // error de sistema
  }
}
```

### Esquema JSON status (LWT + online)
```json
{"state": "online",  "id": "bj-abc12345", "fw": "v1.0.0"}
{"state": "offline", "id": "bj-abc12345"}   ← publicado por broker via LWT
```

### Esquema JSON cmd / cmd-ack
```json
// Comando (dashboard → firmware)
{"cid": 17, "cmd": "fsm_dispense,10000.0,450.0"}

// ACK (firmware → dashboard)
{"cid": 17, "result": "accepted"}
{"cid": 17, "result": "rejected", "reason": "no_syringe"}
```

---

## 8. Próximos pasos en orden de prioridad

### ✅ Completado
- [x] Crear `MQTT_CONTRACT.md` en la raíz del repo
- [x] PR1: `device_id` en `SystemConfig_t` + `mqtt_topics.h/.c` + LWT/status + fix RX topic bug

### Inmediato — PR2 (siguiente sesión)
- [ ] `core0_main.c`: cambiar `"syringe_pump/telemetry"` → `topic_telemetry()` en `task_pump_telemetry`
- [ ] `core0_main.c`: cambiar los 8 literales `"syringe_pump/log/*"` del `task_logger` → tópicos `bj/{id}/...`
- [ ] `core0_main.c`: aumentar `json_buf` de 256 a 512 bytes
- [ ] Agregar `pump_simulator.py` al repo en `tools/simulator/` (ya implementado, solo copiar)
- [ ] Actualizar `pump_simulator.py` con tópicos `bj/{id}/*`

### Paralelo con PR2 — infraestructura dashboard
- [ ] Instalar Mosquitto en la máquina de desarrollo
- [ ] Instalar Node-RED con los 3 paquetes de la sección 6
- [ ] Crear primer flow: suscribir `bj/+/telemetry` + debug node + gauge básico
- [ ] Verificar que el gauge muestra datos del simulador Python

### Sprint siguiente
- [ ] PR3: `cmd_envelope` + ACK en `topic_cmd_ack()` (parsear `{"cid":N,"cmd":"..."}`)
- [ ] Panel de control remoto en dashboard con confirmación de ACK
- [ ] Implementar nodo de estado global por bomba en Node-RED (flow context)
- [ ] Motor de alarmas básico (switch por campo `alm.*`)
- [ ] Primera página de overview con estado de N bombas

### Después
- [ ] PR4: `MQTT_MAX_PAYLOAD=512` + QoS diferenciado en `mqtt_client_publish_reliable()`
- [ ] Persistencia SQLite + datalog descargable
- [ ] Notificaciones PWA

---

## 9. Notas técnicas importantes

### Sobre el manejo de memoria en el firmware
- Heap configurado con `heap_3.c` (malloc estándar de stdlib). `xPortGetFreeHeapSize()`
  no disponible sin `mallinfo`. Monitorear stack HWM por tarea en `task_system_monitor`.
- El buffer de telemetría está en `core0_main.c` como `char json_buf[256]` (aumentar a 512
  cuando se implemente el contrato extendido).

### Sobre el submódulo TFT
- `lib/tft_touch_module` es submódulo git apuntando a `AleeGallo/ProyectoFinal`
- Al clonar: `git clone --recurse-submodules`
- Al integrar cambios del submódulo: pinear el commit en el repo principal con
  `git submodule update --remote` + commit del pointer

### Sobre la kinemática (para el simulador y para entender el firmware)
```
Jeringa 20 mL (Ø 19.05 mm BD Plastipak):
  área_pistón = π × (19.05/2)² = 284.87 mm²
  50 mL/h = 50000 mm³/h = 13.89 mm³/s → velocidad = 13.89/284.87 = 0.0488 mm/s = 48.8 µm/s
  En 60 s a 50 mL/h → 0.833 mL infundidos (verificado en el test del simulador)
```

### Sobre el modelo de presión del simulador
```python
# Parámetros del PressureModel en pump_simulator.py
RAMP_UP_RATE_MMHG_S  = 100.0   # pendiente de oclusión (mmHg/s)
RAMP_DOWN_TAU_S      = 3.0     # constante de tiempo post-liberación
BASELINE_MEAN_MMHG   = 10.0    # basal en reposo
FLOW_COEFF_MMHG_MLH  = 0.05    # 50 mL/h ≈ +2.5 mmHg
```

---

## 10. Recursos y referencias

| Recurso | URL / Ubicación |
|---------|----------------|
| Repo firmware | https://github.com/alan36alexis/pico2w_syringe_pump (rama: tft_integration) |
| Simulador | `tools/simulator/pump_simulator.py` (agregar al repo) |
| Pico SDK docs | https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf |
| lwIP MQTT API | `$PICO_SDK_PATH/lib/lwip/src/include/lwip/apps/mqtt.h` |
| Node-RED Dashboard 2.0 | https://dashboard.flowfuse.com/getting-started.html |
| IEC 60601-2-24 | Resumen en `Entradas_de_diseño.pdf` del proyecto |
| FreeRTOS | https://www.freertos.org/Documentation/RTOS_book.html |
