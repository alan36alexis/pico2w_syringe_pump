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

### 3.2 Tópicos MQTT planos sin jerarquía de ID — ✅ IMPLEMENTADO (PR2)
Módulo `src/mqtt_topics.h/.c` creado (PR1). Suscripción a `topic_cmd()` activa.
PR2 completado: eliminados todos los literales `syringe_pump/*` del código.
- `task_pump_telemetry`: `"syringe_pump/telemetry"` → `topic_telemetry()` (QoS 0)
- `task_logger`: segundo switch de topics eliminado; `LOG_EVENT_FSM_STATE` → `topic_event()` QoS 1 con formato `{"type":"state","from":N,"to":N}`; demás logs son solo serial
- `task_system_monitor`: publish de `system_health` eliminado (no estaba en el contrato)
- `mqtt_client_publish_qos1()` agregada: propaga `msg.qos` a `mqtt_publish()` via `mqtt_msg_t.qos`

Estructura activa:
```
bj/{id}/status           QoS 1, retain=true  — ACTIVO (PR1)
bj/{id}/cmd              QoS 1, sin retain   — ACTIVO (PR1)
bj/{id}/telemetry        QoS 0, sin retain   — ACTIVO (PR2)
bj/{id}/event            QoS 1, sin retain   — ACTIVO (PR2, solo transiciones FSM por ahora)
bj/{id}/cmd/ack          QoS 1, sin retain   — ACTIVO (PR3)
```
No quedan literales `syringe_pump/*` en `src/`.

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

**ACK de comandos — ✅ IMPLEMENTADO (PR3):**
`mqtt_rx_task` parsea el envelope con `sscanf("{\"cid\":%d,\"cmd\":\"%255[^\"]\"}", ...)`.
`pump_hmi_parse_and_execute` cambió de `void` a `bool` (true=accepted, false=rejected).
Publica `{"cid":N,"result":"accepted"|"rejected"}` en `topic_cmd_ack()` QoS 1.
Fallback a string crudo si no hay envelope (backward compat con CLI/mosquitto_pub).

### 3.6 Buffer de payload — ✅ CORREGIDO (PR4)
**Bug:** `mqtt_msg_t.payload[128]` truncaba el JSON de telemetría (~162 chars en worst-case).
**Fix:** `#define MQTT_MAX_PAYLOAD 256` en `mqtt_client.h`; `mqtt_msg_t.payload[MQTT_MAX_PAYLOAD]`.
Todos los `strncpy`/`memcpy` ya usaban `sizeof(msg.payload)` → se actualizaron automáticamente.
`sscanf` format string actualizado de `%127[^\"]` → `%255[^\"]` para consistencia.

### 3.7 QoS diferenciado — ✅ IMPLEMENTADO (PR2/PR3)
- Telemetría → `mqtt_client_publish()` QoS 0
- Eventos FSM, ACKs, status → `mqtt_client_publish_qos1()` QoS 1
- Campo `uint8_t qos` en `mqtt_msg_t` propaga el QoS a lwIP `mqtt_publish()`

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

✅ PR2: Migrar telemetría y eventos a bj/{id}/...  [COMPLETADO]
     → core0_main.c: "syringe_pump/telemetry" → topic_telemetry()
     → core0_main.c: task_logger legacy switch eliminado; FSM → topic_event() QoS 1
     → mqtt_client.c: mqtt_msg_t.qos + mqtt_client_publish_qos1()
     → nodered/pump_simulator.py: ya usaba bj/{id}/... (sin cambios)
     → json_buf 256→512 y alarm events (type:alarm) quedan para PR futuro

✅ PR3: cmd_envelope + ACK  [COMPLETADO]
     → mqtt_rx_task: sscanf parsea {"cid":N,"cmd":"..."}, fallback a string crudo
     → pump_hmi_parse_and_execute: void→bool, retorna accepted/rejected
     → Publica {"cid":N,"result":"..."} en topic_cmd_ack() QoS 1
     → fn_mk_cmd en dashboard: CID incremental + JSON.stringify del envelope

✅ PR4: Fix crítico — payload buffer demasiado chico  [COMPLETADO]
     → #define MQTT_MAX_PAYLOAD 256 en mqtt_client.h
     → mqtt_msg_t.payload[128] → payload[MQTT_MAX_PAYLOAD]
     → strncpy/memcpy ya usaban sizeof() — se actualizaron automáticamente
     → sscanf format: %127[^\"] → %255[^\"] para consistencia
```

---

## 5. Entorno Docker de Desarrollo — ✅ IMPLEMENTADO

**Motivación:** Red corporativa sin acceso al hardware ni al broker de laboratorio.
Permite desarrollar y probar el dashboard Node-RED completamente offline.

### Estructura
```
docker/
├── docker-compose.yml          ← orquesta los 3 servicios
├── mosquitto/config/
│   └── mosquitto.conf          ← listener TCP 1883 + WebSocket 9001, anónimo
├── simulator/
│   ├── Dockerfile              ← python:3.11-slim + paho-mqtt==1.6.1
│   └── requirements.txt
└── flows_bomba.json            ← flow Node-RED importable (ver sección 6)
nodered/
└── pump_simulator.py           ← simulador Python (montado read-only en el contenedor)
```

### Levantar el entorno
```bash
cd docker
docker compose up          # la primera vez descarga las imágenes (~1-2 min)
docker compose up -d       # en background
docker compose down        # detener y eliminar contenedores
```

### Accesos
| Servicio | URL / Puerto |
|---|---|
| Node-RED editor | http://localhost:1880 |
| Dashboard 2.0 | http://localhost:1880/dashboard |
| Broker MQTT TCP | localhost:1883 |
| Broker MQTT WebSocket | localhost:9001 |

### Dentro de Docker, los servicios se ven entre sí por hostname
- Simulador conecta al broker como `mosquitto:1883`
- Node-RED conecta al broker como `mosquitto:1883`
- Desde el host (para debug) usar `localhost:1883`

---

## 6. Simulador Python — ✅ ACTUALIZADO AL CONTRATO MQTT

**Archivo:** `nodered/pump_simulator.py`

### Estado actual (post sesión 2026-06-12)
- ✅ Tópicos actualizados a `bj/{device_id}/...` (contrato completo)
- ✅ Maneja envelope JSON del comando: `{"cid":N,"cmd":"..."}` con fallback a string crudo
- ✅ Publica ACK a `bj/{id}/cmd/ack`: `{"cid":N,"result":"accepted"/"rejected","reason":"..."}`
- ✅ Cola de eventos `pop_events()` + hilo `_event_loop` que publica a `bj/{id}/event`
- ✅ Eventos de transición de estado (`{"type":"state","from":N,"to":M}`)
- ✅ Eventos de alarma (`{"type":"alarm","code":"occ","level":2}`)
- ✅ `parse_and_execute()` retorna `("accepted"|"rejected", reason)` en vez de `None`

### Tópicos activos
```
bj/{id}/telemetry   ← publica JSON cada 2 s (QoS 0)
bj/{id}/cmd         ← suscribe comandos con envelope JSON (QoS 1)
bj/{id}/cmd/ack     ← publica ACK correlacionado (QoS 1)
bj/{id}/event       ← publica transiciones de estado y alarmas (QoS 1)
bj/{id}/status      ← online/offline con retain (QoS 1)
bj/{id}/sim_fault   ← inyección de fallas exclusiva del simulador (QoS 0)
```

### ID del simulador en Docker
El contenedor arranca con `--id bj-deadbeef`. Para cambiar el device ID:
- Editar la línea `command:` en `docker/docker-compose.yml`

### Comandos via mosquitto_pub (desde el host)
```bash
# Ver toda la actividad del simulador
mosquitto_sub -h localhost -t "bj/+/#" -v

# Iniciar infusión (canal sim)
mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "infuse,50.0,20.0"
mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "bolus,5.0,200.0"
mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "fault_occ"
mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "fault_bubble"
mosquitto_pub -h localhost -t bj/bj-deadbeef/sim_fault -m "fault_clear"

# Comandos reales (con envelope JSON del contrato)
mosquitto_pub -h localhost -t bj/bj-deadbeef/cmd -m '{"cid":1,"cmd":"stop"}'
mosquitto_pub -h localhost -t bj/bj-deadbeef/cmd -m '{"cid":2,"cmd":"fsm_reset"}'
mosquitto_pub -h localhost -t bj/bj-deadbeef/cmd -m '{"cid":3,"cmd":"fsm_occ_rel"}'

# Verificar ACK
mosquitto_sub -h localhost -t "bj/bj-deadbeef/cmd/ack" -C 1
```

### Uso local (sin Docker)
```bash
pip install paho-mqtt==1.6.1
python nodered/pump_simulator.py --broker localhost --id bj-001
python nodered/pump_simulator.py --diam 14.50 --cap 10.0  # jeringa 10 mL
python nodered/pump_simulator.py --diam 26.70 --cap 50.0  # jeringa 50 mL
```

---

## 7. Dashboard Node-RED — Estado actual

### Arquitectura de capas
```
[Bombas / Simulador]
        ↓  MQTT pub/sub
[Broker Mosquitto]
        ↓
[Node-RED — servidor central]
  ├── Ingesta          → parseo y validación de JSON entrante
  ├── Comandos         → publicación cmd + espera ack (fn_mk_cmd)
  ├── Estado global    → contexto por bomba en flow context (TODO)
  ├── Motor de alarmas → notificaciones + tabla de eventos
  ├── Persistencia     → SQLite / histórico (TODO)
  ├── Push             → Web Push API → PWA del personal (TODO)
  └── Páginas UI       → Dashboard 2.0 (FlowFuse)
        ↓
[Navegador / PWA]
```

### Flow inicial — ✅ IMPLEMENTADO (`docker/flows_bomba.json`)

Importar en Node-RED: Menú (≡) → Import → seleccionar `docker/flows_bomba.json` → Deploy

**Prerequisito:** instalar Dashboard 2.0 primero:
Menú → Manage Palette → Install → buscar `@flowfuse/node-red-dashboard` → Install

#### Estructura del flow
```
MQTT in (bj/+/telemetry) → fn_telem [3 out] → gauge caudal
                                             → templates: estado, infusión, alarmas
                                             → chart presión

MQTT in (bj/+/status)   → fn_status         → template dispositivo
MQTT in (bj/+/event)    → fn_event [2 out]  → tabla eventos
                                             → ui-notification (alarmas)
MQTT in (bj/+/cmd/ack)  → fn_ack            → ui-text ACK

btn_stop/pause/...       → fn_mk_cmd         → MQTT out (bj/bj-deadbeef/cmd)
btn_sim_infuse/occ/...   → fn_fault_topic    → MQTT out (bj/bj-deadbeef/sim_fault)
```

#### Páginas del flow
| Página | Widgets |
|---|---|
| Monitor | Dispositivo (online/offline + ID), Estado FSM (color por estado), Caudal (gauge), Infusión (vol_inf, vol_tgt, t_ela, t_rem), Presión (chart tiempo real), Alarmas (badges OCC/NEAR/END/BUB/EMP/ERR) |
| Control | Botones: STOP, STOP IMM, PAUSAR, REANUDAR, RESET FSM, LIB. OCLUSIÓN — Sim: Infundir, Oclusión, Burbuja, Limpiar — ACK display + tabla de eventos |

#### IDs hardcodeados a cambiar si se usa otro device_id
- `fn_mk_cmd` → `msg.topic = 'bj/bj-deadbeef/cmd'`
- `fn_fault_topic` → `msg.topic = 'bj/bj-deadbeef/sim_fault'`
- Los MQTT in usan wildcard `bj/+/...` → no necesitan cambio

### Páginas pendientes de implementar
1. **Overview / Sala** — grid de tarjetas para N bombas (multi-device)
2. **Panel de alarmas** — lista priorizada, silenciado, histórico
3. **Datalog** — descarga de histórico (requiere `node-red-node-sqlite`)
4. **Config** — umbral de oclusión, perfil de jeringa

### Paquetes Node-RED adicionales (pendientes)
```
@flowfuse/node-red-dashboard  ← ✅ instalar antes de importar el flow
node-red-node-sqlite          ← persistencia (TODO)
node-red-contrib-web-push     ← notificaciones PWA (TODO)
```

---

## 8. Contrato MQTT — MQTT_CONTRACT.md

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

## 9. Próximos pasos en orden de prioridad

### ✅ Completado
- [x] Crear `MQTT_CONTRACT.md` en la raíz del repo
- [x] PR1: `device_id` en `SystemConfig_t` + `mqtt_topics.h/.c` + LWT/status + fix RX topic bug
- [x] Entorno Docker: mosquitto + simulador + Node-RED (`docker/docker-compose.yml`)
- [x] Simulador actualizado al contrato completo: tópicos `bj/{id}/...`, envelope CMD, ACK, eventos
- [x] Flow inicial Node-RED: Monitor (telemetría, estado, alarmas) + Control (botones) importable
- [x] PR2: migrar telemetría/eventos a `bj/{id}/...`, QoS diferenciado por tipo
- [x] PR3: `cmd_envelope` + ACK correlacionado en `bj/{id}/cmd/ack`
- [x] PR4: `MQTT_MAX_PAYLOAD=256` — fix crítico de truncamiento de telemetría

### Inmediato — Testear el flow con el simulador Docker
1. `cd docker && docker compose up`
2. En Node-RED: instalar `@flowfuse/node-red-dashboard` (Manage Palette)
3. Importar `docker/flows_bomba.json` → Deploy
4. Abrir http://localhost:1880/dashboard
5. Pulsar "▶ Infundir" → verificar que el gauge de caudal y la telemetría se actualicen
6. Pulsar "⚠ Oclusión" → verificar notificación y badge OCC en rojo
7. Pulsar "STOP" → verificar ACK en la tabla de eventos (debe aparecer `#N: accepted`)

---

## 10. Notas técnicas importantes

### Sobre el manejo de memoria en el firmware
- Heap configurado con `heap_3.c` (malloc estándar de stdlib). `xPortGetFreeHeapSize()`
  no disponible sin `mallinfo`. Monitorear stack HWM por tarea en `task_system_monitor`.
- El buffer de telemetría está en `core0_main.c` como `char json_buf[256]` — suficiente
  para el contrato actual. `MQTT_MAX_PAYLOAD=256` en la cola es consistente con este tamaño.

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

## 11. Posibilidades de mejora para sesiones futuras

Esta sección recoge ideas priorizadas por impacto. No son deuda técnica sino oportunidades.

### Firmware — alta prioridad

**A. Alarm events explícitos por MQTT**
- `LOG_EVENT_PRESSURE_ALERT` (y futuros) deben publicar en `topic_event()` con
  `{"type":"alarm","code":"pres","level":2}` además de actualizar `alm.occ` en telemetría.
- Archivos: `src/core0_main.c` (task_logger, switch LOG_EVENT_*) + `src/crosscore_logger.h`.
- Impacto: el dashboard puede disparar alarma visual/sonora en tiempo real sin esperar al ciclo de 2 s.

**B. ACK con reason field**
- Cuando `handled && !ok` en `pump_hmi_parse_and_execute`, incluir el estado FSM actual
  en el ACK: `{"cid":N,"result":"rejected","reason":"invalid_state","state":3}`.
- Cambio de 3 líneas en `src/mqtt_client.c` (pasar `pump_hmi_get_fsm_state()` al snprintf).
- Impacto: el dashboard puede mostrar "Rechazado: la bomba está en estado PURGA" en lugar de solo "rejected".

**C. Watchdog multi-thread**
- FreeRTOS task watchdog via event group: cada tarea setea su bit en un `EventGroupHandle_t`
  cada ciclo. Una tarea watchdog verifica que todos los bits se hayan seteado dentro de N ms.
- Si una tarea se bloquea, activa `NVIC_SystemReset()` y publica offline antes de resetear.
- Archivos nuevos: `src/watchdog_task.c/.h`.

**D. `cmd_parse_and_execute` return value**
- Actualmente retorna `void` → `pump_hmi_parse_and_execute` asume `true` para comandos de bajo nivel.
- Cambiar a `bool` en `src/crosscore_cmd.h/.c` para propagar el resultado real al ACK.

### Firmware — mediano plazo

**E. RTC / NTP + timestamps en eventos**
- El firmware no tiene reloj real; los timestamps de eventos los genera Node-RED (hora del broker).
- Opción liviana: sincronizar SNTP una vez al arrancar y adjuntar `"ts":epoch_ms` en cada
  publish de `topic_event()`. SDK lwIP incluye `lwip/apps/sntp.h`.
- Archivos: `src/core0_main.c`, `src/mqtt_client.c`.

**F. Persistencia de logs en flash (littlefs)**
- `LOG_EVENT_*` se pierden si el firmware resetea. Usar littlefs sobre la flash interna
  del RP2350 para guardar últimos N eventos con timestamp.
- Librería: https://github.com/littlefs-project/littlefs (port RP2040 ya existe).

**G. Reporte de Stack HWM por MQTT**
- `task_system_monitor` ya imprime el HWM por serial; podría publicarlo periódicamente
  en `bj/{id}/telemetry` como campo `"stk_min"` o en un tópico de debug separado.
- Útil para detectar stack overflows antes de que ocurran en producción.

### Dashboard Node-RED — alta prioridad

**H. Página Overview multi-dispositivo**
- Un `ui-template` con Vue que itere `context.global.get('devices')` (Map keyed by device_id)
  y muestre una tarjeta por bomba con estado, caudal y alarmas activas.
- El nodo MQTT activo ya usa wildcard `bj/+/...`; solo falta agregar el device al Map en `fn_telem`.

**I. Alarm history con persistencia**
- `node-red-node-sqlite` para guardar cada evento en una tabla `events(ts, device_id, type, detail)`.
- Un nodo `ui-table` en la página de alarmas muestra el histórico con filtros.
- Export a CSV via endpoint HTTP en Node-RED.

**J. Notificaciones PWA**
- `node-red-contrib-web-push`: suscribir el browser y enviar push cuando `type=alarm` llega
  a `fn_event`. Funciona offline si el dashboard está instalado como PWA.

### Dashboard Node-RED — mediano plazo

**K. `fn_fault_topic` dinámico**
- El nodo de faults del simulador tiene el topic hardcodeado a `bj/bj-deadbeef/sim_fault`.
- Debería usar `context.global.get('online_device_id')` igual que `fn_mk_cmd`.

**L. Histórico de telemetría (sparklines)**
- Guardar últimas N muestras de `rate` y `pres` en flow context para mostrar un sparkline
  en la tarjeta de la bomba sin necesitar SQLite.

### Fuera del alcance inmediato (ideas a largo plazo)

- **Control de lazo cerrado**: integrar el encoder óptico (branch `tft_integration`) en el
  loop de velocidad del motor PAP para compensar slippage.
- **OTA firmware update**: publicar binario por MQTT + bootloader custom en RP2350.
  La flash tiene 4 MB; hay espacio para imagen A/B.
- **Certificación IEC 60601-2-24**: documentar hazard analysis (FMEA), agregar tests de
  precisión de dosificación (±2% en todo el rango de caudal).
- **TFT menus**: branch `tft_integration` WIP — integrar `pump_hmi_execute()` como backend
  de los menús del display táctil para unificar todas las interfaces en un único dispatcher.

---

## 12. Recursos y referencias

| Recurso | URL / Ubicación |
|---------|----------------|
| Repo firmware | https://github.com/alan36alexis/pico2w_syringe_pump (rama: tft_integration) |
| Simulador | `nodered/pump_simulator.py` |
| Flow Node-RED | `docker/flows_bomba.json` (importar en Node-RED) |
| Docker Compose | `docker/docker-compose.yml` |
| Pico SDK docs | https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf |
| lwIP MQTT API | `$PICO_SDK_PATH/lib/lwip/src/include/lwip/apps/mqtt.h` |
| Node-RED Dashboard 2.0 | https://dashboard.flowfuse.com/getting-started.html |
| IEC 60601-2-24 | Resumen en `Entradas_de_diseño.pdf` del proyecto |
| FreeRTOS | https://www.freertos.org/Documentation/RTOS_book.html |
