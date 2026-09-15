# Contexto de Arquitectura — Bomba de Infusión a Jeringa IoT

> **Propósito:** Este documento es el punto de entrada para **cualquier asistente LLM**
> (Claude, GPT, Gemini, u otro) que trabaje sobre este repositorio. Resume la arquitectura,
> los flujos de datos y las **reglas invariantes** que deben respetarse en cada cambio,
> sin necesidad de leer todo el código.
>
> **Leer completo antes de tocar cualquier archivo.** La sección 9 (Reglas invariantes)
> es de cumplimiento obligatorio: un cambio que las viole se considera incorrecto aunque compile.

**Última actualización:** 2026-07-07 (rama `tft_integration`)

---

## 1. Proyecto

**Nombre:** Bomba de Infusión a Jeringa con Monitoreo y Control IoT
**Institución:** UTN FRA — Proyecto Final, Ingeniería Electrónica
**Equipo:** Beherens Braian, Fernandez Pablo, Gallo Alejandro, Velazquez Alan
**Normas de referencia:** IEC 60601-2-24 (bombas de infusión), IEC 62304 (software), IEC 60601-1-8 (alarmas)

Sistema embebido sobre Raspberry Pi Pico 2W (RP2350) que controla una bomba jeringa de
precisión con motor PAP (NEMA 17 + reductora + driver TMC2209), encoder de posición,
sensor de presión/fuerza para detección de oclusión, display TFT táctil, y conectividad
IoT vía MQTT/WiFi con dashboard en Node-RED.

| Capa | Tecnología |
|------|-----------|
| MCU | RP2350 (Pico 2W), dual-core: FreeRTOS v11 (Core 0) + baremetal (Core 1), Pico SDK v2.2.0 |
| Motor | TMC2209 por UART, pulsos por PIO, perfiles trapezoidales 2 segmentos por DMA |
| Encoder | Cuadratura por PIO (semi-lazo cerrado de posición) |
| HMI local | TFT 4" + touch (submódulo `lib/tft_touch_module`, LVGL) — gated por `ENABLE_TFT` |
| IoT | MQTT sobre lwIP (CYW43), broker Mosquitto, dashboard Node-RED + Dashboard 2.0 |
| Persistencia | Flash interna (sector raw con magic+CRC); littlefs planificado |
| Simulador | `nodered/pump_simulator.py` (Python + paho-mqtt), entorno Docker completo en `docker/` |

---

## 2. Layout del repositorio

```
src/                    ← .c del firmware
  main.c                ← entry: system_queues_init() → lanza Core 1 → arranca FreeRTOS
  core0_main.c          ← creación de tareas FreeRTOS, CLI, telemetría legacy, SysMon
  core1_main.c          ← loop baremetal: polling sensores/LSW, inyección de eventos, FSM
  fsm_table.c           ← FSM table-driven de Core 1 (GLOBAL[] + TRANSITIONS[] + policy)
  cmd_dispatcher.c      ← parser único de comandos string (CLI/MQTT/HMI)
  cmd_gate.c            ← validación FSM de acciones de bomba (único acceso a Core 1)
  crosscore_cmd.c       ← cola de comandos Core 0 → Core 1 (queue_t del SDK)
  crosscore_logger.c    ← filtro de categorías de log + helpers de emisión de Core 1
  event_broker.c        ← tarea broker: drena colas de eventos y hace fanout a consumers
  system_queues.c       ← definición de todas las colas de eventos
  serial_consumer.c     ← consumer: eventos → printf (tabla de lookup de strings)
  mqtt_consumer.c       ← consumer: eventos → publicaciones MQTT multi-rate
  hmi_consumer.c        ← consumer: eventos → ui_state / LVGL (no-op sin ENABLE_TFT)
  mqtt_client.c         ← cliente MQTT lwIP, colas TX/RX, LWT, envelope cmd/ack
  mqtt_topics.c         ← accessors de tópicos bj/{id}/... (sin literales en el código)
  config_manager.c      ← persistencia SystemConfig_t en flash (magic + CRC)
  syringe_pump_api.c    ← modelo clínico: PumpContext_t, conversión mL/h ↔ um/s
  closed_loop.c         ← corrección de posición por encoder durante dispensado
  ui_state.c / ui_events.c ← estado compartido y eventos de la UI TFT
headers/                ← TODOS los .h del firmware (no hay headers en src/)
  system_events.h       ← SystemEventID_t, DTOs, SystemEvent_t, CORE1_EMIT
  system_queues.h       ← colas + CORE0_EMIT
  system_config.h       ← constantes de hardware y clínicas (#define centralizados)
  ...(un .h por cada .c de src/)
lib/
  tmc2209/              ← driver TMC2209 (UART + PIO)
  honeywell/            ← sensor de presión SPI
  tft_touch_module/     ← submódulo git (AleeGallo/ProyectoFinal)
tools/fsm_test/         ← test golden de la FSM, compilable en host (sin Pico SDK)
docker/                 ← mosquitto + Node-RED + simulador (desarrollo offline)
nodered/                ← pump_simulator.py + flows
```

**Documentos de contrato (raíz del repo) — consultarlos antes de tocar el área correspondiente:**

| Documento | Área |
|---|---|
| `MQTT_CONTRACT.md` | Tópicos, esquemas JSON, QoS, envelope cmd/ack |
| `MEMORY_CONTRACT.md` | Layout de flash, SystemConfig_t, reglas de escritura |
| `UI_BJ_CONTRACT.md` | Integración TFT/touch/encoder/teclas en Core 0 |
| `TODO_notes.txt` | Backlog priorizado y decisiones descartadas |

---

## 3. Arquitectura dual-core (fundamento de todo el diseño)

```
Core 0 ── FreeRTOS ──  WiFi/MQTT · CLI · Event Broker · Consumers · Config · TFT
              │                                    ▲
   crosscore_cmd_queue (comandos ↓)     g_crosscore_event_q (eventos ↑)
              │         [ambas: queue_t del Pico SDK, lock-free, no bloqueantes]
              ▼                                    │
Core 1 ── Baremetal ──  FSM motion · Motor PAP/DMA · Encoder PIO · Sensor presión · LSW
```

**Razón de diseño:** Core 1 opera con plazos de microsegundos (perfiles de velocidad,
seguridad ante sobrepresión). `printf()` y cualquier primitiva que comparta spinlocks
con el driver CYW43 pueden bloquear Core 1 un tiempo impredecible. Por eso **Core 1
jamás imprime ni llama servicios de Core 0**: solo emite eventos por cola no-bloqueante
y recibe comandos por cola no-bloqueante.

---

## 4. Core 1 — FSM table-driven (`fsm_table.c`)

La FSM de movimiento **no es un switch**: son dos tablas de transiciones + una política
por defecto. Estados y eventos en `headers/core1_main.h` (`Core1State_t`, `Core1Event_t`).

```
fsm_dispatch(ctx, state, event):
  1. GLOBAL[]        — filas con from == ST_COUNT ("cualquier estado"). Prioridad máxima
                       (LSW hit, fault, stall, timeout). Se SALTEA si state == ST_FAULT.
  2. TRANSITIONS[]   — primera fila con (from == state, trigger == event, guard OK).
                       La action se ejecuta ANTES del cambio de estado.
  3. fsm_default_policy() — celda no cubierta → POL_IGNORE o POL_ILLEGAL (→ ST_FAULT).
                       No existe el "default: break" silencioso.
```

- **`FsmCtx_t`** (en `fsm_table.h`): todo el estado mutable que actions/guards pueden tocar.
  `core1_main.c` lo puebla antes de cada `fsm_dispatch()` (snapshot de encoder,
  `motor_is_moving`, payload del comando). Las actions **no acceden al hardware PIO
  directamente**: leen el ctx.
- **Estados principales:** UNHOMED → HOMING → RELEASING_LSW_START → READY_AT_HOME →
  SEARCHING_SYRINGE → SYRINGE_ENGAGED → DISPENSING → DISPENSE_COMPLETED; ramas de
  oclusión (OCCLUSION_STOPPING/RELEASE/PAUSED), calibración (CALIB_SEEK_START/END),
  frenado en LSW (BRAKING_LSW_START/END), SEARCHING_EOT/END_OF_TRAVEL, y ST_FAULT.
- **Capas de detección de falla (safety):**
  - *Layer 1* — completitud espacial: LSW físicos delimitan el recorrido.
  - *Layer 2* — stall por encoder: cuenta sin cambio con DMA activo → `iEV_ENCODER_STALL`.
  - *Layer 3* — deadline derivado: `deadline_ms = K × t_nominal + piso`; vencido → `iEV_TIMEOUT`.
  - **StallGuard del TMC2209 NO es mecanismo de seguridad** (poco confiable a baja
    velocidad); queda solo como telemetría diagnóstica opcional.
- **Dead-time de reversa en LSW:** tras frenar contra un final de carrera, el release
  en dirección opuesta se difiere `LSW_REVERSAL_DEAD_TIME_MS` (200 ms) vía
  `fsm_service_release_deadtime()`; se auto-cancela si la FSM sale del estado RELEASING.
- **Auditoría de cobertura (trazabilidad IEC 62304):** `fsm_audit_coverage()` recorre la
  matriz completa ST_COUNT × EV_COUNT. Toda celda debe quedar clasificada
  (VALID/IGNORE/ILLEGAL). **`SIN_CLASIFICAR == 0` es criterio de build** para cualquier
  commit que toque la tabla.
- **Referencia de posición:** el encoder se resetea SOLO en boot, `CMD_ENC_RESET` y hit
  de LSW_START. STOP no resetea el encoder. La calibración punta-a-punta persiste
  `calibrated_max_encoder_count` en flash y se recarga en boot.

---

## 5. Core 0 — FreeRTOS

Tareas creadas en `core0_main.c` (y por los `*_start()` de cada módulo):

| Tarea | Rol | Prioridad |
|---|---|---|
| `EvBroker` | Drena colas de eventos y hace fanout a consumers | 4 (la más alta) |
| `Init` | Config desde flash, restaura calibración, arranca red | 2 |
| `MQTT_Task` / `MQTT_Rx` | Conexión/keepalive MQTT · recepción y despacho de comandos | 2 |
| `CLI` | Consola serial → `cmd_dispatch_string()` | 1 |
| `SerialCons` / `MqttCons` / `HmiCons` | Consumers de eventos | 1 |
| `SysMon` | Stack high-water-marks, salud del sistema | 1 |
| `Blinky`, `WiFi_Keepalive` | Housekeeping | 1 |
| `UI_Input`, `TFT` | Solo con `ENABLE_TFT`: input táctil/teclas y render LVGL | 2–3 |

---

## 6. Sistema de eventos EDA pub-sub (logging y telemetría)

Toda notificación del sistema (sensores, FSM, red, alarmas, salud) viaja como
`SystemEvent_t` (**máx. 64 bytes**, `_Static_assert` lo garantiza) por colas hacia un
**broker central** que la replica a consumers independientes.

```
Core 1 ── CORE1_EMIT ──► g_crosscore_event_q (queue_t SDK, cap. 32) ──┐
                                                                      ├─► task_event_broker ──► fanout
Core 0 ── CORE0_EMIT ──► g_core0_event_q (FreeRTOS, cap. 24) ─────────┘        │
                                                                               ├─► g_serial_q → serial_consumer (printf)
                                                                               ├─► g_mqtt_q   → mqtt_consumer (publish multi-rate)
                                                                               └─► g_hmi_q    → hmi_consumer (TFT; NULL sin ENABLE_TFT)
```

- **IDs por dominio** (`headers/system_events.h`): TMC `0x01xx`, sensores `0x02xx`,
  motor `0x03xx`, red `0x04xx`, app/FSM `0x05xx`, **alarmas `0x06xx`**, energía `0x07xx`,
  sistema `0x08xx`. Payload = unión de DTOs tipados (`DtoMotion_t`, `DtoAlarm_t`, ...).
- **Sin strings por colas** — única excepción: `DtoDebugStr_t` (56 bytes) para
  `EV_DBG_STRING` de Core 1.
- **Timestamps:** Core 1 emite con `timestamp_ms = 0` y el broker lo sella al recibir;
  Core 0 sella al emitir (`CORE0_EMIT`).
- **El broker es un router uniforme:** no interpreta contenido ni prioridades. Los
  consumers filtran/priorizan por su cuenta (`EV_IS_ALARM(id)`).
- **Los eventos de alarma son notificaciones, NO señales de control.** La acción de
  seguridad (frenar motor, cambiar estado) ya ocurrió en Core 1 / FSM antes de emitir.
- **Emisión no-bloqueante siempre:** si la cola está llena, el evento se descarta en
  silencio (aceptable para logging; jamás bloquear al emisor).
- `broker_side_effects()` en `event_broker.c` es el único lugar para efectos colaterales
  de eventos (hoy: actualizar espejo FSM del cmd_gate, `Pump_UpdatePressure`, flag de
  calibración dirty).

---

## 7. Pipeline de comandos (una sola vía de entrada a Core 1)

```
CLI (task_cli) ─────────┐
MQTT (mqtt_client.c) ───┼──► cmd_dispatch_string(str, src, cid)   [cmd_dispatcher.c]
TFT/HMI (futuro/BLE) ───┘         │  parseo único + emite EV_APP_CMD_EXECUTED
                                  ▼
                        cmd_gate_execute(action, p1, p2)          [cmd_gate.c]
                                  │  valida contra espejo del estado FSM de Core 1
                                  ▼
                        cmd_send_*()  →  crosscore_cmd_queue      [crosscore_cmd.c]
                                  ▼
                        Core 1: pop → evento EV_CMD_* → fsm_dispatch()
```

Reglas del pipeline:
- `cmd_dispatch_string()` es el **único parser** de comandos string, para todas las
  interfaces. Retorna `{accepted, reason}` ("ok" | "invalid_state" | "bad_format" |
  "unknown_cmd" | "queue_full") — el llamador arma su ACK (MQTT usa el `cid`).
- `cmd_gate_execute()` es el **único punto que encola acciones de bomba** hacia Core 1.
  Mantiene un espejo del estado FSM (`s_fsm_state`) actualizado **exclusivamente** por
  el broker al recibir `EV_APP_FSM_STATE`.
- **STOP / STOP_IMM nunca se validan contra el estado FSM** (invariante de seguridad);
  se chequean primero en el dispatcher (`stop_imm` antes que `stop` por el prefijo).
- Comandos de bajo nivel (`home_start`, `nsteps`, fallback `target,vel`) y de
  configuración/red bypasean el gate — son para banco de pruebas, no para operación clínica.
- Para agregar una **nueva interfaz** (ej. BLE): crear su consumer/parser y terminar en
  `cmd_dispatch_string()` o `cmd_gate_execute()`. Nada más.
- Para agregar una **nueva acción FSM**: valor en `PumpAction_t` → case con validación en
  `cmd_gate_execute()` → `cmd_send_*()` en `crosscore_cmd.c/.h` → evento `EV_CMD_*` en
  `Core1Event_t` → filas en la tabla FSM → actualizar golden test y auditoría.

---

## 8. MQTT (resumen — contrato completo en `MQTT_CONTRACT.md`)

Identidad: `device_id` formato `bj-XXXXXXXX` derivado del chip ID, persistido en
`SystemConfig_t`. Tópicos solo vía accessors de `mqtt_topics.h` — **prohibido hardcodear
strings de tópicos**.

```
bj/{id}/status      QoS 1  retain 1   online/offline + LWT
bj/{id}/cmd         QoS 1             envelope {"cid":N,"cmd":"..."} (fallback string crudo)
bj/{id}/cmd/ack     QoS 1             {"cid":N,"result":"accepted"|"rejected"}
bj/{id}/telemetry   QoS 0             JSON clínico cada 2 s (legacy, task_pump_telemetry)
bj/{id}/event       QoS 1             transiciones FSM {"type":"state","from":N,"to":N}
                                      (eventos de alarma por MQTT: pendiente)
bj/{id}/telemetry/sensors   QoS 0     presión (500 ms)     ┐
bj/{id}/telemetry/motion    QoS 0     encoder/avance (1 s) ├ publicados por mqtt_consumer
bj/{id}/telemetry/session   QoS 0     infusión (2 s)       ┘
```

Payloads de comandos = los mismos strings de la CLI (ver tabla del README).
`MQTT_MAX_PAYLOAD = 256`. Publicaciones QoS 1 vía `mqtt_client_publish_qos1()`.

---

## 9. REGLAS INVARIANTES (obligatorias para cualquier LLM/desarrollador)

1. **Core 1 jamás llama `printf()`** ni primitivas bloqueantes compartidas con Core 0.
   Toda salida de Core 1 = `CORE1_EMIT(...)` (o `LOG_DEBUG` que lo envuelve).
2. **Toda comunicación entre cores va por las dos colas existentes** (`crosscore_cmd_queue`
   hacia Core 1, `g_crosscore_event_q` hacia Core 0). No crear otros canales ni variables
   compartidas sin sincronización explícita y justificada.
3. **Ningún módulo escribe en `crosscore_cmd_queue` directamente** salvo `crosscore_cmd.c`.
   Las acciones de bomba entran solo por `cmd_gate_execute()`; los comandos string solo
   por `cmd_dispatch_string()`.
4. **`cmd_gate_update_fsm_state()` se llama SOLO desde `event_broker.c`** — nunca desde
   consumers de interfaz.
5. **STOP siempre se acepta**: ningún cambio puede condicionar STOP/STOP_IMM al estado FSM.
6. **La FSM se modifica solo por tablas** (`GLOBAL[]`, `TRANSITIONS[]`,
   `fsm_default_policy()`), nunca reintroduciendo switches. Tras cualquier cambio:
   `fsm_audit_coverage()` debe dar `SIN_CLASIFICAR == 0` y el golden test
   (`tools/fsm_test`) debe actualizarse y pasar.
7. **Las actions/guards de la FSM no tocan hardware directamente**: leen/escriben
   `FsmCtx_t`; `core1_main.c` es quien puebla el ctx y toca PIO/DMA.
8. **`sizeof(SystemEvent_t) ≤ 64`** — al agregar un DTO, respetar el `_Static_assert`.
   Nada de `char[]` en DTOs (excepción existente: `DtoDebugStr_t`).
9. **Emisión de eventos siempre no-bloqueante** (drop silencioso si la cola está llena).
   El broker no interpreta eventos; efectos colaterales solo en `broker_side_effects()`.
10. **`system_queues_init()` debe ejecutarse antes de `multicore_launch_core1()`**
    (orden en `main.c` — no alterar).
11. **Escrituras a flash** (`config_manager_save`): solo con motor detenido, con
    multicore lockout + IRQs off. Layout según `MEMORY_CONTRACT.md` — no inventar offsets.
12. **Tópicos MQTT solo vía `mqtt_topics.h`**; esquemas JSON según `MQTT_CONTRACT.md`.
    No reintroducir tópicos legacy `syringe_pump/*`.
13. **Credenciales (SSID/pass WiFi) no se emiten por eventos ni MQTT** (solo
    `config_info` local por serial).
14. **Código TFT/HMI siempre gated por `ENABLE_TFT`**; los módulos deben ser no-op si
    `g_hmi_q == NULL`. Nada de lógica de display ni de protocolo de red en cmd_gate.
15. **Headers en `headers/`, fuentes en `src/`**; todo archivo nuevo se agrega a
    `CMakeLists.txt`. Constantes de hardware/clínicas centralizadas en `system_config.h`
    (no magic numbers en el código).
16. **StallGuard no se usa como mecanismo de seguridad** — la seguridad de movimiento es
    Layers 1–3 (sección 4). No "optimizar" eliminando esas capas.
17. **Git lo maneja el usuario**: no hacer commits, pushes ni cambios de rama salvo
    pedido explícito.
18. **Features descartadas — no reintroducir:** finales de carrera virtuales
    (`fsm_virtual_lsw`), perfiles S-curve, `task_logger` monolítico.

---

## 10. Testing

**Golden test de la FSM** (`tools/fsm_test/`): compila `fsm_table.c` en host (sin Pico
SDK, hardware mockeado) y verifica cada transición contra la tabla de referencia
`fsm_golden.h`. Corre en Windows/Linux/Mac:

```bash
cd tools/fsm_test
gcc -Wall -Wextra -o test_golden test_fsm_golden.c && ./test_golden
```

Obligatorio tras cualquier cambio en `fsm_table.c`, `core1_main.h` (estados/eventos) o
la lógica de guards. Al agregar estados/eventos: extender `fsm_golden.h` y la
clasificación de `fsm_default_policy()` en el mismo commit.

**Entorno de integración sin hardware:** `cd docker && docker compose up` levanta
Mosquitto + Node-RED + simulador Python (`bj-deadbeef`). Ver sección Docker del
`MQTT_CONTRACT.md` y `docker/flows_bomba.json`.

No hay CI configurado: el criterio de "verde" es build limpio del firmware + golden test
pasando + auditoría de cobertura sin celdas sin clasificar.

---

## 11. Estado actual y trabajo pendiente

- **Rama activa:** `tft_integration` (la integración TFT está en scaffold; el grueso del
  trabajo reciente fue la migración EDA y la FSM table-driven).
- **Backlog priorizado y decisiones descartadas:** ver `TODO_notes.txt` (mantenerlo
  actualizado al completar o descartar items).
- Convivencia transitoria: `task_pump_telemetry` (telemetría legacy en un solo JSON) y
  `mqtt_consumer` (sub-tópicos multi-rate) corren en paralelo hasta validar el nuevo en
  hardware; después se elimina el legacy.

---

## 12. Convenciones de trabajo

- **Idioma:** comentarios y docs mezclan español e inglés — mantener el estilo del
  archivo que se edita. Mensajes de commit en inglés, formato convencional
  (`feat(fsm): ...`, `fix(mqtt): ...`, `refactor: ...`).
- **Documentar decisiones:** si un cambio altera un contrato (tópicos, layout de flash,
  tabla FSM, DTOs), actualizar el `.md` de contrato correspondiente **en el mismo cambio**.
- **Compilación firmware:** CMake + Pico SDK v2.2.0 (toolchain ARM), tareas de VSCode ya
  configuradas en `.vscode/`. El test de host se compila aparte (sección 10).
- Ante ambigüedad entre este documento y el código: el código manda; reportar la
  discrepancia y actualizar este documento.
