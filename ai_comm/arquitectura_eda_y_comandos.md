# Arquitectura EDA y Pipeline de Comandos en pico2w_syringe_pump

Este documento recopila la explicación detallada sobre cómo está implementada la **Arquitectura Dirigida por Eventos (EDA)** en el firmware de la bomba jeringa, el funcionamiento del pipeline de comandos agnóstico hacia la FSM y la guía paso a paso para extender el sistema con nuevas interfaces (emisión de comandos y consumo de eventos).

---

## 1. Visión General y Filosofía de Diseño

El sistema opera sobre una **Raspberry Pi Pico 2W (RP2350)** con una arquitectura asimétrica dual-core:
* **Core 1 (Baremetal):** Control en tiempo real estricto a nivel de microsegundos (generación de pulsos PAP por PIO/DMA, lectura del sensor de presión Honeywell por SPI, encoder de cuadratura por PIO, detección de oclusión y fin de carrera).
* **Core 0 (FreeRTOS):** Conectividad IP (WiFi y MQTT con lwIP), interfaz gráfica TFT táctil (LVGL), consola serial CLI, persistencia en memoria Flash y monitoreo del sistema.

### ¿Por qué EDA en este proyecto?
> **Regla Invariante:** Core 1 **jamás** debe bloquearse, esperar mutexes compartidos con Core 0, ni llamar a primitivas como `printf()` o sockets de red.

Para garantizar esto sin acoplamiento sincrónico:
1. **Para telemetría, alarmas y estado:** Se utiliza un patrón **Publish-Subscribe con Broker central**. Todo cambio relevante se empaqueta en un `SystemEvent_t` y se emite de forma asíncrona y no-bloqueante.
2. **Para acciones de control del motor:** Se utiliza un pipeline unidireccional validado por un **Command Gate** que deposita mensajes en una cola de hardware IPC hacia Core 1.

---

## 2. Arquitectura de Eventos (Event Broker / Pub-Sub)

```
       PRODUCTORES (Generadores)                               BROKER                               CONSUMIDORES
 ┌────────────────────────────────────┐              ┌───────────────────────┐            ┌──────────────────────────────┐
 │ Core 1 (Baremetal)                 │              │                       │            │ serial_consumer.c (FreeRTOS) │
 │  - FSM transitions (fsm_table.c)   │ CORE1_EMIT   │                       ├──g_serial_q┤  - Traduce IDs a texto       │
 │  - Sensores / LSW / Oclusión       ├─────────────►│                       │ (cap. 32)  │  - printf() por consola UART │
 │  - Encoder / TMC2209 / Motor       │              │  task_event_broker    │            └──────────────────────────────┘
 │  - Alarmas y Heartbeat             │              │  (src/event_broker.c) │            ┌──────────────────────────────┐
 └────────────────────────────────────┘              │  Prioridad 4 (máxima) │            │ mqtt_consumer.c   (FreeRTOS) │
                                                     │                       ├──g_mqtt_q──┤  - Snapshot de estado        │
 ┌────────────────────────────────────┐ CORE0_EMIT   │  1. Estampa timestamp │ (cap. 32)  │  - Publicación multi-rate    │
 │ Core 0 (FreeRTOS Tasks)            ├─────────────►│  2. Efectos colater.  │            │    (sensors, motion, session)│
 │  - WiFi / MQTT State               │              │  3. Fan-out uniforme  │            └──────────────────────────────┘
 │  - cmd_dispatcher (CLI/MQTT/HMI)   │              │                       │            ┌──────────────────────────────┐
 │  - Sesión clínica (syringe_pump)   │              │                       ├──g_hmi_q───┤ hmi_consumer.c    (FreeRTOS) │
 │  - Batería / SysMon (heap, stats)  │              │                       │ (cap. 16)  │  - Filtra eventos clínicos   │
 └────────────────────────────────────┘              └───────────────────────┘            │  - Actualiza UI LVGL / TFT   │
```

### Contrato del Evento (`SystemEvent_t`)
Definido en `headers/system_events.h`:
* **Tamaño acotado:** `sizeof(SystemEvent_t) <= 64 bytes` garantizado por `_Static_assert`.
* **Sin strings dinámicos:** El payload es una unión de DTOs tipados (`DtoMotion_t`, `DtoForce_t`, `DtoAlarm_t`, `DtoFsm_t`, etc.). La única excepción es `DtoDebugStr_t` (56 bytes) para logs de debug de Core 1 (`LOG_DEBUG`).
* **Marcas de tiempo:** Core 1 emite con `timestamp_ms = 0` (no tiene acceso al tick de FreeRTOS); el broker lo estampa al recibirlo. Core 0 estampa al emitir (`CORE0_EMIT`).

### El Broker (`src/event_broker.c`)
Corre en Core 0 con la prioridad más alta (Prioridad 4):
1. **Drena colas:** Lee `g_crosscore_event_q` (cola lock-free con spinlock de hardware) y `g_core0_event_q` (cola FreeRTOS).
2. **Aplica efectos colaterales (`broker_side_effects`):** Actualiza el contexto clínico (`Pump_UpdatePressure`) y el espejo del estado de la FSM en `cmd_gate_update_fsm_state`.
3. **Persistencia diferida:** Si hay datos de calibración pendientes (`g_calibration_dirty`), espera a que el motor esté en reposo (`fsm_state_is_motion_idle`) antes de pausar núcleos y escribir en Flash.
4. **Fan-out uniforme:** Envía una copia a `g_serial_q`, `g_mqtt_q` y `g_hmi_q` con timeout `0` (drop silencioso si la cola está llena, jamás bloquea).

---

## 3. Catálogo de Generadores y Consumidores

### A. Generadores (Producers)

#### Desde Core 1 (Baremetal vía `CORE1_EMIT`):
* **FSM de Movimiento (`fsm_table.c`):**
  - `EV_APP_FSM_STATE`: Transiciones de estado (ej: de `ST_UNHOMED` a `ST_HOMING`, `ST_DISPENSING`).
  - `EV_APP_CALIBRATION`: Resultado y conteo de pasos al completar calibración.
* **Sensores de Proceso y Actuadores (`core1_main.c`):**
  - `EV_ACT_PRESSURE` / `EV_ACT_PRESSURE_OCC`: Lecturas periódicas de presión Honeywell SPI o superación del umbral de oclusión.
  - `EV_ACT_LSW_END`: Activación del switch de fin de carrera.
  - `EV_ACT_SYRINGE_DET`: Jeringa detectada o retirada.
* **Cinemática y Encoder PIO:**
  - `EV_MOT_ENCODER`, `EV_MOT_SPEED`, `EV_MOT_PROGRESS`: Conteo de encoder, velocidad y progreso de la infusión.
  - `EV_MOT_CORRECTION`: Correcciones de posición aplicadas en lazo cerrado (`closed_loop.c`).
* **Driver TMC2209:**
  - `EV_TMC_DRV_STATUS`, `EV_TMC_STALL`, `EV_TMC_OVERTEMP`: Flags diagnósticos del driver de motor.
* **Alarmas de Seguridad (IEC 60601-1-8):**
  - `EV_ALARM_OCCLUSION`, `EV_ALARM_EOT`, `EV_ALARM_DRV_FAULT`.
  *(Nota: El frenado o parada del motor ocurre antes en hardware; el evento es solo notificación).*
* **Salud / Depuración:**
  - `EV_SYS_HEARTBEAT`: Latido periódico de Core 1.
  - `EV_DBG_STRING`: Logs de texto formateados en Core 1 emitidos vía `LOG_DEBUG`.

#### Desde Core 0 (FreeRTOS vía `CORE0_EMIT`):
* **Red / Conectividad (`mqtt_client.c`):**
  - `EV_NET_WIFI_CONN`, `EV_NET_WIFI_DISC`, `EV_NET_WIFI_CONNECTING`: Estado de la red WiFi.
  - `EV_NET_MQTT_CONN`, `EV_NET_MQTT_DISC`, `EV_NET_MQTT_TX_DROP`: Estado de la sesión con el broker MQTT.
* **Despachador de Comandos (`cmd_dispatcher.c`):**
  - `EV_APP_CMD_EXECUTED`: Notificación para auditoría del resultado de cualquier comando (`DtoManualOp_t`).
* **Modelo Clínico (`syringe_pump_api.c`):**
  - `EV_APP_SESSION_START`, `EV_APP_SESSION_UPD`, `EV_APP_SESSION_END`: Volumen infundido, volumen objetivo, tasa (mL/h) y tiempo transcurrido.
* **Monitoreo del Sistema (`core0_main.c`):**
  - `EV_PWR_BATTERY_UPD`, `EV_PWR_MAINS_DETECT`: Nivel de batería y presencia de alimentación 220V.
  - `EV_SYS_HEAP_UPD`: Diagnóstico de memoria RAM libre y marcas de agua.
  - `EV_SYS_CLI_READY`: Notificación de consola serial lista.
  - `EV_SYS_CALIBRATION_SAVED`: Notificación de escritura en Flash completada.

---

### B. Consumidores (Consumers)

1. **`Serial Consumer` (`src/serial_consumer.c` — Tarea `SerialCons`):**
   - Cola: `g_serial_q` (capacidad 32).
   - Rol: Único lugar del firmware donde los IDs binarios se convierten a strings legibles (`format_event`). Imprime por UART/USB mediante `printf()`.
2. **`MQTT Consumer` (`src/mqtt_consumer.c` — Tarea `MqttCons`):**
   - Cola: `g_mqtt_q` (capacidad 32).
   - Rol: Mantiene un snapshot del estado (`MqttSnapshot_t`).
   - Publicación multi-rate:
     - Inmediata con QoS 1 para transiciones FSM (`bj/{id}/event`).
     - Cada 500 ms: `bj/{id}/telemetry/sensors` (presión).
     - Cada 1000 ms: `bj/{id}/telemetry/motion` (encoder, posición mm, velocidad).
     - Cada 2000 ms: `bj/{id}/telemetry/session` (volúmenes, tasa).
3. **`HMI Consumer` (`src/hmi_consumer.c` — Tarea `HmiCons`):**
   - Cola: `g_hmi_q` (capacidad 16, activa con `ENABLE_TFT`).
   - Rol: Filtra mediante `is_hmi_relevant()` eventos ruidosos y actualiza `ui_state.c` y los widgets gráficos LVGL de la pantalla táctil de 4".

---

## 4. Pipeline de Comandos: Ejecución Agnóstica a la Fuente

La FSM y Core 1 son **100% agnósticos al origen del comando**. No saben ni tienen forma técnica de saber si una orden provino de la consola serie, de un paquete MQTT o de la pantalla táctil.

```
[CLI Serial]       [MQTT Client]       [TFT / HMI Touch]   [Nueva interfaz (ej. BLE)]
     │                   │                     │                        │
     └───────────┬───────┴─────────────────────┴────────────────────────┘
                 │  "fsm_home,1500" + CmdSource_t (SERIAL/MQTT/HMI/BLE)
                 ▼
      ┌───────────────────────┐
      │   cmd_dispatcher.c    │  ◄── 1. Único parser de strings
      └──────────┬────────────┘      - Emite EV_APP_CMD_EXECUTED (auditoría)
                 │                   - Descarta 'source' para la ejecución
                 │  PUMP_ACTION_HOME, param1=1500, param2=0
                 ▼
      ┌───────────────────────┐
      │      cmd_gate.c       │  ◄── 2. Validador común agnóstico
      └──────────┬────────────┘      - Valida acción contra espejo s_fsm_state
                 │                   - STOP y RESET pasan incondicionalmente
                 ▼
      ┌───────────────────────┐
      │    crosscore_cmd.c    │  ◄── 3. Cola IPC (crosscore_cmd_queue)
      └──────────┬────────────┘      - Empaqueta en Core1CmdMessage_t
                 │                   - NO EXISTE campo 'source' en el struct
                 ▼
      ┌───────────────────────┐
      │      Core 1 / FSM     │  ◄── 4. Recibe evento EV_CMD_HOME
      │     (fsm_table.c)     │      - Evalúa: (from_state, event, guard) -> action
      └───────────────────────┘
```

### ¿Para qué se usa `CmdSource_t`?
El parámetro `src` en `cmd_dispatch_string` se usa únicamente para:
1. Sellar la auditoría en `EV_APP_CMD_EXECUTED` (`DtoManualOp_t.source`), permitiendo registrar en telemetría qué interfaz disparó la acción.
2. Responder el ACK correlacionado al cliente (usando el `cid` en MQTT).

---

## 5. Guía Práctica: Cómo Integrar una Nueva Interfaz

Si se desea agregar un nuevo medio de control (ejemplo: **Bluetooth BLE** o **HTTP/REST**):

### Parte A: Enviar Comandos hacia el Sistema

1. **Registrar la interfaz en `headers/cmd_dispatcher.h`:**
   ```c
   typedef enum {
       CMD_SRC_SERIAL = 0,
       CMD_SRC_MQTT   = 1,
       CMD_SRC_HMI    = 2,
       CMD_SRC_BLE    = 3,   // <-- Nueva interfaz
   } CmdSource_t;
   ```

2. **Invocar `cmd_dispatch_string` desde la tarea receptora:**
   ```c
   #include "cmd_dispatcher.h"

   void task_ble_rx(void *arg) {
       char rx_buf[128];
       int32_t req_id = -1;

       for (;;) {
           if (ble_receive_packet(rx_buf, sizeof(rx_buf), &req_id)) {
               // Enviar al pipeline central
               CmdDispatchResult_t res = cmd_dispatch_string(rx_buf, CMD_SRC_BLE, req_id);

               // Responder ACK/NACK al cliente
               if (res.accepted) {
                   ble_send_ack(req_id, "accepted");
               } else {
                   ble_send_nack(req_id, res.reason); // "invalid_state", "bad_format", etc.
               }
           }
       }
   }
   ```

---

### Parte B: Consumir Eventos del Sistema

1. **Declarar la cola en `headers/system_queues.h`:**
   ```c
   extern QueueHandle_t g_ble_q;
   ```

2. **Instanciar la cola en `src/system_queues.c`:**
   ```c
   QueueHandle_t g_ble_q = NULL;

   void system_queues_init(void) {
       // ...
       g_ble_q = xQueueCreate(16, sizeof(SystemEvent_t));
       configASSERT(g_ble_q);
   }
   ```

3. **Registrar el Fan-out en `src/event_broker.c`:**
   ```c
   static void broker_fanout(const SystemEvent_t *ev) {
       broker_side_effects(ev);
       if (g_serial_q != NULL) xQueueSend(g_serial_q, ev, 0);
       if (g_mqtt_q   != NULL) xQueueSend(g_mqtt_q,   ev, 0);
       if (g_hmi_q    != NULL) xQueueSend(g_hmi_q,    ev, 0);
       if (g_ble_q    != NULL) xQueueSend(g_ble_q,    ev, 0);  // Envío no-bloqueante
   }
   ```

4. **Implementar el consumidor (`src/ble_consumer.c`):**
   ```c
   #include "system_queues.h"
   #include "system_events.h"

   static void task_ble_consumer(void *arg) {
       (void)arg;
       SystemEvent_t ev;

       for (;;) {
           if (xQueueReceive(g_ble_q, &ev, portMAX_DELAY) == pdTRUE) {
               // Filtrar y procesar lo relevante
               if (EV_IS_ALARM(ev.id)) {
                   ble_notify_alarm(ev.id, ev.payload.alarm.severity);
               } else if (ev.id == EV_APP_FSM_STATE) {
                   ble_notify_state(ev.payload.fsm.state_from, ev.payload.fsm.state_to);
               } else if (ev.id == EV_ACT_PRESSURE) {
                   ble_notify_pressure(ev.payload.force.mmhg);
               }
           }
       }
   }

   void ble_consumer_start(void) {
       if (g_ble_q == NULL) return;
       xTaskCreate(task_ble_consumer, "BleCons", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
   }
   ```

5. **Iniciar el consumidor en `src/core0_main.c`:**
   ```c
   serial_consumer_start();
   mqtt_consumer_start();
   hmi_consumer_start();
   ble_consumer_start(); // Iniciar nuevo consumidor
   ```

---

## 6. Resumen de Invariantes de Seguridad

1. **Core 1 nunca imprime ni bloquea:** Emite por `CORE1_EMIT` (no-bloqueante, drop silencioso si la cola se llena).
2. **`cmd_gate_execute` es la única vía hacia Core 1:** Ningún generador escribe directamente en `crosscore_cmd_queue`.
3. **STOP siempre pasa:** `stop` y `stop_imm` se despachan de emergencia sin validar el estado FSM.
4. **El Broker es neutral:** Distribuye a todos por igual; cada consumidor decide qué filtrar y a qué frecuencia transmitir.
