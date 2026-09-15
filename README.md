# Pico W Syringe Pump

Este proyecto implementa el control de una bomba de jeringa de alta precisión utilizando la placa **Raspberry Pi Pico W/2W (RP2040/RP2350)**. 

Se destaca por el uso concurrente de un driver para motores paso a paso **TMC2209**, un sensor de presión SPI **Honeywell HSC**, conectividad Wi-Fi, y la utilización extensiva del hardware de la placa (DMA, Hardware Spinlocks, PIO).

## Arquitectura Dual-Core Híbrida

Para garantizar estabilidad, respuesta en tiempo real (hard real-time) y funciones conectadas en un mismo microcontrolador, el proyecto divide lógicamente las responsabilidades de procesamiento entre ambos núcleos disponibles bajo una arquitectura híbrida:

### **Core 1: Baremetal (Tiempo Real)**
El núcleo 1 se ejecuta **sin sistema operativo (Baremetal)** impulsado completamente por interrupciones de hardware, temporizadores dedicados, DMA (Direct Memory Access) y algoritmos bloqueantes controlados.
- **Responsabilidades:**
  - **Máquina de estados de movimiento table-driven** (`src/fsm_table.c`): dos tablas de transiciones (`GLOBAL[]` de alta prioridad + `TRANSITIONS[]` por estado) con guards, actions y una política por defecto sin `default: break` silencioso. Incluye auditoría de cobertura de la matriz completa estado × evento (trazabilidad IEC 62304).
  - **Tres capas de detección de falla**: finales de carrera físicos (Layer 1), stall por encoder — cuenta sin cambio con DMA activo (Layer 2), y deadline derivado de distancia/velocidad (Layer 3). *StallGuard del TMC2209 no se usa como mecanismo de seguridad* (poco confiable a baja velocidad); queda como telemetría diagnóstica opcional.
  - Comunicación SPI1 ultrarrápida (1 MHz) con el sensor de presión Honeywell para monitorear sobrepresiones con lecturas deterministas cada 500 ms.
  - Generación de pulsos para el motor usando la abstracción en C hacia el componente PIO y perfiles trapezoidales de 2 segmentos en DMA para aceleración/desaceleración suave del TMC2209.
  - Lectura de encoder de cuadratura por PIO (semi-lazo cerrado de posición durante el dispensado, `src/closed_loop.c`).
  - Configuración UART bidireccional asíncrona dedicada (57600 baudios) para programar microstepping y corriente del motor dinámicamente en el TMC2209.
  - Polling de alta frecuencia de los finales de carrera (`START_PIN` / `END_PIN`), con rampa de frenado al impacto y **dead-time de 200 ms** entre el freno y el movimiento de release en dirección opuesta.

### **Core 0: FreeRTOS (Procesamiento Concurrente Asíncrono)**
El núcleo 0 ejecuta un kernel de **FreeRTOS** y centraliza todas las entradas, salidas globales del usuario (puerto serie / WiFi CYW43), temporizadores relajados y telemetría general.
- **Responsabilidades:**
  - Inicialización del subsistema Wi-Fi, cliente MQTT sobre lwIP y reconexión automática.
  - **Event broker central** (`task_event_broker`) que distribuye los eventos del sistema a consumidores independientes (serial, MQTT, HMI) — ver sección siguiente.
  - Pipeline unificado de comandos (CLI / MQTT / HMI) con validación de estado FSM antes de despachar a Core 1.
  - CLI por consola serie, persistencia de configuración en Flash, monitor de salud del sistema (stack high-water-marks) y tareas de UI TFT (gated por `ENABLE_TFT`).

---

## Arquitectura de Eventos Pub-Sub (EDA)

Toda notificación del sistema (sensores, transiciones FSM, red, alarmas, salud) viaja como un evento tipado `SystemEvent_t` (máx. 64 bytes, sin strings — garantizado por `_Static_assert`) hacia un **broker central** que lo replica a consumidores desacoplados:

```
Core 1 ── CORE1_EMIT ──► g_crosscore_event_q (queue_t SDK, no bloqueante) ──┐
                                                                            ├─► task_event_broker
Core 0 ── CORE0_EMIT ──► g_core0_event_q (FreeRTOS) ────────────────────────┘        │
                                                                                     ├─► serial_consumer → printf
                                                                                     ├─► mqtt_consumer   → publish multi-rate
                                                                                     └─► hmi_consumer    → TFT/LVGL
```

**¿Por qué?** `printf()` (y cualquier primitiva que comparta spinlocks con el driver CYW43) puede bloquear al Core 1 un tiempo impredecible y arruinar los plazos de microsegundos del control de motor. Por eso el Core 1 **jamás imprime**: emite eventos con `CORE1_EMIT` sobre una cola de hardware no-bloqueante (`pico/util/queue.h`) y sigue moviendo el motor; si la cola está llena el evento se descarta en silencio. El broker (prioridad máxima en FreeRTOS) sella el timestamp y hace fanout uniforme sin interpretar el contenido; cada consumidor decide qué eventos le interesan (p. ej. filtro de alarmas `EV_IS_ALARM`).

Los IDs de evento están organizados por dominio en `headers/system_events.h` (driver TMC `0x01xx`, sensores `0x02xx`, motor `0x03xx`, red `0x04xx`, aplicación/FSM `0x05xx`, alarmas `0x06xx`, energía `0x07xx`, sistema `0x08xx`), con DTOs tipados por dominio (`DtoMotion_t`, `DtoAlarm_t`, `DtoSession_t`, ...).

---

## Pipeline de Comandos

Todos los comandos, vengan de donde vengan, atraviesan la misma cadena:

```
CLI / MQTT / HMI → cmd_dispatch_string() → cmd_gate_execute() → crosscore_cmd_queue → FSM Core 1
                   (parser único)          (valida estado FSM)   (queue_t no bloqueante)
```

- `cmd_dispatcher.c` es el **único parser** de comandos string; retorna `accepted/reason` para que cada interfaz arme su ACK (MQTT usa el correlation ID).
- `cmd_gate.c` mantiene un espejo del estado FSM de Core 1 (actualizado solo por el broker vía `EV_APP_FSM_STATE`) y rechaza acciones inválidas para el estado actual **antes** de encolarlas.
- **STOP / STOP_IMM nunca se validan contra el estado FSM** — se aceptan siempre (invariante de seguridad).

---

## Test de la FSM en Host (Golden Table)

`tools/fsm_test/` compila la tabla FSM real en una PC (sin Pico SDK ni toolchain ARM) y verifica cada transición contra una tabla de referencia dorada:

```bash
cd tools/fsm_test
gcc -Wall -Wextra -o test_golden test_fsm_golden.c && ./test_golden
```

Correrlo es obligatorio tras cualquier cambio en `fsm_table.c` o en los estados/eventos de `core1_main.h`.

---

## Interfaz de Comandos y Telemetría (MQTT / CLI)

El sistema soporta el envío de comandos de movimiento y la configuración dinámica de credenciales mediante *dos interfaces unificadas*:
1. **MQTT**: Mediante la subscripción al tópico de comandos definido y publicando payloads de texto.
2. **CLI (Puerto Serial)**: Abriendo la consola UART/USB de la Pico y tecleando los comandos directamente.

Ambas interfaces convergen en el mismo parser (`cmd_dispatch_string`) y la misma validación de estado (`cmd_gate`): un comando `fsm_*` inválido para el estado actual se rechaza con `invalid_state` (y el ACK MQTT lo refleja). Los comandos de bajo nivel (`home_start`, `nsteps`, movimiento lineal) bypasean la validación FSM — son para banco de pruebas, no para operación clínica.

### Comandos de Operación Disponibles (Vía CLI o MQTT payload)

| Comando Payload | Descripción | Notas |
|---|---|---|
| `stop_imm` o `STOP_IMM` | Parada inmediata (Hard Stop) | Frena el motor deteniendo su generador abruptamente. Prioridad máxima: se procesa antes que cualquier comando pendiente en la cola. |
| `stop` o `STOP` | Parada suave (Soft Stop) | Desacelera respetando la rampa de 2 segmentos hasta llegar a 0. |
| `fsm_home,<VEL>` | FSM: Inicio (Homing) | Inicia la secuencia de búsqueda del tope de inicio a la velocidad indicada (um/s). Ej: `fsm_home,1500.0` |
| `fsm_search,<VEL>` | FSM: Buscar jeringa | Inicia la búsqueda del émbolo de la jeringa a la velocidad indicada (um/s). Ej: `fsm_search,1200.0` |
| `fsm_dispense,<TARGET>,<VEL>` | FSM: Dosificar | Inicia la dosificación a una posición dada (um) y velocidad (um/s). Ej: `fsm_dispense,10000.0,450.0` |
| `fsm_search_eot` | FSM: Buscar Fin de Carrera | Busca el tope de fin de carrera (End Of Travel). |
| `fsm_reset` | FSM: Reset | Resetea la máquina de estados a ST_UNHOMED. |
| `fsm_cont` | FSM: Continuar | Continúa la dosificación previamente pausada. |
| `fsm_occ_rel` | FSM: Liberar Oclusión | Retrocede el motor para liberar presión tras una oclusión. |
| `fsm_resume` | FSM: Reanudar | Reanuda la operación después de resolver un evento. |
| `fsm_calibrate[,<VEL_MOVE>[,<VEL_SEEK>]]` | FSM: Calibrar Encoder | Secuencia de ida y vuelta a los topes para medir el recorrido máximo en encoder counts. Velocidades opcionales (0 o ausente = default). El resultado se persiste automáticamente en Flash y se recarga al boot. Ej: `fsm_calibrate,1500,800` |
| `fsm_enc_reset` | Reset de encoder | Pone en cero la cuenta del encoder. No pasa por el gate ni cambia el estado FSM. |
| `home_start,<VEL>` | Homing manual (Atrás) | Motor se mueve en dirección negativa a velocidad constante hasta hallar el tope físico. Ej: `home_start,1200` |
| `home_end,<VEL>` | Homing manual (Adelante) | Motor se mueve en dirección positiva a velocidad constante hasta hallar el tope físico. Ej: `home_end,1200` |
| `nsteps,<pasos>,<freq_hz>` | Movimiento por pasos puros | Inyecta N pasos a una frecuencia fija (Hz). Ej: `nsteps,3200,500.0` |
| `<POSICION>,<VELOCIDAD>` | Movimiento lineal (fallback) | Mueve a la posición indicada (um) a la velocidad indicada (um/s). Se ejecuta si el payload no coincide con ningún comando anterior. Ej: `15000.0,500.0` |

### Comandos de Configuración Exclusivos de CLI

Actualmente, estos comandos son accesibles mediante la consola serial y se utilizan para guardar datos persistentes en la memoria **Flash** del microcontrolador (Thread-Safe mediante *multicore lockout*).

| Comando CLI | Descripción |
|---|---|
| `config_wifi,<SSID>,<PASS>`| Guarda temporalmente en RAM y usa las nuevas credenciales de Wi-Fi. |
| `config_mqtt,<IP>,<PORT>` | Guarda temporalmente en RAM y usa la nueva IP/Puerto del Broker. |
| `set_ssid,<SSID>` | Cambia únicamente el SSID del Wi-Fi en RAM. |
| `set_wpass,<PASS>` | Cambia únicamente la contraseña del Wi-Fi en RAM. |
| `set_mqtt_ip,<IP>` | Cambia únicamente la IP del Broker MQTT en RAM. |
| `set_mqtt_port,<PORT>`| Cambia únicamente el Puerto del Broker MQTT en RAM. |
| `config_save` | Escribe los valores de RAM en la Memoria Flash profunda de forma definitiva. (Solo funcionará si el motor no se está moviendo). |
| `config_info` | Muestra un resumen de variables actuales cargadas en el gestor. |
| `reconnect` | Efectúa un reseteo *suave* (Soft Reset) del hardware Wi-Fi desasociándolo de su actual red (LwIP leave) obligándolo a re-engancharse y conectar MQTT con la nueva configuración sin reiniciar el procesador. |
| `net_disable` | Apaga el chip Wi-Fi (desactiva RF y tareas de red) para máximo ahorro de batería. |
| `net_enable` | Enciende el chip Wi-Fi y restaura la conectividad de red a sus valores guardados. |
| `log_en,<HDR>` | Activa una categoría de log de diagnóstico en el serial. Usar `ALL` para activar todas. |
| `log_dis,<HDR>` | Desactiva una categoría de log de diagnóstico. Usar `ALL` para silenciar todo. |

**Categorías de log disponibles (`<HDR>`):**

| Header | Descripción |
|---|---|
| `ALL` | Activa / desactiva todas las categorías a la vez. |
| `TGT` | Target de movimiento (distancia y velocidad solicitados). |
| `CFG` | Configuración del driver aplicada (microsteps, corriente, chopper). |
| `KIN` | Cinemática calculada (um/paso, total de micropasos). |
| `PRF` | Perfil de aceleración calculado (frecuencias, segmentos). |
| `FSM` | Transiciones de estado de la FSM. |
| `ADC` | Lecturas del ADC. |
| `PRG` | Progreso del movimiento en curso (porcentaje). |
| `ENC` | Lecturas del encoder (posición y velocidad). |
| `MTR` | Eventos de parada del motor. |

### Tópicos MQTT

El sistema usa una jerarquía de tópicos con ID de dispositivo para soporte multi-bomba.
El `{id}` tiene el formato `bj-XXXXXXXX` derivado del ID único del chip (ej: `bj-a1b2c3d4`).
Ver `MQTT_CONTRACT.md` en la raíz del repo para el contrato completo (esquemas JSON, QoS, diagramas).

#### Tópicos activos

| Tópico | QoS | Retain | Descripción |
|--------|-----|--------|-------------|
| `bj/{id}/status` | 1 | Sí | Online/Offline. LWT configurado: el broker publica `offline` si se pierde el keepalive (60 s). |
| `bj/{id}/cmd` | 1 | No | Comandos con envelope `{"cid":N,"cmd":"..."}` (fallback a string crudo para debug) |
| `bj/{id}/cmd/ack` | 1 | No | Confirmación de comandos con correlation ID `{"cid":N,"result":"accepted"\|"rejected"}` |
| `bj/{id}/telemetry` | 0 | No | JSON de telemetría clínica cada 2 s *(legacy — convive con los sub-tópicos hasta validar en hardware)* |
| `bj/{id}/telemetry/sensors` | 0 | No | Presión (psi/mmHg) cada 500 ms — publicado por `mqtt_consumer` |
| `bj/{id}/telemetry/motion` | 0 | No | Encoder, posición, velocidad, progreso cada 1 s — publicado por `mqtt_consumer` |
| `bj/{id}/telemetry/session` | 0 | No | Sesión de infusión (volúmenes, tasa, tiempo) cada 2 s — publicado por `mqtt_consumer` |
| `bj/{id}/event` | 1 | No | Transiciones de estado FSM `{"type":"state","from":N,"to":N}` (eventos de alarma: pendiente) |

**Suscripción recomendada en Node-RED:**
```
bj/+/status     QoS 1, retain=true
bj/+/telemetry  QoS 0
bj/+/event      QoS 1
bj/+/cmd/ack    QoS 1
```

#### Comandos vía `bj/{id}/cmd`

El payload debe ser un JSON con correlation ID y el string de comando:
```json
{"cid": 17, "cmd": "fsm_dispense,10000.0,450.0"}
```
Los strings de comando son los mismos que los de la CLI (ver tabla de comandos más arriba).

#### Tópicos legacy (eliminados en PR2)

Los tópicos `syringe_pump/*` fueron eliminados del firmware. No usar en integraciones nuevas.

### TODO

#### MQTT / IoT
- [x] Identidad de dispositivo única (`device_id` derivado de chip ID, formato `bj-XXXXXXXX`)
- [x] Módulo `mqtt_topics` — jerarquía `bj/{id}/...` sin strings literales en el código
- [x] Last Will Testament (LWT) — broker publica `offline` ante desconexión inesperada
- [x] Fix bug topic en callbacks RX MQTT (`s_rx_topic`, `MQTT_DATA_FLAG_LAST`)
- [x] `MQTT_CONTRACT.md` — contrato de tópicos, esquemas JSON, QoS, diagrama cmd/ack
- [x] Migrar telemetría y logs a `bj/{id}/...` (eliminar literales `syringe_pump/*`) **PR2**
- [x] `nodered/pump_simulator.py` en el repo con tópicos `bj/{id}/*` actualizados **PR2**
- [x] `cmd_envelope` + ACK correlacionado en `bj/{id}/cmd/ack` **PR3**
- [x] `MQTT_MAX_PAYLOAD=256` — fix truncamiento silencioso de telemetría (~162 chars > 127) **PR4**
- [ ] Dashboard Node-RED: overview, detalle, alarmas, control remoto, datalog
- [ ] Notificaciones PWA (Web Push)

#### Firmware
- [x] Implementar CLI para control del sistema.
- [x] Arquitectura de eventos pub-sub (broker + consumers serial/MQTT/HMI).
- [x] FSM table-driven con auditoría de cobertura + test golden en host (`tools/fsm_test`).
- [x] Pipeline unificado de comandos con validación de estado FSM (`cmd_dispatcher` + `cmd_gate`).
- [x] Calibración punta a punta con persistencia en Flash y recarga al boot.
- [x] Detección de fallas por capas: stall de encoder, deadline de movimiento, timeouts de release de LSW.
- [ ] Implementar libreria de control de TFT+Touch y lógica de menues. **WIP** *(submodulo integrado en branch `tft_integration`)*
- [ ] Implementar control lazo cerrado (driver+motor PAP, encoder). **WIP** *(lazo de desplazamiento activo; lazo de velocidad pendiente)*
- [ ] Implementar watchdog de hardware alimentado desde ambos cores.
- [ ] Librería sensor de fuerza: ADC → presión (mmHg), calibración persistida, detección de desconexión.
- [ ] Detector de burbujas + integración a FSM.
- [ ] Migrar Flash a littlefs (config, calibración, log de eventos, sesiones).
- [ ] RTC DS3231 + timestamps reales en eventos y sesiones.
- [ ] Rutina de autochequeo inicial (POST) al boot.
- [ ] Modo bajo consumo y monitoreo de energía (batería/red).

> El backlog completo, priorizado y con contexto de cada item vive en [`TODO_notes.txt`](TODO_notes.txt).

### Documentación

| Documento | Contenido |
|---|---|
| [`CLAUDE_CODE_CONTEXT.md`](CLAUDE_CODE_CONTEXT.md) | **Arquitectura y reglas invariantes** — punto de entrada para desarrolladores y asistentes LLM |
| [`MQTT_CONTRACT.md`](MQTT_CONTRACT.md) | Contrato MQTT: tópicos, esquemas JSON, QoS, envelope cmd/ack |
| [`MEMORY_CONTRACT.md`](MEMORY_CONTRACT.md) | Layout de flash y reglas de persistencia |
| [`UI_BJ_CONTRACT.md`](UI_BJ_CONTRACT.md) | Integración del módulo TFT/touch en Core 0 |