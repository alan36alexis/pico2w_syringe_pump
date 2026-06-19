# Contrato UI — Módulo BJ (TFT + Touch + Encoder + Keys)

Documento de referencia para la integración del módulo TFT/UI en `core0_main.c` FreeRTOS.  
Define los tipos de datos compartidos, mecanismos de IPC y responsabilidades de cada parte.

---

## Arquitectura de integración

```
┌──────────────────────────────────────────────────── CORE 0 — FreeRTOS ─────┐
│                                                                              │
│   task_ui_input  ─── UIEvent_t queue ──►  task_hmi / task_pump_control      │
│   (encoder + keys + touch)                  │                               │
│                                             │  pump_hmi_execute()           │
│   task_tft  ◄── ui_state_get_snapshot() ───┘  (ya existente)               │
│   (render LVGL)                                                              │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
         ▲  SPI1 (display)                         │  crosscore_cmd_queue
         │  SPI0/I2C (touch)                       ▼
         │  GPIO (encoder + keys)           ┌─ CORE 1 — Baremetal ─┐
    [MSP4020/4021]                          │   Motor / Sensor       │
    [XPT2046 o equiv]                       └────────────────────────┘
```

Puntos clave:
- El módulo TFT/UI corre **exclusivamente en Core 0** como tareas FreeRTOS.
- **No hay hardware FIFO** entre módulo UI y el resto: la comunicación es mediante colas FreeRTOS (`QueueHandle_t`).
- La **dirección UI → Pump** usa `ui_event_queue` (evento de usuario → lógica de bomba).
- La **dirección Pump → UI** usa `ui_state_get_snapshot()` (lectura periódica de `UIState_t`, ya implementado).
- `pump_hmi_execute()` es el único punto de entrada para despachar acciones de la bomba.

---

## Tareas del módulo UI

### `task_tft` — Renderizado de display

| Parámetro | Valor |
|---|---|
| Prioridad FreeRTOS | `tskIDLE_PRIORITY + 2` |
| Stack | 4096 words (RP2350: 16 KB) |
| Período | 20 ms (50 Hz tick LVGL máximo) |
| Bloqueo | Nunca — siempre usa `xQueueReceive(..., 0)` |

**Responsabilidades:**
- Llamar a `lv_tick_inc(20)` y `lv_task_handler()` en cada ciclo.
- Leer `ui_state_get_snapshot()` para actualizar widgets LVGL.
- Procesar `UIEvent_t` de tipo `UI_EVENT_TOUCH` del `ui_event_queue` para actualizar cursor/selección.
- **No llama a `pump_hmi_execute()` directamente** — solo dibuja y reenvía eventos táctiles.

### `task_ui_input` — Lectura de periféricos UI

| Parámetro | Valor |
|---|---|
| Prioridad FreeRTOS | `tskIDLE_PRIORITY + 3` |
| Stack | 512 words (2 KB) |
| Período | 10 ms polling (encoder/keys) |
| Bloqueo | `xQueueSend(..., 0)` — evento descartado si cola llena |

**Responsabilidades:**
- Leer encoder rotativo y calcular delta de detents.
- Detectar pulsación, liberación y hold de teclas (con debounce de software).
- Leer pantalla táctil y enviar coordenadas calibradas.
- Enviar `UIEvent_t` al `ui_event_queue`.
- **No ejecuta lógica de negocio ni accede a FreeRTOS semaphores de la bomba.**

---

## Tipos compartidos — `ui_events.h`

> Este header es el "contrato" en código C. Toda función que produzca o consuma
> eventos UI debe incluirlo.

```c
// ui_events.h  — Ver archivo adjunto
```

Ver sección **Anexo A** para el header completo.

---

## Cola de eventos UI

```c
// Definida globalmente en ui_events.c / ui_events.h
extern QueueHandle_t ui_event_queue;

// Profundidad de la cola
#define UI_EVENT_QUEUE_DEPTH  16

// Inicializar (llamar ANTES de xTaskCreate para task_tft y task_ui_input)
void ui_events_init(void);
```

**Reglas:**
- Solo `task_ui_input` escribe en la cola (single producer).
- `task_tft` lee eventos `UI_EVENT_TOUCH` para actualizar el display.
- `task_hmi` (o equivalente en `core0_main.c`) lee eventos `UI_EVENT_KEY_*` y `UI_EVENT_ENCODER_DELTA` para llamar a `pump_hmi_execute()`.
- Si la cola está llena, el evento se descarta silenciosamente (`xQueueSend(..., 0)`).

---

## Mapa de teclas físicas → UIKeyID_t

| Tecla física | `UIKeyID_t` | Función principal | Función hold (>1s) |
|---|---|---|---|
| Tecla START/STOP | `UI_KEY_START_STOP` | Iniciar / Pausar infusión | Detener definitivo |
| Tecla CONFIRM/OK | `UI_KEY_CONFIRM` | Confirmar selección | — |
| Tecla BACK/ESC | `UI_KEY_BACK` | Cancelar / Volver | — |
| Tecla ALARM SILENCE | `UI_KEY_ALARM_SILENCE` | Silenciar alarma activa | — |
| Encoder press | `UI_KEY_ENCODER_PRESS` | Confirmar en menú | — |

**Hold threshold:** 1000 ms. El evento `UI_EVENT_KEY_HOLD` se emite una sola vez al superar el umbral.

---

## Encoder rotativo

- Resolución: 1 evento `UI_EVENT_ENCODER_DELTA` por detent mecánico.
- Sentido: `delta = +1` (horario), `delta = -1` (antihorario).
- Aceleración: **No implementada en este contrato** — a decisión del módulo UI.
- La lectura se hace por polling en `task_ui_input` (no por interrupción) para evitar problemas con FreeRTOS tick.

---

## Pantalla táctil

- Coordenadas: sistema de referencia del display físico (0,0 = esquina superior izquierda).
- Rango: X ∈ [0, 479], Y ∈ [0, 319] (display 480×320).
- Calibración: coeficientes almacenados en `SystemConfig_t` (ver MEMORY_CONTRACT).
- Un toque válido requiere dos lecturas consecutivas dentro de 50 px (debounce).

---

## Máquina de estados de pantalla (Screen State Machine)

La lógica de navegación de pantallas es **interna al módulo TFT** — no es parte de este contrato.  
El contrato solo especifica:

1. El módulo TFT **no decide** si una acción de bomba es válida. Envía el evento UI y
   `pump_hmi_execute()` decide si es ejecutable en el estado actual del FSM.
2. El módulo TFT **sí decide** qué pantalla mostrar según `UIState_t.fsm_state`.
3. Los estados de alarma (`PUMP_STATE_ALARM`) deben mostrar pantalla de alarma con prioridad,
   bloqueando la navegación normal.

---

## Inicialización — Orden requerido

```c
// En core0_main_setup(), ANTES de vTaskStartScheduler():

// 1. Inicializar hardware SPI para display y touch
spi_init(SPI_PORT_TFT, TFT_SPI_FREQ_HZ);
// ... configuración de pines ...

// 2. Inicializar driver TFT y LVGL
tft_driver_init();
lv_init();
lv_display_set_default(lv_display_create(...));

// 3. Inicializar sistema de eventos UI
ui_events_init();   // Crea ui_event_queue

// 4. Inicializar estado compartido (ya existe)
ui_state_init();    // Ya existe en ui_state.h

// 5. Crear tareas
xTaskCreate(task_ui_input, "ui_input", 512,  NULL, tskIDLE_PRIORITY+3, NULL);
xTaskCreate(task_tft,      "tft",      4096, NULL, tskIDLE_PRIORITY+2, NULL);
```

---

## Reglas de thread safety

| Recurso | Escritura | Lectura | Mecanismo |
|---|---|---|---|
| `ui_event_queue` | `task_ui_input` | `task_tft`, `task_hmi` | Cola FreeRTOS — intrínsecamente thread-safe |
| `UIState_t` (snapshot) | `task_logger` | `task_tft` | Mutex en `ui_state.h` (ya implementado) |
| Hardware SPI TFT | `task_tft` | — | Uso exclusivo de `task_tft` — no compartir |
| Hardware SPI/I2C touch | `task_ui_input` | — | Uso exclusivo de `task_ui_input` |
| `g_sys_config` | `task_init`, CLI | `task_tft` (solo lectura) | Mutex de `config_manager` |

**Regla crítica:** Ninguna función del módulo TFT/UI llama a `vTaskDelay()` durante la
inicialización del hardware SPI antes de que el scheduler esté corriendo. Usar
`busy_wait_us()` del SDK de Pico en esa fase.

---

## Versiones

| Campo | Valor |
|---|---|
| Display target | MSP4020 / MSP4021 (480×320, SPI, ILI9488) |
| Touch target | XPT2046 (SPI) |
| Encoder | Mecánico, 20 detents/vuelta |
| Teclas | Pull-up + active-low, debounce 20 ms |
| LVGL | v9.x |
| Esquema | v1 |

---

## Anexo A — `ui_events.h` (header del contrato)

```c
#ifndef UI_EVENTS_H
#define UI_EVENTS_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ─── Profundidad de la cola ───────────────────────────────────────────────────
#define UI_EVENT_QUEUE_DEPTH  16

// ─── Identificadores de teclas ───────────────────────────────────────────────
typedef enum {
    UI_KEY_START_STOP      = 0,
    UI_KEY_CONFIRM         = 1,
    UI_KEY_BACK            = 2,
    UI_KEY_ALARM_SILENCE   = 3,
    UI_KEY_ENCODER_PRESS   = 4,
    UI_KEY_COUNT
} UIKeyID_t;

// ─── Tipos de evento ─────────────────────────────────────────────────────────
typedef enum {
    UI_EVENT_KEY_PRESS    = 0,   // Pulsación detectada (flanco descendente)
    UI_EVENT_KEY_RELEASE  = 1,   // Liberación detectada (flanco ascendente)
    UI_EVENT_KEY_HOLD     = 2,   // Hold superó UI_KEY_HOLD_THRESHOLD_MS (una sola vez)
    UI_EVENT_ENCODER_DELTA = 3,  // Rotación de encoder
    UI_EVENT_TOUCH        = 4,   // Toque táctil válido
} UIEventType_t;

// ─── Hold threshold ──────────────────────────────────────────────────────────
#define UI_KEY_HOLD_THRESHOLD_MS  1000U

// ─── Struct de evento ────────────────────────────────────────────────────────
typedef struct {
    UIEventType_t type;
    uint32_t      timestamp_ms;   // xTaskGetTickCount() * portTICK_PERIOD_MS
    union {
        struct {
            UIKeyID_t id;
            uint32_t  hold_ms;    // Válido solo para UI_EVENT_KEY_HOLD
        } key;
        struct {
            int8_t delta;         // +1 horario, -1 antihorario (1 por detent)
        } encoder;
        struct {
            uint16_t x;           // Píxeles, referencia display físico
            uint16_t y;
            bool     pressed;     // true=toque activo, false=levantó dedo
        } touch;
    };
} UIEvent_t;

// ─── Handle de la cola (definido en ui_events.c) ─────────────────────────────
extern QueueHandle_t ui_event_queue;

// ─── API ─────────────────────────────────────────────────────────────────────

/**
 * @brief Crea la cola de eventos UI.
 *        Llamar ANTES de xTaskCreate() de task_tft y task_ui_input.
 */
void ui_events_init(void);

/**
 * @brief Envía un evento a la cola (non-blocking).
 *        Descarta el evento si la cola está llena.
 *        Solo llamar desde task_ui_input o ISR con FromISR variant.
 * @return true si el evento fue encolado, false si se descartó.
 */
bool ui_event_send(const UIEvent_t *event);

/**
 * @brief Variante para ISR (si se usan interrupciones de GPIO para teclas).
 */
bool ui_event_send_from_isr(const UIEvent_t *event, BaseType_t *pxHigherPriorityTaskWoken);

#ifdef __cplusplus
}
#endif

#endif // UI_EVENTS_H
```

---

## Notas de diseño

**¿Por qué cola FreeRTOS y no hardware FIFO?**  
El hardware FIFO (queue.h del SDK) está reservado para el IPC Core0↔Core1 que ya existe.
El módulo UI corre en el mismo core que el resto de FreeRTOS, por lo que usar
`QueueHandle_t` de FreeRTOS es la solución correcta: thread-safe, con timeout configurable
y con soporte para análisis de stack overflow.

**¿Por qué `task_ui_input` tiene prioridad más alta que `task_tft`?**  
Los eventos de entrada deben capturarse antes de que la próxima ventana de renderizado
consuma el ciclo. Con un período de polling de 10 ms para input y 20 ms para TFT,
la prioridad más alta garantiza que los eventos no se acumulen entre frames.

**¿Por qué no ISR para las teclas?**  
Las ISR de GPIO en FreeRTOS en RP2350 requieren atención especial con spinlocks.
Para este sistema, 10 ms de latencia de input es perfectamente aceptable para una
interfaz de bomba médica. El polling simplifica el código y evita race conditions.
