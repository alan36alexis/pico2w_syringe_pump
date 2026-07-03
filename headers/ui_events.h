/**
 * @file    ui_events.h
 * @brief   Contrato C de eventos de interfaz de usuario local.
 *
 * Define los tipos de datos y la API de la cola de eventos UI.
 * Este header es el punto de acuerdo entre:
 *   - Productor: task_ui_input (encoder, keys, touch)
 *   - Consumidores: task_tft (touch → LVGL), task_hmi (keys/encoder → pump_hmi)
 *
 * Ver UI_BJ_CONTRACT.md para el diseño completo.
 */

#ifndef UI_EVENTS_H
#define UI_EVENTS_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Profundidad de la cola ──────────────────────────────────────────────── */
#define UI_EVENT_QUEUE_DEPTH    16U

/* ─── Debounce y hold threshold ───────────────────────────────────────────── */
#define UI_KEY_DEBOUNCE_MS      20U     /**< Tiempo mínimo de estabilidad para key press/release */
#define UI_KEY_HOLD_THRESHOLD_MS 1000U  /**< Hold emite UI_EVENT_KEY_HOLD una sola vez */
#define UI_TOUCH_JITTER_PX      50U     /**< Radio máximo para considerar dos lecturas el mismo toque */

/* ─── Identificadores de teclas ────────────────────────────────────────────── */
/**
 * @brief Identifica cada tecla física o acción del encoder.
 *
 * Mapeo de hardware (activo-bajo con pull-up):
 *   UI_KEY_START_STOP     → GPIO definido en board_config.h
 *   UI_KEY_CONFIRM        → GPIO definido en board_config.h
 *   UI_KEY_BACK           → GPIO definido en board_config.h
 *   UI_KEY_ALARM_SILENCE  → GPIO definido en board_config.h
 *   UI_KEY_ENCODER_PRESS  → GPIO del encoder (push)
 */
typedef enum {
    UI_KEY_START_STOP      = 0,  /**< START / PAUSE (hold > 1s = STOP definitivo) */
    UI_KEY_CONFIRM         = 1,  /**< OK / ENTER en menú                          */
    UI_KEY_BACK            = 2,  /**< ESC / CANCEL / volver pantalla anterior      */
    UI_KEY_ALARM_SILENCE   = 3,  /**< Silenciar alarma activa                      */
    UI_KEY_ENCODER_PRESS   = 4,  /**< Pulsación del eje del encoder rotativo       */
    UI_KEY_COUNT                 /**< Centinela — no usar como tecla                */
} UIKeyID_t;

/* ─── Tipos de evento ──────────────────────────────────────────────────────── */
typedef enum {
    UI_EVENT_KEY_PRESS     = 0,  /**< Flanco descendente detectado (debounce OK)           */
    UI_EVENT_KEY_RELEASE   = 1,  /**< Flanco ascendente detectado                          */
    UI_EVENT_KEY_HOLD      = 2,  /**< Tecla mantenida ≥ UI_KEY_HOLD_THRESHOLD_MS (1 vez)   */
    UI_EVENT_ENCODER_DELTA = 3,  /**< Rotación del encoder (1 evento por detent)           */
    UI_EVENT_TOUCH         = 4,  /**< Toque táctil validado en coordenadas de display      */
} UIEventType_t;

/* ─── Struct de evento ─────────────────────────────────────────────────────── */
/**
 * @brief Evento generado por task_ui_input y consumido por task_tft / task_hmi.
 *
 * Tamaño máximo del struct: 16 bytes (caben 16 eventos en ~256 bytes de RAM de cola).
 */
typedef struct {
    UIEventType_t type;          /**< Discriminador del union                               */
    uint32_t      timestamp_ms;  /**< xTaskGetTickCount() * portTICK_PERIOD_MS al generarse */

    union {
        /** Válido para UI_EVENT_KEY_PRESS, UI_EVENT_KEY_RELEASE, UI_EVENT_KEY_HOLD */
        struct {
            UIKeyID_t id;        /**< Qué tecla                                             */
            uint32_t  hold_ms;   /**< Tiempo de hold; válido solo en UI_EVENT_KEY_HOLD      */
        } key;

        /** Válido para UI_EVENT_ENCODER_DELTA */
        struct {
            int8_t delta;        /**< +1 horario (CW), -1 antihorario (CCW), 1 por detent  */
        } encoder;

        /** Válido para UI_EVENT_TOUCH */
        struct {
            uint16_t x;          /**< Coordenada X en píxeles (0 = borde izquierdo)        */
            uint16_t y;          /**< Coordenada Y en píxeles (0 = borde superior)          */
            bool     pressed;    /**< true = toque activo, false = dedo levantado           */
        } touch;
    };
} UIEvent_t;

/* ─── Verificación en compilación ─────────────────────────────────────────── */
/* UIEvent_t debe caber cómodamente en la cola — 20 bytes máx recomendado    */
_Static_assert(sizeof(UIEvent_t) <= 24, "UIEvent_t demasiado grande para cola FreeRTOS eficiente");

/* ─── Handle de la cola ────────────────────────────────────────────────────── */
/**
 * @brief Cola de eventos UI.  Definida en ui_events.c.
 *        Disponible para lectura por task_tft y task_hmi.
 */
extern QueueHandle_t ui_event_queue;

/* ─── API ──────────────────────────────────────────────────────────────────── */

/**
 * @brief Crea la cola de eventos UI.
 * @note  Llamar ANTES de xTaskCreate() de task_tft y task_ui_input,
 *        y ANTES de vTaskStartScheduler().
 */
void ui_events_init(void);

/**
 * @brief Envía un evento a la cola (non-blocking, safe desde task context).
 * @param event  Puntero al evento a enviar (se copia por valor).
 * @return true  si el evento fue encolado.
 * @return false si la cola estaba llena (evento descartado).
 * @note  Llamar SOLO desde task_ui_input. Un solo productor garantiza orden.
 */
bool ui_event_send(const UIEvent_t *event);

/**
 * @brief Variante para ISR de GPIO (si se adopta ISR en el futuro).
 * @param pxHigherPriorityTaskWoken  Pasar a portYIELD_FROM_ISR() si es true.
 */
bool ui_event_send_from_isr(const UIEvent_t        *event,
                             BaseType_t             *pxHigherPriorityTaskWoken);

#ifdef __cplusplus
}
#endif

#endif /* UI_EVENTS_H */
