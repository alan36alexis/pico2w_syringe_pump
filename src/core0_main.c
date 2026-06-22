#include "core0_main.h"
#include "system_config.h"

// Descomentar o comentar esta linea para habilitar/deshabilitar el monitoreo de
// salud del sistema
// #define ENABLE_SYS_HEALTH_MONITOR

#include "FreeRTOS.h"
#include "config_manager.h"
#include "crosscore_cmd.h"
#include "crosscore_logger.h"
#include "mqtt_client.h"
#include "mqtt_topics.h"
#include "pump_hmi.h"
#include "ui_state.h"
#include "ui_events.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#ifdef ENABLE_SYS_HEALTH_MONITOR
#include "hardware/adc.h"
#endif
#include "queue.h"
#include "syringe_pump_api.h"
#include "task.h"
#include <stdarg.h>
#include <stdio.h>

static void wifi_keepalive_task(void *params);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  printf("\n[ERR]: *** STACK OVERFLOW en tarea: %s ***\n", pcTaskName);
  __breakpoint(); // Detiene el debugger en este punto
  for (;;);
}

#define PRINT_QUEUE_LENGTH 15
#define PRINT_MSG_MAX_LEN 128
static QueueHandle_t print_q = NULL;

/**
 * @brief Thread-safe proxy para mandar strings al logger centralizado.
 */
void safe_printf(const char *fmt, ...) {
  if (print_q == NULL)
    return;
  char buf[PRINT_MSG_MAX_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  xQueueSend(print_q, buf,
             0); // Si está llena, se descarta el log (non-blocking)
}

/**
 * @brief Tarea de inicializacion
 */
static void task_init(void *params) {
  config_manager_init();

  // Inicializacion de GPIO y Wi-Fi chip (CYW43)
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    safe_printf("[NET]: Wi-Fi init failed\n");
    vTaskDelete(NULL);
    return;
  }

  if (g_sys_config.wifi_enabled) {
    cyw43_arch_enable_sta_mode();
    safe_printf("[NET]: Connecting to Wi-Fi SSID: [%s], PASS: [%s]...\n", g_sys_config.wifi_ssid, g_sys_config.wifi_pass);
    int err = cyw43_arch_wifi_connect_timeout_ms(g_sys_config.wifi_ssid,
                                           g_sys_config.wifi_pass,
                                           CYW43_AUTH_WPA2_MIXED_PSK, 30000);
    if (err) {
      safe_printf("[NET]: Failed to connect to Wi-Fi on boot. Error: %d. Keepalive task will retry.\n", err);
    } else {
      safe_printf("[NET]: Connected to Wi-Fi.\n");
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
  } else {
    cyw43_arch_disable_sta_mode();
    safe_printf("[NET]: Wi-Fi is disabled by configuration (Battery Save Mode).\n");
  }

  // Iniciar la tarea cliente MQTT
  xTaskCreate(mqtt_client_task, "MQTT_Task", configMINIMAL_STACK_SIZE * 4, NULL,
              2, NULL);

  xTaskCreate(wifi_keepalive_task, "WiFi_Keepalive", 1024,
              NULL, 1, NULL);

  // Elimino la tarea para liberar recursos tras una única ejecución
  vTaskDelete(NULL);
}

/**
 * @brief Tarea para mantener la conexion Wi-Fi
 */
static void wifi_keepalive_task(void *params) {
  while (1) {
    if (!g_sys_config.wifi_enabled) {
      vTaskDelay(pdMS_TO_TICKS(10000));
      continue;
    }
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status != CYW43_LINK_UP) {
      safe_printf("[NET]: Wi-Fi disconnected (status: %d). Reconnecting to %s...\n",
                  link_status, g_sys_config.wifi_ssid);
      int err = cyw43_arch_wifi_connect_timeout_ms(g_sys_config.wifi_ssid,
                                             g_sys_config.wifi_pass,
                                             CYW43_AUTH_WPA2_MIXED_PSK, 30000);
      if (err) {
        safe_printf("[NET]: Failed to reconnect to Wi-Fi. Error: %d\n", err);
      } else {
        safe_printf("[NET]: Reconnected to Wi-Fi.\n");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

/**
 * @brief Tarea de blinky de LED indicadora de salud de conexión
 */
static void task_blinky(void *params) {
  while (1) {
    if (!g_sys_config.wifi_enabled) {
      // Latido muy lento: 30ms encendido, 2970ms apagado (Ahorro de bateria)
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
      vTaskDelay(pdMS_TO_TICKS(30));
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(2970));
    } else {
      // Verificar estado de conexión Wi-Fi
      int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
      uint32_t delay_ms = (link_status == CYW43_LINK_UP) ? 100 : 1000;

      // Toggle del LED de la placa Pico W
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,
                          !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));

      // Demora según estado
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }
}

/**
 * @brief Tarea para procesar los logs provenientes del Core 1
 */
static void task_logger(void *params) {
  LogMessage_t msg;
  char buf[256];
  char print_msg[PRINT_MSG_MAX_LEN];
  static Core1State_t s_prev_fsm_state = (Core1State_t)0;
  while (1) {
    // Procesa peticiones de impresión procedentes de otras tareas del Core 0
    while (xQueueReceive(print_q, print_msg, 0) == pdTRUE) {
      printf("%s", print_msg);
    }

    // Persist calibration if Core 1 flagged a new result
    if (g_calibration_dirty) {
        g_calibration_dirty = false;
        config_manager_save(true);
        safe_printf("[CFG]: Calibration saved to Flash.\n");
    }

    // Verifica si hay mensajes en la cola desde el Core 1
    while (queue_try_remove(&crosscore_log_queue, &msg)) {
      buf[0] = '\0';
      switch (msg.id) {
      case LOG_EVENT_HEARTBEAT:
        printf("[SYS]: Core 1 counter: %u\n", msg.payload.counter);
        snprintf(buf, sizeof(buf), "Core 1 counter: %u", msg.payload.counter);
        break;
      case LOG_EVENT_PRESSURE_UPDATE:
        printf("[PRS]: Status: OK, Pressure: %.2f psi (%.2f mmHg)\n",
               msg.payload.pressure_psi, msg.payload.pressure_psi * 51.7149f);
        snprintf(buf, sizeof(buf), "Status: OK, Pressure: %.2f psi (%.2f mmHg)",
                 msg.payload.pressure_psi, msg.payload.pressure_psi * 51.7149f);

        // Feed real-time pressure to API
        Pump_UpdatePressure(msg.payload.pressure_psi * 51.7149f);
        break;
      case LOG_EVENT_PRESSURE_ALERT:
        printf("[PRS]: ALERTA: Sobrepresion (%.2f PSI). Frenando para retroceder!\n",
               msg.payload.pressure_psi);
        snprintf(buf, sizeof(buf),
                 "ALERTA: Sobrepresion (%.2f PSI). Frenando para retroceder!",
                 msg.payload.pressure_psi);
        break;
      case LOG_EVENT_PRESSURE_SAFE:
        printf("[PRS]: Presion segura (%.2f PSI). Deteniendo definitivamente.\n",
               msg.payload.pressure_psi);
        snprintf(buf, sizeof(buf),
                 "Presion segura (%.2f PSI). Deteniendo definitivamente.",
                 msg.payload.pressure_psi);
        break;
      case LOG_EVENT_MOTOR_STOPPED:
        if (g_log_filter.show_mtr) printf("[MTR]: Motor detenido.\n");
        snprintf(buf, sizeof(buf), "Motor detenido.");
        break;
      case LOG_EVENT_MOTOR_RETRACTING:
        printf("[MTR]: Retroceso iniciado...\n");
        snprintf(buf, sizeof(buf), "Retroceso iniciado...");
        break;
      case LOG_EVENT_MOTOR_START_HIT:
        printf("[MTR]: Tope INICIO alcanzado. Iniciando frenado suave...\n");
        snprintf(buf, sizeof(buf),
                 "Tope INICIO alcanzado. Iniciando frenado suave...");
        break;
      case LOG_EVENT_MOTOR_END_HIT:
        printf("[MTR]: Tope FIN alcanzado. Iniciando frenado suave...\n");
        snprintf(buf, sizeof(buf),
                 "Tope FIN alcanzado. Iniciando frenado suave...");
        break;
      case LOG_EVENT_MOTOR_STALL:
        printf("$%u;\n", msg.payload.motor_status.stall);
        snprintf(buf, sizeof(buf), "$%u;", msg.payload.motor_status.stall);
        break;
      case LOG_EVENT_DRV_STATUS_ERROR:
        snprintf(buf, sizeof(buf), "Stall: %u | DRV: 0x%X | GSTAT: 0x%X",
                 msg.payload.motor_status.stall,
                 msg.payload.motor_status.drv_status,
                 msg.payload.motor_status.gstat);
        break;
      case LOG_EVENT_UART_INIT_OK:
        printf("[MTR]: Conexion UART Exitosa. IOIN: 0x%08X\n", msg.payload.raw_data);
        snprintf(buf, sizeof(buf), "Conexion UART Exitosa. IOIN: 0x%08X",
                 msg.payload.raw_data);
        break;
      case LOG_EVENT_UART_INIT_FAIL:
        printf("[MTR]: ERROR CRITICO: No hay comunicacion UART (Lectura = 0).\n");
        printf("[MTR]: Revisar conexion TX/RX y alimentacion VM.\n");
        snprintf(buf, sizeof(buf),
                 "ERROR CRITICO: No hay comunicacion UART (Lectura = 0). "
                 "Revisar conexion TX/RX y alimentacion VM.");
        break;
      case LOG_EVENT_UART_INIT_MICROSTEPS_READ:
        printf("[MTR]: Microsteps Leido: %u\n", msg.payload.raw_data);
        snprintf(buf, sizeof(buf), "Microsteps Leido: %u",
                 msg.payload.raw_data);
        break;
      case LOG_EVENT_PINS_INIT_MODE:
        printf("[MTR]: Iniciando en MODO PINES (Pines MS usados para microstepping)\n");
        snprintf(
            buf, sizeof(buf),
            "Iniciando en MODO PINES (Pines MS usados para microstepping)");
        break;
      case LOG_EVENT_GENERAL_DEBUG:
        printf("[SYS]: Debug raw: %u\n", msg.payload.raw_data);
        snprintf(buf, sizeof(buf), "Debug raw: %u", msg.payload.raw_data);
        break;
      case LOG_EVENT_MOTOR_MOVING:
        printf("[MTR]: Motor en movimiento...\n");
        snprintf(buf, sizeof(buf), "Motor en movimiento...");
        break;
      case LOG_EVENT_STRING_MSG:
        printf("%s\n", msg.payload.msg_str);
        snprintf(buf, sizeof(buf), "%s", msg.payload.msg_str);
        break;
      case LOG_EVENT_ENCODER_UPDATE: {
        float displacement_um = msg.payload.encoder_count * calc_um_per_pulse(USE_QUADRATURE_ENCODER);
        if (g_log_filter.show_enc) {
            printf("[ENC]: Encoder Val: %d steps, %.2f um\n", msg.payload.encoder_count, displacement_um);
        }
        snprintf(buf, sizeof(buf), "Encoder Val: %d steps, %.2f um",
                 msg.payload.encoder_count, displacement_um);
        break;
      }
      case LOG_EVENT_ENCODER_INDEP_UPDATE:
        if (g_log_filter.show_enc) {
            printf("[ENC]: Encoder Indep: A=%d B=%d\n",
                   msg.payload.encoder_indep_count.count_a,
                   msg.payload.encoder_indep_count.count_b);
        }
        snprintf(buf, sizeof(buf), "{\"A\": %d, \"B\": %d}",
                 msg.payload.encoder_indep_count.count_a,
                 msg.payload.encoder_indep_count.count_b);
        break;
      case LOG_EVENT_ENCODER_SPEED:
        if (g_log_filter.show_enc) {
            printf("[ENC]: Encoder Speed: %.2f um/s\n", msg.payload.encoder_speed);
        }
        snprintf(buf, sizeof(buf), "{\"speed_ums\": %.2f}",
                 msg.payload.encoder_speed);
        break;
      case LOG_EVENT_SPEED_WARNING:
        if (g_log_filter.show_enc) {
            printf("[ENC]: WARNING: Speed deviation (expected: %.2f um/s, actual: %.2f "
                   "um/s)\n",
                   msg.payload.speed_warning.expected_ums,
                   msg.payload.speed_warning.actual_ums);
        }
        snprintf(buf, sizeof(buf),
                 "{\"warning\": \"Speed Deviation\", \"expected_ums\": %.2f, "
                 "\"actual_ums\": %.2f}",
                 msg.payload.speed_warning.expected_ums,
                 msg.payload.speed_warning.actual_ums);
        break;
      case LOG_EVENT_CORRECTION_APPLIED:
        printf("[ENC]: Semi-Closed Loop correction applied (missing: %.2f um)\n",
               msg.payload.correction_um);
        snprintf(buf, sizeof(buf), "{\"correction_applied_um\": %.2f}",
                 msg.payload.correction_um);
        break;
      case LOG_EVENT_MOTOR_PROGRESS:
        snprintf(buf, sizeof(buf), "{\"progress_pct\": %.1f}",
                 msg.payload.progress_pct);
        break;
      case LOG_EVENT_FSM_STATE:
        if (g_log_filter.show_fsm)
          printf("[FSM]: -> %s\n", get_state_name(msg.payload.fsm_state));
        snprintf(buf, sizeof(buf), "{\"type\":\"state\",\"from\":%d,\"to\":%d}",
                 (int)s_prev_fsm_state, (int)msg.payload.fsm_state);
        s_prev_fsm_state = msg.payload.fsm_state;
        break;
      default:
        printf("[SYS]: Unknown crosscore logger event: %d\n", msg.id);
        snprintf(buf, sizeof(buf), "Unknown crosscore logger event: %d",
                 msg.id);
        break;
      }

      // Actualizar el mirror de estado para la UI (task_tft lo leerá via ui_state_get_snapshot)
      ui_state_update_from_event(&msg);

      // Solo los eventos FSM van al dashboard; los logs diagnósticos son solo serial
      if (msg.id == LOG_EVENT_FSM_STATE) {
        mqtt_client_publish_qos1(topic_event(), buf);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Poll cada 10ms
  }
}

/**
 * @brief Tarea de ejemplo interno (mantenida para uso interno/testing)
 */
static void task_example_internal_cmd(void *params) {
  // Inicializar API Clínica y configurar jeringa genérica de ejemplo
  Pump_Init();
  Pump_SelectSyringe(19.13f, 20.0f);

  vTaskDelay(pdMS_TO_TICKS(5000));

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

/**
 * @brief Tarea para reportar telemetría JSON
 */
static void task_pump_telemetry(void *params) {
  char json_buf[256];
  const uint32_t telemetry_period_ms = 2000;

  while (1) {
    // Calculadora temporal: Avanza el volumen y contador de tiempo 2000 ms = 2
    // seg
    Pump_Tick(telemetry_period_ms);
    Pump_GetTelemetryJSON(json_buf, sizeof(json_buf));
    mqtt_client_publish(topic_telemetry(), json_buf);
    vTaskDelay(pdMS_TO_TICKS(telemetry_period_ms)); // Publicar cada 2 segundos
  }
}

/**
 * @brief Tarea para procesar comandos via Serial (CLI)
 */
static void task_cli(void *params) {
  char cli_buf[64];
  int cli_idx = 0;

  safe_printf("\nPico CLI Ready. Waiting for commands...\n");

  while (1) {
    int c = getchar_timeout_us(20);
    if (c != PICO_ERROR_TIMEOUT) {
      if (c == '\r' || c == '\n') {
        if (cli_idx > 0) {
          cli_buf[cli_idx] = '\0';
          printf("\n"); // Echo newline
          pump_hmi_parse_and_execute(cli_buf);
          cli_idx = 0;
        }
      } else if (c == '\b' || c == 127) { // backspace
        if (cli_idx > 0) {
          cli_idx--;
          printf("\b \b");
        }
      } else if (cli_idx < sizeof(cli_buf) - 1) {
        cli_buf[cli_idx++] = (char)c;
        putchar(c); // Echo character
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(2)); // Delay to allow other tasks to run. Reduced to 2ms to prevent 32-byte UART FIFO overflow at 115200 baud.
    }
  }
}

#ifdef ENABLE_SYS_HEALTH_MONITOR
/**
 * @brief Tarea para monitorear la salud del sistema (Temp, RAM, Stack de
 * tareas)
 */
static void task_system_monitor(void *params) {
  // Inicializar ADC y lectura de sensor de temperatura
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);

  char json_buf[128];

  while (1) {
    // 1. Lectura de temperatura del procesador RP2040
    uint16_t result = adc_read();
    float voltage = result * 3.3f / (1 << 12);
    // Formula para RP2040: T = 27 - (V - 0.706) / 0.001721
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    // 2. Lectura de Heap Global FreeRTOS
    // Al usar heap_3.c, FreeRTOS usa el malloc estandar de stdlib y no tiene
    // metricas de xPortGetFreeHeapSize activas de forma nativa sin mallinfo.
    uint32_t free_heap = 0;
    uint32_t min_free_heap = 0;

    // 3. Imprimir marcas de agua del stack por tarea en UART
    safe_printf("\n--- System Health ---\n");
    safe_printf("CPU Temp : %.2f C\n", temp_c);
    // safe_printf("Free Heap: %u bytes (Min: %u bytes)\n", free_heap,
    // min_free_heap);

    // Obtener y mostrar el High Water Mark de cada tarea (stack restante minimo
    // historico en words)
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *pxTaskStatusArray =
        pvPortMalloc(num_tasks * sizeof(TaskStatus_t));
    if (pxTaskStatusArray != NULL) {
      uint32_t total_run_time;
      // Obtener el estado del array. Nota: Como configGENERATE_RUN_TIME_STATS
      // es 0, el run time puede ser omitido.
      num_tasks =
          uxTaskGetSystemState(pxTaskStatusArray, num_tasks, &total_run_time);
      safe_printf("\n[Task Name]      [Least Free Stack] (Words)\n");
      for (UBaseType_t i = 0; i < num_tasks; i++) {
        safe_printf("%-16s %u\n", pxTaskStatusArray[i].pcTaskName,
                    pxTaskStatusArray[i].usStackHighWaterMark);
      }
      vPortFree(pxTaskStatusArray);
    } else {
      safe_printf("Could not allocate memory for task status.\n");
    }
    safe_printf("---------------------\n\n");

    vTaskDelay(pdMS_TO_TICKS(10000)); // Repetir cada 10 segundos
  }
}
#endif

#ifdef ENABLE_TFT
#include "display_driver.h"
#include "touch_driver.h"
#include "encoder_driver.h"
#include "ui.h"

static void task_ui_input(void *params) {
  (void)params;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);
  while (1) {
    // TODO: leer encoder delta, estados de teclas (debounce) y touch (XPT2046)
    // y enviar UIEvent_t via ui_event_send().
    vTaskDelayUntil(&last_wake, period);
  }
}

static void task_tft(void *params) {
  (void)params;
  lv_init();
  display_driver_init();
  touch_driver_init();
  encoder_driver_init();
  ui_init();

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(20);
  while (1) {
    lv_tick_inc(20);
    lv_timer_handler();

    UIState_t state = ui_state_get_snapshot();
    (void)state; // TODO: actualizar widgets LVGL con state

    UIEvent_t ev;
    while (xQueueReceive(ui_event_queue, &ev, 0) == pdTRUE) {
      if (ev.type == UI_EVENT_TOUCH) {
        // TODO: pasar coordenadas calibradas al driver de input de LVGL
      }
    }

    vTaskDelayUntil(&last_wake, period);
  }
}
#endif // ENABLE_TFT

/**
 * @brief Función para configurar todas las tareas del Core 0 antes de iniciar
 * FreeRTOS
 */
void core0_main_setup(void) {
  // Inicializar colas y frameworks de logeo
  print_q = xQueueCreate(PRINT_QUEUE_LENGTH, PRINT_MSG_MAX_LEN);
  crosscore_logger_init();
  crosscore_cmd_init();
  mqtt_client_queue_init();
  pump_hmi_init();
  ui_events_init();
  ui_state_init();

  // Creación de las tareas de FreeRTOS
  xTaskCreate(task_init, "Init", 1024, NULL, 2, NULL);
  xTaskCreate(task_blinky, "Blinky", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
  xTaskCreate(task_logger, "Logger", configMINIMAL_STACK_SIZE * 5, NULL, 1,
              NULL);
  xTaskCreate(task_pump_telemetry, "Telemetry", configMINIMAL_STACK_SIZE * 4,
              NULL, 1, NULL);
  xTaskCreate(task_cli, "CLI", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
  // mqtt_msg_t (324 B) + cmd_buf[128] + sscanf internals + publish_qos1 frame = ~500 words peak
  xTaskCreate(mqtt_rx_task, "MQTT_Rx", configMINIMAL_STACK_SIZE * 8, NULL, 2,
              NULL);
#ifdef ENABLE_SYS_HEALTH_MONITOR
  xTaskCreate(task_system_monitor, "SysMon", configMINIMAL_STACK_SIZE * 3, NULL,
              1, NULL);
#endif
  xTaskCreate(task_example_internal_cmd, "CmdExample", configMINIMAL_STACK_SIZE * 2,
              NULL, 1, NULL);
#ifdef ENABLE_TFT
  // task_ui_input: prioridad más alta para capturar input antes del frame LVGL
  xTaskCreate(task_ui_input, "UI_Input", 512,  NULL, tskIDLE_PRIORITY + 3, NULL);
  // task_tft: 4096 words (16 KB) — LVGL necesita stack generoso
  xTaskCreate(task_tft,      "TFT",      4096, NULL, tskIDLE_PRIORITY + 2, NULL);
#endif
}
