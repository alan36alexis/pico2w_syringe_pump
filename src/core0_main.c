#include "core0_main.h"
#include "event_broker.h"
#include "hmi_consumer.h"
#include "mqtt_consumer.h"
#include "serial_consumer.h"
#include "system_queues.h"

// Descomentar o comentar esta linea para habilitar/deshabilitar el monitoreo de
// salud del sistema
// #define ENABLE_SYS_HEALTH_MONITOR

#include "FreeRTOS.h"
#include "config_manager.h"
#include "crosscore_cmd.h"
#include "mqtt_client.h"
#include "mqtt_topics.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "cmd_dispatcher.h"
#include "pump_hmi.h"
#include "ui_events.h"
#include "ui_state.h"

#ifdef ENABLE_SYS_HEALTH_MONITOR
#include "hardware/adc.h"
#endif
#include "queue.h"
#include "syringe_pump_api.h"
#include "task.h"
#include <stdio.h>

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  printf("\n[ERR]: *** STACK OVERFLOW en tarea: %s ***\n", pcTaskName);
  __breakpoint(); // Detiene el debugger en este punto
  for (;;)
    ;
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
      CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)link_status);
      int err = cyw43_arch_wifi_connect_timeout_ms(
          g_sys_config.wifi_ssid, g_sys_config.wifi_pass,
          CYW43_AUTH_WPA2_MIXED_PSK, 30000);
      if (err) {
        CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)err);
      } else {
        CORE0_EMIT(EV_NET_WIFI_CONN, wifi,
                   ((DtoWifi_t){.rssi_dbm = 0, .channel = 0}));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

/**
 * @brief Tarea de inicializacion
 */
static void task_init(void *params) {
  config_manager_init();

  // Inicializacion de GPIO y Wi-Fi chip (CYW43)
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)0xFF);
    vTaskDelete(NULL);
    return;
  }

  if (g_sys_config.wifi_enabled) {
    cyw43_arch_enable_sta_mode();
    CORE0_EMIT(EV_NET_WIFI_CONNECTING, param, 0); // SSID/pass not logged (security)
    int err = cyw43_arch_wifi_connect_timeout_ms(
        g_sys_config.wifi_ssid, g_sys_config.wifi_pass,
        CYW43_AUTH_WPA2_MIXED_PSK, 30000);
    if (err) {
      CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)err);
    } else {
      CORE0_EMIT(EV_NET_WIFI_CONN, wifi,
                 ((DtoWifi_t){.rssi_dbm = 0, .channel = 0}));
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
  } else {
    cyw43_arch_disable_sta_mode();
    printf("[NET]: Wi-Fi deshabilitado (Battery Save Mode).\n");
  }

  // Iniciar la tarea cliente MQTT
  xTaskCreate(mqtt_client_task, "MQTT_Task", configMINIMAL_STACK_SIZE * 4, NULL,
              2, NULL);

  xTaskCreate(wifi_keepalive_task, "WiFi_Keepalive", 1024, NULL, 1, NULL);

  // Elimino la tarea para liberar recursos tras una única ejecución
  vTaskDelete(NULL);
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
 * @brief Tarea para procesar comandos via Serial (CLI)
 */
static void task_cli(void *params) {
  char cli_buf[64];
  int cli_idx = 0;

  CORE0_EMIT(EV_SYS_CLI_READY, param, 0);

  while (1) {
    int c = getchar_timeout_us(20);
    if (c != PICO_ERROR_TIMEOUT) {
      if (c == '\r' || c == '\n') {
        if (cli_idx > 0) {
          cli_buf[cli_idx] = '\0';
          printf("\n"); // Echo newline
          cmd_dispatch_string(cli_buf, CMD_SRC_SERIAL, -1);
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
      vTaskDelay(pdMS_TO_TICKS(
          2)); // Delay to allow other tasks to run. Reduced to 2ms to prevent
               // 32-byte UART FIFO overflow at 115200 baud.
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
    printf("\n--- System Health ---\n");
    printf("CPU Temp : %.2f C\n", temp_c);
    // printf("Free Heap: %u bytes (Min: %u bytes)\n", free_heap,
    // min_free_heap);

    CORE0_EMIT(EV_SYS_HEAP_UPD, sys_health, ((DtoSysHealth_t){
        .free_heap_bytes = free_heap,
        .min_ever_heap_bytes = min_free_heap,
        .mqtt_tx_drops = (uint8_t)mqtt_get_tx_drops(),
        .mqtt_rx_drops = 0
    }));

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
      printf("\n[Task Name]      [Least Free Stack] (Words)\n");
      for (UBaseType_t i = 0; i < num_tasks; i++) {
        printf("%-16s %u\n", pxTaskStatusArray[i].pcTaskName,
               pxTaskStatusArray[i].usStackHighWaterMark);
      }
      vPortFree(pxTaskStatusArray);
    } else {
      printf("Could not allocate memory for task status.\n");
    }
    printf("---------------------\n\n");

    vTaskDelay(pdMS_TO_TICKS(10000)); // Repetir cada 10 segundos
  }
}
#endif

#ifdef ENABLE_TFT
#include "display_driver.h"
#include "encoder_driver.h"
#include "touch_driver.h"
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
  // NOTE: system_queues_init() is called from main.c before
  // multicore_launch_core1() so that g_crosscore_event_q is ready before Core 1
  // uses CORE1_EMIT.

  crosscore_cmd_init();
  mqtt_client_queue_init();
  pump_hmi_init();
  ui_events_init();
  ui_state_init();

  // EDA broker and consumers
  event_broker_start();
  serial_consumer_start();
  mqtt_consumer_start();
  hmi_consumer_start();

  // Creación de las tareas de FreeRTOS
  xTaskCreate(task_init, "Init", 1024, NULL, 2, NULL);
  xTaskCreate(task_blinky, "Blinky", configMINIMAL_STACK_SIZE * 4, NULL, 1,
              NULL);
  xTaskCreate(task_cli, "CLI", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
  // mqtt_msg_t (324 B) + cmd_buf[128] + sscanf internals + publish_qos1 frame =
  // ~500 words peak
  xTaskCreate(mqtt_rx_task, "MQTT_Rx", configMINIMAL_STACK_SIZE * 8, NULL, 2,
              NULL);
#ifdef ENABLE_SYS_HEALTH_MONITOR
  xTaskCreate(task_system_monitor, "SysMon", configMINIMAL_STACK_SIZE * 3, NULL,
              1, NULL);
#endif
  xTaskCreate(task_example_internal_cmd, "CmdExample",
              configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
#ifdef ENABLE_TFT
  // task_ui_input: prioridad más alta para capturar input antes del frame LVGL
  xTaskCreate(task_ui_input, "UI_Input", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
  // task_tft: 4096 words (16 KB) — LVGL necesita stack generoso
  xTaskCreate(task_tft, "TFT", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);
#endif
}
