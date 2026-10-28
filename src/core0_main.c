#include "core0_main.h"
#include "event_broker.h"
#include "hmi_consumer.h"
#include "mqtt_consumer.h"
#include "serial_consumer.h"
#include "system_queues.h"
#include "io_expander.h"
#include "mock_pcal9555.h"

// Descomentar o comentar esta linea para habilitar/deshabilitar el monitoreo de
// salud del sistema
// #define ENABLE_SYS_HEALTH_MONITOR

#include "FreeRTOS.h"
#include "config_manager.h"
#include "core1_main.h"
#include "system_config.h"
#include "crosscore_cmd.h"
#include "mqtt_client.h"
#include "mqtt_topics.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "cmd_dispatcher.h"
#include "cmd_gate.h"
#include "ui_events.h"
#include "ui_state.h"

#ifdef ENABLE_SYS_HEALTH_MONITOR
#include "hardware/adc.h"
#endif
#include "queue.h"
#include "semphr.h"
#include "syringe_pump_api.h"
#include "task.h"
#include <stdio.h>

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  serial_log("!ALM", "STACK_OVERFLOW task=%s", pcTaskName);
  __breakpoint(); // Detiene el debugger en este punto
  for (;;)
    ;
}

#define MAX_SCANNED_APS 16
#define WIFI_CONNECT_TIMEOUT_MS 20000
#define WIFI_SCAN_TIMEOUT_MS    5000

typedef struct {
  char ssid[MAX_SSID_LEN];
  int16_t rssi;
  uint8_t auth_mode;
  uint8_t channel;
} ScannedAp_t;

typedef struct {
  ScannedAp_t aps[MAX_SCANNED_APS];
  int count;
} ScanContext_t;

typedef struct {
  int profile_idx;
  int16_t rssi;
  bool visible_in_scan;
} WifiCandidate_t;

static TaskHandle_t s_wifi_task_handle = NULL;
static char s_forced_ssid[MAX_SSID_LEN] = {0};
static int16_t s_connected_rssi = 0;

// Buffers estáticos en .bss para evitar desbordar el stack de tareas FreeRTOS
static ScanContext_t s_scan_ctx;
static cyw43_wifi_scan_options_t s_scan_options;
static SemaphoreHandle_t s_scan_mutex = NULL;

static int wifi_scan_result_cb(void *env, const cyw43_ev_scan_result_t *result) {
  if (!result || result->ssid_len == 0) return 0;
  ScanContext_t *ctx = (ScanContext_t *)env;
  if (!ctx) return 0;

  // Sanear longitud máxima a 31 caracteres (+1 para null terminator)
  size_t len = result->ssid_len;
  if (len >= MAX_SSID_LEN) {
    len = MAX_SSID_LEN - 1;
  }

  char temp_ssid[MAX_SSID_LEN];
  memcpy(temp_ssid, result->ssid, len);
  temp_ssid[len] = '\0';

  // Evitar duplicados por múltiples BSSID/canales del mismo AP
  for (int i = 0; i < ctx->count; i++) {
    if (strcmp(ctx->aps[i].ssid, temp_ssid) == 0) {
      if (result->rssi > ctx->aps[i].rssi) {
        ctx->aps[i].rssi = result->rssi;
        ctx->aps[i].channel = result->channel;
      }
      return 0;
    }
  }

  if (ctx->count < MAX_SCANNED_APS) {
    memcpy(ctx->aps[ctx->count].ssid, temp_ssid, len + 1);
    ctx->aps[ctx->count].rssi = result->rssi;
    ctx->aps[ctx->count].auth_mode = result->auth_mode;
    ctx->aps[ctx->count].channel = result->channel;
    ctx->count++;
  }
  return 0;
}

static int wifi_scan_perform(uint32_t timeout_ms) {
  if (s_scan_mutex == NULL) {
    s_scan_mutex = xSemaphoreCreateMutex();
  }
  if (xSemaphoreTake(s_scan_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    serial_log("!NET", "wifi_scan=BUSY");
    return -1;
  }

  s_scan_ctx.count = 0;
  memset(&s_scan_options, 0, sizeof(s_scan_options));

  // Asegurar que no haya un escaneo previo colgado en el driver
  if (cyw43_wifi_scan_active(&cyw43_state)) {
    uint32_t wait_prev = 0;
    while (cyw43_wifi_scan_active(&cyw43_state) && wait_prev < 2000) {
      vTaskDelay(pdMS_TO_TICKS(50));
      wait_prev += 50;
    }
  }

  cyw43_arch_lwip_begin();
  int err = cyw43_wifi_scan(&cyw43_state, &s_scan_options, &s_scan_ctx, wifi_scan_result_cb);
  cyw43_arch_lwip_end();

  if (err != 0) {
    serial_log("!NET", "wifi_scan=FAIL err=%d", err);
    xSemaphoreGive(s_scan_mutex);
    return err;
  }

  uint32_t elapsed = 0;
  while (cyw43_wifi_scan_active(&cyw43_state)) {
    vTaskDelay(pdMS_TO_TICKS(100));
    elapsed += 100;
    if (elapsed >= timeout_ms) {
      break;
    }
  }

  // Si venció el timeout pero el hardware aún reporta escaneo activo, esperar a que cierre
  // para evitar colisiones con cyw43_wifi_join o cyw43_wifi_leave
  if (cyw43_wifi_scan_active(&cyw43_state)) {
    uint32_t wait_drain = 0;
    while (cyw43_wifi_scan_active(&cyw43_state) && wait_drain < 2000) {
      vTaskDelay(pdMS_TO_TICKS(100));
      wait_drain += 100;
    }
  }

  xSemaphoreGive(s_scan_mutex);
  return 0;
}

void wifi_manager_notify(void) {
  if (s_wifi_task_handle != NULL) {
    xTaskNotifyGive(s_wifi_task_handle);
  }
}

void wifi_manager_force_connect(const char *ssid) {
  if (!ssid || ssid[0] == '\0') return;
  strncpy(s_forced_ssid, ssid, MAX_SSID_LEN - 1);
  s_forced_ssid[MAX_SSID_LEN - 1] = '\0';
  serial_log("NET", "wifi=FORCE_REQ ssid='%s'", s_forced_ssid);
  cyw43_arch_lwip_begin();
  cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
  cyw43_arch_lwip_end();
  wifi_manager_notify();
}

void wifi_manager_scan_and_print(void) {
  serial_log("NET", "scan=START");
  int err = wifi_scan_perform(WIFI_SCAN_TIMEOUT_MS);
  if (err != 0) {
    serial_log("!NET", "scan=FAIL err=%d", err);
    return;
  }
  serial_log("NET", "scan count=%d", s_scan_ctx.count);
  for (int i = 0; i < s_scan_ctx.count; i++) {
    const char *sec = "OPEN";
    if (s_scan_ctx.aps[i].auth_mode == CYW43_AUTH_WPA2_AES_PSK ||
        s_scan_ctx.aps[i].auth_mode == CYW43_AUTH_WPA2_MIXED_PSK) {
      sec = "WPA2";
    } else if (s_scan_ctx.aps[i].auth_mode == CYW43_AUTH_WPA_TKIP_PSK) {
      sec = "WPA";
    }
    bool saved = (config_wifi_profile_find(s_scan_ctx.aps[i].ssid) != NULL);
    serial_log("NET", "scan ssid='%-20s' rssi=%d ch=%d sec=%s saved=%d",
               s_scan_ctx.aps[i].ssid,
               s_scan_ctx.aps[i].rssi,
               s_scan_ctx.aps[i].channel,
               sec,
               saved ? 1 : 0);
  }
}

/**
 * @brief Escanea redes y selecciona inteligentemente el perfil guardado con
 * mejor señal (mayor RSSI). Si se especificó una red forzada, conecta a esa directamente.
 */
static int wifi_connect_smart(void) {
  // Caso A: Se solicitó forzar un SSID específico
  if (s_forced_ssid[0] != '\0') {
    char target[MAX_SSID_LEN];
    strncpy(target, s_forced_ssid, sizeof(target));
    target[sizeof(target) - 1] = '\0';
    s_forced_ssid[0] = '\0'; // Consumir solicitud forzada

    const WifiProfile_t *prof = config_wifi_profile_find(target);
    const char *pw = NULL;
    const char *broker_ip = g_sys_config.mqtt_ip;
    uint16_t broker_port = g_sys_config.mqtt_port;

    if (prof != NULL) {
      pw = (prof->pass[0] != '\0') ? prof->pass : NULL;
      broker_ip = prof->mqtt_ip;
      broker_port = prof->mqtt_port;
    } else if (strcmp(target, g_sys_config.wifi_ssid) == 0) {
      pw = (g_sys_config.wifi_pass[0] != '\0') ? g_sys_config.wifi_pass : NULL;
    }

    uint32_t auth = (pw != NULL) ? CYW43_AUTH_WPA2_MIXED_PSK : CYW43_AUTH_OPEN;
    serial_log("NET", "wifi=TRY_FORCE ssid='%s'", target);

    cyw43_arch_lwip_begin();
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    cyw43_arch_lwip_end();
    vTaskDelay(pdMS_TO_TICKS(150));

    int err = cyw43_arch_wifi_connect_timeout_ms(target, pw, auth, WIFI_CONNECT_TIMEOUT_MS);
    if (err == 0) {
      config_wifi_profile_set_active(target, pw, broker_ip, broker_port);
      s_connected_rssi = 0;
      mqtt_client_force_reconnect();
      return 0;
    }
    serial_log("!NET", "wifi=FAIL_FORCE ssid='%s' err=%d", target, err);
    return -1;
  }

  // Caso B: Escanear redes disponibles y ordenar perfiles guardados por RSSI
  serial_log("NET", "wifi=SCAN");
  wifi_scan_perform(WIFI_SCAN_TIMEOUT_MS);

  // Pausa de 200 ms para que el driver CYW43 y el stack lwIP asienten el fin del escaneo
  vTaskDelay(pdMS_TO_TICKS(200));

  WifiCandidate_t candidates[MAX_WIFI_PROFILES];
  int cand_count = 0;

  for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
    if (!g_sys_config.wifi_profiles[i].valid) continue;

    const char *p_ssid = g_sys_config.wifi_profiles[i].ssid;
    int found_idx = -1;
    for (int s = 0; s < s_scan_ctx.count; s++) {
      if (strcmp(p_ssid, s_scan_ctx.aps[s].ssid) == 0) {
        found_idx = s;
        break;
      }
    }

    candidates[cand_count].profile_idx = i;
    if (found_idx >= 0) {
      candidates[cand_count].rssi = s_scan_ctx.aps[found_idx].rssi;
      candidates[cand_count].visible_in_scan = true;
      g_sys_config.wifi_profiles[i].last_rssi = (int8_t)candidates[cand_count].rssi;
    } else {
      // Red no vista en escaneo (ej. beacon miss por colisión de radio o SSID oculto)
      // Si coincide con la red activa configurada, priorizarla (-90 dBm) sobre perfiles no detectados (-100 dBm)
      if (strcmp(p_ssid, g_sys_config.wifi_ssid) == 0) {
        candidates[cand_count].rssi = -90;
      } else {
        candidates[cand_count].rssi = -100;
      }
      candidates[cand_count].visible_in_scan = false;
    }
    cand_count++;
  }

  if (cand_count == 0) {
    serial_log("!NET", "wifi=NO_PROFILES");
    return -1;
  }

  // Ordenar candidatos por RSSI descendente (mayor potencia de señal primero)
  for (int i = 0; i < cand_count - 1; i++) {
    for (int j = i + 1; j < cand_count; j++) {
      if (candidates[j].rssi > candidates[i].rssi) {
        WifiCandidate_t tmp = candidates[i];
        candidates[i] = candidates[j];
        candidates[j] = tmp;
      }
    }
  }

  serial_log("NET", "cand count=%d", cand_count);
  for (int i = 0; i < cand_count; i++) {
    int p_idx = candidates[i].profile_idx;
    serial_log("NET", "cand [%d/%d] ssid='%s' rssi=%d det=%d broker=%s:%d",
           i + 1, cand_count,
           g_sys_config.wifi_profiles[p_idx].ssid,
           candidates[i].rssi,
           candidates[i].visible_in_scan ? 1 : 0,
           g_sys_config.wifi_profiles[p_idx].mqtt_ip,
           g_sys_config.wifi_profiles[p_idx].mqtt_port);
  }

  // Intentar conectar en orden de potencia
  for (int i = 0; i < cand_count; i++) {
    int p_idx = candidates[i].profile_idx;
    WifiProfile_t *prof = &g_sys_config.wifi_profiles[p_idx];
    const char *pw = (prof->pass[0] != '\0') ? prof->pass : NULL;
    uint32_t auth = (pw != NULL) ? CYW43_AUTH_WPA2_MIXED_PSK : CYW43_AUTH_OPEN;

    serial_log("NET", "wifi=TRY [%d/%d] ssid='%s'", i + 1, cand_count, prof->ssid);

    cyw43_arch_lwip_begin();
    cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
    cyw43_arch_lwip_end();
    vTaskDelay(pdMS_TO_TICKS(150));

    int err = cyw43_arch_wifi_connect_timeout_ms(prof->ssid, pw, auth, WIFI_CONNECT_TIMEOUT_MS);
    if (err == 0) {
      config_wifi_profile_set_active(prof->ssid, prof->pass, prof->mqtt_ip, prof->mqtt_port);
      s_connected_rssi = (int8_t)candidates[i].rssi;
      mqtt_client_force_reconnect();
      return 0;
    }
    serial_log("!NET", "wifi=FAIL ssid='%s' err=%d", prof->ssid, err);
  }

  // Dejar la interfaz limpia si fallaron todos los candidatos
  cyw43_arch_lwip_begin();
  cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);
  cyw43_arch_lwip_end();

  return -1;
}

/**
 * @brief Tarea para mantener la conexion Wi-Fi
 */
static void wifi_keepalive_task(void *params) {
  s_wifi_task_handle = xTaskGetCurrentTaskHandle();

  while (1) {
    if (!g_sys_config.wifi_enabled) {
      // Wi-Fi deshabilitado (net_disable): esperar indefinidamente hasta notificación
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }

    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status == CYW43_LINK_UP && s_forced_ssid[0] == '\0') {
      // Regla de Histeresis / Estabilidad:
      // Mantener la conexión activa sin escanear ni cambiar de red mientras el enlace siga arriba.
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
      continue;
    }

    // Se perdió la conexión o se solicitó forzar un SSID específico
    CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)link_status);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    int err = wifi_connect_smart();
    if (err == 0) {
      CORE0_EMIT(EV_NET_WIFI_CONN, wifi,
                 ((DtoWifi_t){.rssi_dbm = s_connected_rssi, .channel = 0}));
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    } else {
      CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)err);
      serial_log("!NET", "wifi=NONE retry_in=60s");
      // Backoff de 60 segundos, interrumpible inmediatamente si el usuario añade red o reconecta
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000));
    }
  }
}

/**
 * @brief Tarea de inicializacion
 */
static void task_init(void *params) {
  config_manager_init();
  topics_init(g_sys_config.device_id);

  // Sincronizar umbrales ADC iniciales hacia Core 1
  cmd_send_set_adc_thresholds(g_sys_config.thresh_syringe_contact_v,
                              g_sys_config.thresh_occlusion_v,
                              g_sys_config.thresh_occ_release_v);

  // Sincronizar modo anti-bolus inicial hacia Core 1
  cmd_send_set_antibolus_mode(g_sys_config.auto_antibolus_enabled != 0);

  // Restaurar la calibracion persistida como valor activo. Core 1 arranca
  // antes de que la config este cargada, asi que se publica desde aqui
  // (escritura de 32 bits alineada — atomica entre cores).
  calibration_max_encoder_count = g_sys_config.calibration_valid
                                      ? g_sys_config.calibrated_max_encoder_count
                                      : MAX_TRAVEL_ENCODER_COUNT;
  CORE0_EMIT(EV_APP_CALIBRATION, calibration,
             ((DtoCalibration_t){
                 .max_encoder_count = calibration_max_encoder_count,
                 .travel_mm = calibration_max_encoder_count *
                              calc_um_per_pulse(USE_QUADRATURE_ENCODER) / 1000.0f,
                 .trigger = 0,
                 .success = g_sys_config.calibration_valid}));

  // Inicializacion del bus I2C0 y del expansor PCAL9555
  if (io_expander_init(NULL)) {
    io_expander_start_task(1);
    serial_log("SYS", "i2c_exp=OK addr=0x%02X", PCAL9555_I2C_ADDR_DEFAULT);
  } else {
    serial_log("!SYS", "i2c_exp=NOT_FOUND addr=0x%02X", PCAL9555_I2C_ADDR_DEFAULT);
#ifdef ENABLE_MOCK_PCAL9555
    mock_pcal9555_start_task(1);
#endif
  }

  // Inicializacion de GPIO y Wi-Fi chip (CYW43)
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)0xFF);
    vTaskDelete(NULL);
    return;
  }

  if (g_sys_config.wifi_enabled) {
    cyw43_arch_enable_sta_mode();
    vTaskDelay(pdMS_TO_TICKS(300)); // Estabilización del chip Wi-Fi tras activar STA
    CORE0_EMIT(EV_NET_WIFI_CONNECTING, param, 0); // SSID/pass not logged (security)
    int err = wifi_connect_smart();
    if (err == 0) {
      CORE0_EMIT(EV_NET_WIFI_CONN, wifi,
                 ((DtoWifi_t){.rssi_dbm = s_connected_rssi, .channel = 0}));
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    } else {
      CORE0_EMIT(EV_NET_WIFI_DISC, param, (uint32_t)err);
    }
  } else {
    cyw43_arch_disable_sta_mode();
    serial_log("NET", "wifi=DISABLED");
  }

  // Iniciar la tarea cliente MQTT
  xTaskCreate(mqtt_client_task, "MQTT_Task", configMINIMAL_STACK_SIZE * 4, NULL,
              2, NULL);

  xTaskCreate(wifi_keepalive_task, "WiFi_Keepalive", 1024 * 2, NULL, 1, &s_wifi_task_handle);

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
  char cli_buf[160];
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
        } else {
          printf("\n");
        }
      } else if (c == '\b' || c == 127) { // backspace
        if (cli_idx > 0) {
          cli_idx--;
          printf("\b \b");
        }
      } else if (c != '\0' && cli_idx < sizeof(cli_buf) - 1) {
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
#include "modulo_tft.h"
#include "lvgl.h"
#include "screens.h"
#include "vars.h"
#include "vars.h"

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
  modulo_tft_init();
  if (objects.bar_infusion) {
    lv_bar_set_range(objects.bar_infusion, 0, 100);
  }

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5);
  while (1) {
    modulo_tft_run_once();

    UIState_t state = ui_state_get_snapshot();
    bool update_progress = false;
    float progress_pct = 0.0f;
    if (state.fsm_state == ST_DISPENSING) {
      progress_pct = state.progress_pct;
      update_progress = true;
    } else if (state.fsm_state == ST_DISPENSE_COMPLETED) {
      progress_pct = 100.0f;
      update_progress = true;
    } else if (state.session_data_valid) {
      progress_pct = state.progress_pct;
      update_progress = true;
    }

    if (update_progress) {
      if (progress_pct < 0.0f) progress_pct = 0.0f;
      if (progress_pct > 100.0f) progress_pct = 100.0f;
      int32_t rounded_pct = (int32_t)(progress_pct + 0.5f);
      set_var_value_infusion_test((float)rounded_pct);
      if (objects.bar_infusion) {
        int32_t displayed_pct = lv_bar_get_value(objects.bar_infusion);
        if (displayed_pct != rounded_pct) {
          lv_anim_enable_t animation =
              state.fsm_state == ST_DISPENSE_COMPLETED ? LV_ANIM_OFF : LV_ANIM_ON;
          lv_bar_set_value(objects.bar_infusion, rounded_pct, animation);
        }
      }
    }

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
  cmd_gate_init();
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
  xTaskCreate(task_cli, "CLI", configMINIMAL_STACK_SIZE * 6, NULL, 1, NULL);
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
  xTaskCreate(task_tft, "TFT", 1024 * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
#endif
}
