#include "core0_main.h"
#include "FreeRTOS.h"
#include "mqtt_client.h"
#include "crosscore_cmd.h"
#include "crosscore_logger.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdarg.h>
#include "syringe_pump_api.h"

static void wifi_keepalive_task(void *params);

#define PRINT_QUEUE_LENGTH 15
#define PRINT_MSG_MAX_LEN 128
static QueueHandle_t print_q = NULL;

/**
 * @brief Thread-safe proxy para mandar strings al logger centralizado.
 */
void safe_printf(const char *fmt, ...) {
    if (print_q == NULL) return;
    char buf[PRINT_MSG_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    xQueueSend(print_q, buf, 0); // Si está llena, se descarta el log (non-blocking)
}

/**
 * @brief Tarea de inicializacion
 */
static void task_init(void *params) {
  // Inicializacion de GPIO y Wi-Fi chip (CYW43)
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    safe_printf("Wi-Fi init failed\n");
    vTaskDelete(NULL);
    return;
  }
  
  cyw43_arch_enable_sta_mode();
  safe_printf("Connecting to Wi-Fi (%s)...\n", WIFI_SSID);
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
      safe_printf("Failed to connect to Wi-Fi.\n");
      vTaskDelete(NULL);
      return;
  }
  safe_printf("Connected to Wi-Fi.\n");
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

  // Iniciar la tarea cliente MQTT
  xTaskCreate(mqtt_client_task, "MQTT_Task", configMINIMAL_STACK_SIZE * 4, NULL, 2, NULL);

  xTaskCreate(wifi_keepalive_task, "WiFi_Keepalive", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // Elimino la tarea para liberar recursos tras una única ejecución
  vTaskDelete(NULL);
}

/**
 * @brief Tarea para mantener la conexion Wi-Fi
 */
static void wifi_keepalive_task(void *params) {
  while (1) {
    int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (link_status != CYW43_LINK_UP) {
      safe_printf("Wi-Fi disconnected (status: %d). Reconnecting...\n", link_status);
      if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
          safe_printf("Failed to reconnect to Wi-Fi.\n");
      } else {
          safe_printf("Reconnected to Wi-Fi.\n");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

/**
 * @brief Tarea de blinky de LED
 */
static void task_blinky(void *params) {
  while (1) {
    // Toggle del LED de la placa Pico W
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,
                        !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
    // Demora de ticks equivalentes a 500 ms
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/**
 * @brief Tarea para procesar los logs provenientes del Core 1
 */
static void task_logger(void *params) {
  LogMessage_t msg;
  char buf[256];
  char print_msg[PRINT_MSG_MAX_LEN];
  while (1) {
    // Procesa peticiones de impresión procedentes de otras tareas del Core 0
    while (xQueueReceive(print_q, print_msg, 0) == pdTRUE) {
      printf("%s", print_msg);
    }
    
    // Verifica si hay mensajes en la cola desde el Core 1
    while (queue_try_remove(&crosscore_log_queue, &msg)) {
      buf[0] = '\0';
      switch (msg.id) {
      case LOG_EVENT_HEARTBEAT:
        printf("Core 1 counter: %u\n", msg.payload.counter);
        snprintf(buf, sizeof(buf), "Core 1 counter: %u", msg.payload.counter);
        break;
      case LOG_EVENT_PRESSURE_UPDATE:
        printf("Status: OK, Pressure: %.2f psi (%.2f mmHg)\n",
               msg.payload.pressure_psi, msg.payload.pressure_psi * 51.7149f);
        snprintf(buf, sizeof(buf), "Status: OK, Pressure: %.2f psi (%.2f mmHg)",
               msg.payload.pressure_psi, msg.payload.pressure_psi * 51.7149f);
        
        // Feed real-time pressure to API
        Pump_UpdatePressure(msg.payload.pressure_psi * 51.7149f);
        break;
      case LOG_EVENT_PRESSURE_ALERT:
        printf("ALERTA: Sobrepresion (%.2f PSI). Frenando para retroceder!\n",
               msg.payload.pressure_psi);
        snprintf(buf, sizeof(buf), "ALERTA: Sobrepresion (%.2f PSI). Frenando para retroceder!",
               msg.payload.pressure_psi);
        break;
      case LOG_EVENT_PRESSURE_SAFE:
        printf("Presion segura (%.2f PSI). Deteniendo definitivamente.\n",
               msg.payload.pressure_psi);
        snprintf(buf, sizeof(buf), "Presion segura (%.2f PSI). Deteniendo definitivamente.",
               msg.payload.pressure_psi);
        break;
      case LOG_EVENT_MOTOR_STOPPED:
        printf("Motor detenido. Iniciando retroceso...\n");
        snprintf(buf, sizeof(buf), "Motor detenido. Iniciando retroceso...");
        break;
      case LOG_EVENT_MOTOR_RETRACTING:
        printf("Retroceso iniciado...\n");
        snprintf(buf, sizeof(buf), "Retroceso iniciado...");
        break;
      case LOG_EVENT_MOTOR_START_HIT:
        printf("Tope INICIO alcanzado. Iniciando frenado suave...\n");
        snprintf(buf, sizeof(buf), "Tope INICIO alcanzado. Iniciando frenado suave...");
        break;
      case LOG_EVENT_MOTOR_END_HIT:
        printf("Tope FIN alcanzado. Iniciando frenado suave...\n");
        snprintf(buf, sizeof(buf), "Tope FIN alcanzado. Iniciando frenado suave...");
        break;
      case LOG_EVENT_MOTOR_STALL:
        printf("$%u;\n", msg.payload.motor_status.stall);
        snprintf(buf, sizeof(buf), "$%u;", msg.payload.motor_status.stall);
        break;
      case LOG_EVENT_DRV_STATUS_ERROR:
        printf("Stall: %u | DRV: 0x%X | GSTAT: 0x%X\n",
               msg.payload.motor_status.stall,
               msg.payload.motor_status.drv_status,
               msg.payload.motor_status.gstat);
        snprintf(buf, sizeof(buf), "Stall: %u | DRV: 0x%X | GSTAT: 0x%X",
               msg.payload.motor_status.stall,
               msg.payload.motor_status.drv_status,
               msg.payload.motor_status.gstat);
        break;
      case LOG_EVENT_UART_INIT_OK:
        printf("Conexion UART Exitosa. IOIN: 0x%08X\n", msg.payload.raw_data);
        snprintf(buf, sizeof(buf), "Conexion UART Exitosa. IOIN: 0x%08X", msg.payload.raw_data);
        break;
      case LOG_EVENT_UART_INIT_FAIL:
        printf("ERROR CRITICO: No hay comunicacion UART (Lectura = 0).\n");
        printf("Revisar conexion TX/RX y alimentacion VM.\n");
        snprintf(buf, sizeof(buf), "ERROR CRITICO: No hay comunicacion UART (Lectura = 0). Revisar conexion TX/RX y alimentacion VM.");
        break;
      case LOG_EVENT_UART_INIT_MICROSTEPS_READ:
        printf("Microsteps Leido: %u\n", msg.payload.raw_data);
        snprintf(buf, sizeof(buf), "Microsteps Leido: %u", msg.payload.raw_data);
        break;
      case LOG_EVENT_PINS_INIT_MODE:
        printf("Iniciando en MODO PINES (Pines MS usados para microstepping)\n");
        snprintf(buf, sizeof(buf), "Iniciando en MODO PINES (Pines MS usados para microstepping)");
        break;
      case LOG_EVENT_GENERAL_DEBUG:
        printf("Debug raw: %u\n", msg.payload.raw_data);
        snprintf(buf, sizeof(buf), "Debug raw: %u", msg.payload.raw_data);
        break;
      case LOG_EVENT_MOTOR_MOVING:
        printf("Motor en movimiento...\n");
        snprintf(buf, sizeof(buf), "Motor en movimiento...");
        break;
      case LOG_EVENT_STRING_MSG:
        printf("%s\n", msg.payload.msg_str);
        snprintf(buf, sizeof(buf), "%s", msg.payload.msg_str);
        break;
      default:
        printf("Unknown crosscore logger event: %d\n", msg.id);
        snprintf(buf, sizeof(buf), "Unknown crosscore logger event: %d", msg.id);
        break;
      }
      
      const char *topic = "syringe_pump/log/system";
      switch (msg.id) {
        case LOG_EVENT_MOTOR_MOVING:
        case LOG_EVENT_MOTOR_STOPPED:
        case LOG_EVENT_MOTOR_RETRACTING:
        case LOG_EVENT_MOTOR_START_HIT:
        case LOG_EVENT_MOTOR_END_HIT:
          topic = "syringe_pump/log/motor_state";
          break;
        case LOG_EVENT_PRESSURE_UPDATE:
        case LOG_EVENT_PRESSURE_ALERT:
        case LOG_EVENT_PRESSURE_SAFE:
          topic = "syringe_pump/log/pressure";
          break;
        case LOG_EVENT_MOTOR_STALL:
        case LOG_EVENT_DRV_STATUS_ERROR:
        case LOG_EVENT_UART_INIT_OK:
        case LOG_EVENT_UART_INIT_FAIL:
        case LOG_EVENT_UART_INIT_MICROSTEPS_READ:
        case LOG_EVENT_PINS_INIT_MODE:
          topic = "syringe_pump/log/motor_regs";
          break;
        default:
          topic = "syringe_pump/log/system";
          break;
      }
      mqtt_client_publish(topic, buf);
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

  // Esperar 10 segundos antes de accionar (para estabilizar red y driver)
  vTaskDelay(pdMS_TO_TICKS(10000));
  safe_printf("Iniciando bomba jeringa a 50 mL/h en Modo Continuo...\n");
  Pump_Mode_Continuous(50.0f);

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
    // Calculadora temporal: Avanza el volumen y contador de tiempo 2000 ms = 2 seg
    Pump_Tick(telemetry_period_ms);
    Pump_GetTelemetryJSON(json_buf, sizeof(json_buf));
    mqtt_client_publish("syringe_pump/telemetry", json_buf);
    vTaskDelay(pdMS_TO_TICKS(telemetry_period_ms)); // Publicar cada 2 segundos
  }
}

/**
 * @brief Función para configurar todas las tareas del Core 0 antes de iniciar
 * FreeRTOS
 */
void core0_main_setup(void) {
  // Inicializar colas y frameworks de logeo
  print_q = xQueueCreate(PRINT_QUEUE_LENGTH, PRINT_MSG_MAX_LEN);
  crosscore_logger_init();
  crosscore_cmd_init();

  // Creación de las tareas de FreeRTOS
  xTaskCreate(task_init, "Init", 1024, NULL, 2, NULL);
  xTaskCreate(task_blinky, "Blinky", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(task_logger, "Logger", configMINIMAL_STACK_SIZE * 3, NULL, 1,
              NULL);
  xTaskCreate(task_pump_telemetry, "Telemetry", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
  xTaskCreate(task_example_internal_cmd, "CmdExample", configMINIMAL_STACK_SIZE, NULL,
              1, NULL);
}
