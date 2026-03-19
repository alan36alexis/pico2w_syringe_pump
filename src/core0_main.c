#include "core0_main.h"
#include "FreeRTOS.h"
#include "mqtt_client.h"
#include "crosscore_cmd.h"
#include "crosscore_logger.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "task.h"
#include <stdio.h>

/**
 * @brief Tarea de inicializacion
 */
static void task_init(void *params) {
  // Inicializacion de GPIO y Wi-Fi chip (CYW43)
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_WORLDWIDE)) {
    printf("Wi-Fi init failed\n");
    vTaskDelete(NULL);
    return;
  }
  
  cyw43_arch_enable_sta_mode();
  printf("Connecting to Wi-Fi (%s)...\n", WIFI_SSID);
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
      printf("Failed to connect to Wi-Fi.\n");
      vTaskDelete(NULL);
      return;
  }
  printf("Connected to Wi-Fi.\n");
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

  // Iniciar la tarea cliente MQTT
  xTaskCreate(mqtt_client_task, "MQTT_Task", configMINIMAL_STACK_SIZE * 4, NULL, 2, NULL);

  // Elimino la tarea para liberar recursos tras una única ejecución
  vTaskDelete(NULL);
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
  while (1) {
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
      case LOG_EVENT_STRING_MSG:
        printf("%s\n", msg.payload.msg_str);
        snprintf(buf, sizeof(buf), "%s", msg.payload.msg_str);
        break;
      default:
        printf("Unknown crosscore logger event: %d\n", msg.id);
        snprintf(buf, sizeof(buf), "Unknown crosscore logger event: %d", msg.id);
        break;
      }
      
      mqtt_client_publish("syringe_pump/log", buf);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Poll cada 10ms
  }
}

/**
 * @brief Tarea de ejemplo interno (mantenida para uso interno/testing)
 */
static void task_example_internal_cmd(void *params) {
  while (1) {
    // Incrementar el delay a 60s para que no dispare aleatoriamente
    vTaskDelay(pdMS_TO_TICKS(60000));
    // cmd_send_move_linear_um(10000.0f, 450.0f);
    // vTaskDelay(pdMS_TO_TICKS(10000));
    // cmd_send_stop_motor();
  }
}

/**
 * @brief Función para configurar todas las tareas del Core 0 antes de iniciar
 * FreeRTOS
 */
void core0_main_setup(void) {
  // Inicializar la cola crosscore
  crosscore_logger_init();
  crosscore_cmd_init();

  // Creación de las tareas de FreeRTOS
  xTaskCreate(task_init, "Init", 1024, NULL, 2, NULL);
  xTaskCreate(task_blinky, "Blinky", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(task_logger, "Logger", configMINIMAL_STACK_SIZE * 3, NULL, 1,
              NULL);
  xTaskCreate(task_example_internal_cmd, "CmdExample", configMINIMAL_STACK_SIZE, NULL,
              1, NULL);
}
