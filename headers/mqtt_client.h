#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Credenciales de redes conocidas (Casa y Móvil)
#define WIFI_SSID_HOME          "Telecentro-c220"
#define WIFI_PASSWORD_HOME      "QCW6G53HNLYG"
#define MQTT_BROKER_IP_HOME     "192.168.0.15"

#define WIFI_SSID_MOBILE        "Kupa"
#define WIFI_PASSWORD_MOBILE    ""
#define MQTT_BROKER_IP_MOBILE   "10.75.224.223"

#ifndef WIFI_SSID
#define WIFI_SSID WIFI_SSID_HOME
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD WIFI_PASSWORD_HOME
#endif

#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP MQTT_BROKER_IP_HOME
#endif

#define MQTT_BROKER_PORT  1883
#define MQTT_MAX_PAYLOAD  384  // max bytes de cualquier payload en la cola MQTT (aloja realtime JSON ~300 B)
#define MQTT_TASK_DELAY_MS 5000

void mqtt_client_queue_init(void);
void mqtt_client_task(void *params);
void mqtt_rx_task(void *params);
bool mqtt_client_publish(const char *topic, const char *payload);
bool mqtt_client_publish_qos1(const char *topic, const char *payload);
void     mqtt_client_force_reconnect(void);
uint32_t mqtt_get_tx_drops(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_H
