#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Placeholder for user credentials
#ifndef WIFI_SSID
#define WIFI_SSID "Telecentro-c220" //"UTN_2022"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "QCW6G53HNLYG" //"utn.2022"
#endif

#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP "192.168.0.21" // MQTT broker IP
#endif

#define MQTT_BROKER_PORT  1883
#define MQTT_MAX_PAYLOAD  256  // max bytes de cualquier payload en la cola MQTT

void mqtt_client_queue_init(void);
void mqtt_client_task(void *params);
void mqtt_rx_task(void *params);
bool mqtt_client_publish(const char *topic, const char *payload);
bool mqtt_client_publish_qos1(const char *topic, const char *payload);
void mqtt_client_force_reconnect(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_H
