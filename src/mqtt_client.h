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
#define WIFI_SSID "Telecentro-c220"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "QCW6G53HNLYG"
#endif

#ifndef MQTT_BROKER_IP
#define MQTT_BROKER_IP "192.168.0.22" // MQTT broker IP
#endif

#define MQTT_BROKER_PORT 1883

void mqtt_client_task(void *params);
bool mqtt_client_publish(const char *topic, const char *payload);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_H
