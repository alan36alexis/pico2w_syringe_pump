#include "mqtt_client.h"
#include "lwip/apps/mqtt.h"
#include "crosscore_cmd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connected = false;

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg;
    (void)flags;
    // We expect a string like "10000.0,450.0"
    char payload_str[64];
    u16_t copy_len = len < (sizeof(payload_str) - 1) ? len : (sizeof(payload_str) - 1);
    memcpy(payload_str, data, copy_len);
    payload_str[copy_len] = '\0';
    
    printf("MQTT Payload received: %s\n", payload_str);
    
    // Parse the payload depending on the topic length/context.
    // For now, assume it's cmd_send_move_linear_um
    float speed = 0.0f, pos = 0.0f;
    char *comma = strchr(payload_str, ',');
    if (comma != NULL) {
        *comma = '\0'; // Dividir el string en 2
        speed = (float)atof(payload_str);
        pos = (float)atof(comma + 1);
        printf("Executing command via MQTT: speed=%.2f, pos=%.2f\n", speed, pos);
        cmd_send_move_linear_um(speed, pos);
    } else {
        printf("Failed to parse command payload\n");
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg;
    printf("MQTT Incoming publish on topic: %s, total length: %u\n", topic, tot_len);
}

static void mqtt_request_cb(void *arg, err_t err) {
    (void)arg;
    (void)err;
    // Callback para operaciones MQTT como publish o subscribe
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT Connected!\n");
        mqtt_connected = true;
        
        // Setup incoming callbacks
        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
        
        // Subscribe to command topic
        err_t err = mqtt_subscribe(client, "syringe_pump/cmd", 0, mqtt_request_cb, NULL);
        if (err != ERR_OK) {
            printf("Failed to subscribe (err %d)\n", err);
        }
    } else {
        printf("MQTT Connection disconnected, status: %d\n", status);
        mqtt_connected = false;
    }
}

void mqtt_client_task(void *params) {
    (void)params;
    ip_addr_t broker_ip;
    ipaddr_aton(MQTT_BROKER_IP, &broker_ip);

    mqtt_client = mqtt_client_new();
    if (mqtt_client == NULL) {
        printf("Failed to create MQTT client\n");
        vTaskDelete(NULL);
        return;
    }

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico2w_syringe_pump";
    ci.keep_alive = 60;

    while (1) {
        if (!mqtt_client_is_connected(mqtt_client)) {
            mqtt_connected = false;
            printf("Attempting MQTT connection...\n");
            err_t err = mqtt_client_connect(mqtt_client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_cb, NULL, &ci);
            if (err != ERR_OK) {
                printf("MQTT connection error: %d\n", err);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

bool mqtt_client_publish(const char *topic, const char *payload) {
    if (!mqtt_connected || mqtt_client == NULL) {
        return false;
    }
    
    err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload), 0, 0, mqtt_request_cb, NULL);
    return (err == ERR_OK);
}
