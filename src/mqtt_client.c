#include "mqtt_client.h"
#include "lwip/apps/mqtt.h"
#include "crosscore_cmd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config_manager.h"

#include "pico/cyw43_arch.h"

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
    
    // Parse and execute the command
    cmd_parse_and_execute(payload_str);
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
    ipaddr_aton(g_sys_config.mqtt_ip, &broker_ip);

    cyw43_arch_lwip_begin();
    mqtt_client = mqtt_client_new();
    cyw43_arch_lwip_end();

    if (mqtt_client == NULL) {
        printf("Failed to create MQTT client\n");
        vTaskDelete(NULL);
        return;
    }

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico2w_syringe_pump";
    ci.keep_alive = 60;

    // Retry loop
    while (1) {
        cyw43_arch_lwip_begin();
        bool is_connected = mqtt_client_is_connected(mqtt_client);
        cyw43_arch_lwip_end();

        if (!is_connected) {
            // Update broker IP in case it was changed dynamically
            ipaddr_aton(g_sys_config.mqtt_ip, &broker_ip);

            mqtt_connected = false;
            printf("Attempting MQTT connection to %s:%d...\n", g_sys_config.mqtt_ip, g_sys_config.mqtt_port);
            
            cyw43_arch_lwip_begin();
            err_t err = mqtt_client_connect(mqtt_client, &broker_ip, g_sys_config.mqtt_port, mqtt_connection_cb, NULL, &ci);
            cyw43_arch_lwip_end();

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
    
    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(mqtt_client, topic, payload, strlen(payload), 0, 0, mqtt_request_cb, NULL);
    cyw43_arch_lwip_end();
    
    return (err == ERR_OK);
}

void mqtt_client_force_reconnect(void) {
    if (mqtt_client != NULL && mqtt_client_is_connected(mqtt_client)) {
        printf("Forcing MQTT disconnect...\n");
        cyw43_arch_lwip_begin();
        mqtt_disconnect(mqtt_client);
        cyw43_arch_lwip_end();
    }
}
