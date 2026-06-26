#include "mqtt_client.h"
#include "mqtt_topics.h"
#include "lwip/apps/mqtt.h"
#include "cmd_dispatcher.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "config_manager.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "system_queues.h"

static volatile uint32_t s_mqtt_tx_drops = 0;

// Define struct for queue messages
typedef struct {
    char topic[64];
    char payload[MQTT_MAX_PAYLOAD];
    uint8_t qos;
} mqtt_msg_t;

static QueueHandle_t mqtt_tx_queue = NULL;
static QueueHandle_t mqtt_rx_queue = NULL;

static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connected = false;

void mqtt_client_queue_init(void) {
    if (mqtt_tx_queue == NULL) {
        mqtt_tx_queue = xQueueCreate(10, sizeof(mqtt_msg_t));
    }
    if (mqtt_rx_queue == NULL) {
        mqtt_rx_queue = xQueueCreate(10, sizeof(mqtt_msg_t));
    }
}

static char s_rx_topic[64];

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg;

    if (mqtt_rx_queue == NULL) return;
    if (!(flags & MQTT_DATA_FLAG_LAST)) return; // ignore fragmented payloads

    mqtt_msg_t rx_msg;
    strncpy(rx_msg.topic, s_rx_topic, sizeof(rx_msg.topic) - 1);
    rx_msg.topic[sizeof(rx_msg.topic) - 1] = '\0';

    u16_t copy_len = len < (sizeof(rx_msg.payload) - 1) ? len : (sizeof(rx_msg.payload) - 1);
    memcpy(rx_msg.payload, data, copy_len);
    rx_msg.payload[copy_len] = '\0';

    xQueueSendToBack(mqtt_rx_queue, &rx_msg, 0);
}

void mqtt_rx_task(void *params) {
    (void)params;
    mqtt_msg_t msg;

    while (1) {
        if (xQueueReceive(mqtt_rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            int  cid = -1;
            char cmd_buf[128];  // comandos son strings cortos; no usar sizeof(msg.payload)

            // Intentar parsear envelope {"cid":N,"cmd":"..."}
            if (sscanf(msg.payload,
                       "{\"cid\":%d,\"cmd\":\"%127[^\"]\"}", &cid, cmd_buf) == 2) {
                // envelope válido — cmd_buf contiene el comando
            } else {
                // string crudo — backward compat con CLI y versiones previas
                cid = -1;
                strncpy(cmd_buf, msg.payload, sizeof(cmd_buf) - 1);
                cmd_buf[sizeof(cmd_buf) - 1] = '\0';
            }

            CmdDispatchResult_t r = cmd_dispatch_string(cmd_buf, CMD_SRC_MQTT, cid);

            if (cid >= 0) {
                char ack[64];
                snprintf(ack, sizeof(ack),
                         "{\"cid\":%d,\"result\":\"%s\"}",
                         cid, r.accepted ? "accepted" : "rejected");
                mqtt_client_publish_qos1(topic_cmd_ack(), ack);
            }
        }
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg;
    (void)tot_len;
    strncpy(s_rx_topic, topic, sizeof(s_rx_topic) - 1);
    s_rx_topic[sizeof(s_rx_topic) - 1] = '\0';
}

static void mqtt_request_cb(void *arg, err_t err) {
    (void)arg;
    if (err != ERR_OK)
        s_mqtt_tx_drops++;
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        mqtt_connected = true;
        CORE0_EMIT(EV_NET_MQTT_CONN, param, (uint32_t)0);

        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

        err_t err = mqtt_subscribe(client, topic_cmd(), 1, mqtt_request_cb, NULL);
        if (err != ERR_OK) {
            printf("[MQT]: Subscribe error: %d\n", err);
        }

        // Publish online status with retain so the dashboard always sees it
        char online[80];
        snprintf(online, sizeof(online),
            "{\"state\":\"online\",\"id\":\"%s\",\"fw\":\"v1.0.0\"}", g_sys_config.device_id);
        mqtt_publish(client, topic_status(), online, strlen(online), 1, 1, mqtt_request_cb, NULL);
    } else {
        mqtt_connected = false;
        CORE0_EMIT(EV_NET_MQTT_DISC, param, (uint32_t)status);
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
        printf("[MQT]: Failed to create MQTT client\n");
        vTaskDelete(NULL);
        return;
    }

    topics_init(g_sys_config.device_id);

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id  = g_sys_config.device_id;
    ci.keep_alive = 60;
    ci.will_topic  = topic_status();
    ci.will_msg    = "{\"state\":\"offline\"}";
    ci.will_qos    = 1;
    ci.will_retain = 1;

    // Retry loop
    mqtt_msg_t msg;
    while (1) {
        cyw43_arch_lwip_begin();
        bool is_connected = mqtt_client_is_connected(mqtt_client);
        cyw43_arch_lwip_end();

        if (!is_connected) {
            // Update broker IP in case it was changed dynamically
            ipaddr_aton(g_sys_config.mqtt_ip, &broker_ip);

            mqtt_connected = false;
            
            int link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            if (link_status == CYW43_LINK_UP) {
                cyw43_arch_lwip_begin();
                err_t err = mqtt_client_connect(mqtt_client, &broker_ip, g_sys_config.mqtt_port, mqtt_connection_cb, NULL, &ci);
                cyw43_arch_lwip_end();

                if (err != ERR_OK) {
                    printf("[MQT]: Connection error: %d\n", err);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(MQTT_TASK_DELAY_MS));
        } else {
            // Guardián del Transmisor: espera 5s en la cola
            if (mqtt_tx_queue != NULL) {
                if (xQueueReceive(mqtt_tx_queue, &msg, pdMS_TO_TICKS(MQTT_TASK_DELAY_MS)) == pdTRUE) {
                    if (mqtt_connected) {
                        cyw43_arch_lwip_begin();
                        mqtt_publish(mqtt_client, msg.topic, msg.payload, strlen(msg.payload), msg.qos, 0, mqtt_request_cb, NULL);
                        cyw43_arch_lwip_end();
                    }
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(MQTT_TASK_DELAY_MS));
            }
        }
    }
}

bool mqtt_client_publish(const char *topic, const char *payload) {
    if (mqtt_tx_queue == NULL) {
        return false;
    }
    // Si mqtt no está conectado, el paquete se encola igual y se procesará si/cuando vuelva
    
    mqtt_msg_t msg;
    strncpy(msg.topic, topic, sizeof(msg.topic) - 1);
    msg.topic[sizeof(msg.topic) - 1] = '\0';
    strncpy(msg.payload, payload, sizeof(msg.payload) - 1);
    msg.payload[sizeof(msg.payload) - 1] = '\0';
    msg.qos = 0;

    if (xQueueSendToBack(mqtt_tx_queue, &msg, 0) != pdTRUE) {
        s_mqtt_tx_drops++;
        return false;
    }
    return true;
}

bool mqtt_client_publish_qos1(const char *topic, const char *payload) {
    if (mqtt_tx_queue == NULL) return false;
    mqtt_msg_t msg;
    strncpy(msg.topic,   topic,   sizeof(msg.topic)   - 1); msg.topic[sizeof(msg.topic)     - 1] = '\0';
    strncpy(msg.payload, payload, sizeof(msg.payload) - 1); msg.payload[sizeof(msg.payload) - 1] = '\0';
    msg.qos = 1;
    if (xQueueSendToBack(mqtt_tx_queue, &msg, 0) != pdTRUE) {
        s_mqtt_tx_drops++;
        return false;
    }
    return true;
}

uint32_t mqtt_get_tx_drops(void) {
    return s_mqtt_tx_drops;
}

void mqtt_client_force_reconnect(void) {
    if (mqtt_client != NULL && mqtt_client_is_connected(mqtt_client)) {
        printf("[MQT]: Forcing disconnect...\n");
        cyw43_arch_lwip_begin();
        mqtt_disconnect(mqtt_client);
        cyw43_arch_lwip_end();
    }
}
