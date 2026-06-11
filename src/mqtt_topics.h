#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

#ifdef __cplusplus
extern "C" {
#endif

// Must be called once with the device_id from g_sys_config before any MQTT connection.
void        topics_init(const char *device_id);

const char* topic_status(void);     // bj/{id}/status      — QoS 1, retain
const char* topic_telemetry(void);  // bj/{id}/telemetry   — QoS 0
const char* topic_event(void);      // bj/{id}/event       — QoS 1
const char* topic_cmd(void);        // bj/{id}/cmd         — QoS 1
const char* topic_cmd_ack(void);    // bj/{id}/cmd/ack     — QoS 1

#ifdef __cplusplus
}
#endif

#endif // MQTT_TOPICS_H
