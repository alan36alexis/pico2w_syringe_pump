#include "mqtt_topics.h"
#include <stdio.h>

// Max topic length: "bj/" + 16 chars id + "/cmd/ack" + null = 28 chars; 48 gives ample margin.
#define TOPIC_BUF_LEN 48

static char s_status[TOPIC_BUF_LEN];
static char s_telemetry[TOPIC_BUF_LEN];
static char s_event[TOPIC_BUF_LEN];
static char s_cmd[TOPIC_BUF_LEN];
static char s_cmd_ack[TOPIC_BUF_LEN];

void topics_init(const char *device_id) {
    snprintf(s_status,    sizeof(s_status),    "bj/%s/status",   device_id);
    snprintf(s_telemetry, sizeof(s_telemetry), "bj/%s/telemetry",device_id);
    snprintf(s_event,     sizeof(s_event),     "bj/%s/event",    device_id);
    snprintf(s_cmd,       sizeof(s_cmd),       "bj/%s/cmd",      device_id);
    snprintf(s_cmd_ack,   sizeof(s_cmd_ack),   "bj/%s/cmd/ack",  device_id);
}

const char* topic_status(void)    { return s_status; }
const char* topic_telemetry(void) { return s_telemetry; }
const char* topic_event(void)     { return s_event; }
const char* topic_cmd(void)       { return s_cmd; }
const char* topic_cmd_ack(void)   { return s_cmd_ack; }
