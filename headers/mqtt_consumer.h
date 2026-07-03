#pragma once

// Creates and starts the MQTT consumer task (priority 2).
// Maintains MqttSnapshot_t and will publish multi-rate telemetry (Paso 6).
// Must be called after system_queues_init().
void mqtt_consumer_start(void);
