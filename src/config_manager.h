#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SSID_LEN 32
#define MAX_PASS_LEN 64
#define MAX_IP_LEN   16

typedef struct {
    uint32_t magic;
    char wifi_ssid[MAX_SSID_LEN];
    char wifi_pass[MAX_PASS_LEN];
    char mqtt_ip[MAX_IP_LEN];
    uint16_t mqtt_port;
    uint8_t _padding[2];
    uint32_t crc;
} SystemConfig_t;

extern SystemConfig_t g_sys_config;

// Initializes the configuration manager: loads from flash if valid, 
// otherwise populates with defaults.
void config_manager_init(void);

// Saves the current g_sys_config to flash.
// Returns false if motor is running or something failed.
bool config_manager_save(bool override_motor_check);

// Dynamic setters
void config_set_wifi(const char* ssid, const char* pass);
void config_set_wifi_ssid(const char* ssid);
void config_set_wifi_pass(const char* pass);
void config_set_mqtt(const char* ip, uint16_t port);
void config_set_mqtt_ip(const char* ip);
void config_set_mqtt_port(uint16_t port);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H
