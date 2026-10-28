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
#define MAX_WIFI_PROFILES 6

typedef struct {
    char ssid[MAX_SSID_LEN];
    char pass[MAX_PASS_LEN];
    char mqtt_ip[MAX_IP_LEN];
    uint16_t mqtt_port;
    uint8_t valid;
    int8_t last_rssi;
} WifiProfile_t;

typedef struct {
    uint32_t magic;
    char wifi_ssid[MAX_SSID_LEN];       /* Red Wi-Fi activa / conectada */
    char wifi_pass[MAX_PASS_LEN];       /* Contraseña activa */
    char mqtt_ip[MAX_IP_LEN];           /* Broker MQTT activo */
    uint16_t mqtt_port;                 /* Puerto broker MQTT activo */
    uint8_t wifi_enabled;               /* 1 = Habilitado, 0 = Ahorro de batería */
    uint8_t calibration_valid;
    int32_t calibrated_max_encoder_count;
    char device_id[17];
    float thresh_syringe_contact_v;
    float thresh_occlusion_v;
    float thresh_occ_release_v;
    uint8_t auto_antibolus_enabled;     /* 1 = Automático (defecto), 0 = Manual */
    uint8_t wifi_profile_count;         /* Cantidad de perfiles válidos (0..MAX_WIFI_PROFILES) */
    uint8_t _reserved[2];               /* Alineación a 4 bytes */
    WifiProfile_t wifi_profiles[MAX_WIFI_PROFILES];
    uint32_t crc;
} SystemConfig_t;

extern SystemConfig_t g_sys_config;

// Set by Core 1 after a successful calibration; cleared by Core 0 after saving.
extern volatile bool g_calibration_dirty;

// Initializes the configuration manager: loads from flash if valid, 
// otherwise populates with defaults.
void config_manager_init(void);

// Saves the current g_sys_config to flash.
// Returns false if motor is running or something failed.
bool config_manager_save(bool override_motor_check);

// Wi-Fi Profiles Management API
bool config_wifi_profile_add(const char *ssid, const char *pass, const char *mqtt_ip, uint16_t mqtt_port);
bool config_wifi_profile_del(const char *ssid);
int  config_wifi_profile_get_all(WifiProfile_t *out, int max_entries);
const WifiProfile_t* config_wifi_profile_find(const char *ssid);
void config_wifi_profile_set_active(const char *ssid, const char *pass, const char *mqtt_ip, uint16_t mqtt_port);

// Dynamic setters
void config_set_wifi(const char* ssid, const char* pass);
void config_set_wifi_ssid(const char* ssid);
void config_set_wifi_pass(const char* pass);
void config_set_mqtt(const char* ip, uint16_t port);
void config_set_mqtt_ip(const char* ip);
void config_set_mqtt_port(uint16_t port);
void config_set_wifi_enabled(bool enabled);
void config_set_adc_thresholds(float v_syringe, float v_occ, float v_release);
void config_set_antibolus_mode(bool auto_enabled);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H
