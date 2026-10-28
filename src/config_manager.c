#include "config_manager.h"
#include "system_config.h"
#include "crosscore_cmd.h"
#include "mqtt_client.h" // For default macros
#include "serial_consumer.h"
#include <string.h>
#include <stdio.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "tmc2209.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Reference to global motor on Core 1 if needed to check if it's moving
extern TMC2209_t *global_motor;

// Ensure flash size is defined, typically 2MB (RP2350 usually has 2MB or 4MB, pico2_w has 4MB, but PICO_FLASH_SIZE_BYTES handles it)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024) 
#endif

// Reservamos el último sector de la flash (1 sector = 4096 bytes)
#define CONFIG_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CONFIG_MAGIC_WORD 0xA1B2C3D8

// Multiplo de FLASH_PAGE_SIZE (256 B): 4 paginas = 1024 bytes (aloja ~850 B de SystemConfig_t)
#define CONFIG_FLASH_BUF_SIZE (4 * FLASH_PAGE_SIZE)

SystemConfig_t g_sys_config;
volatile bool g_calibration_dirty = false;
static SemaphoreHandle_t g_config_mutex = NULL;

static void config_lock(void) {
    if (g_config_mutex == NULL) {
        g_config_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(g_config_mutex, portMAX_DELAY);
}

static void config_unlock(void) {
    if (g_config_mutex != NULL) {
        xSemaphoreGive(g_config_mutex);
    }
}

static uint32_t calculate_checksum(SystemConfig_t *cfg) {
    uint32_t sum = 0;
    uint8_t *p = (uint8_t*)cfg;
    // Calculate sum of everything except 'crc'
    size_t len = sizeof(SystemConfig_t) - sizeof(uint32_t);
    for (size_t i = 0; i < len; i++) {
        sum += p[i];
    }
    return sum;
}

void config_manager_init(void) {
    // Initializing the mutex early
    if (g_config_mutex == NULL) {
        g_config_mutex = xSemaphoreCreateMutex();
    }

    // Read from flash. XIP_BASE is starting address of execute-in-place flash.
    const SystemConfig_t *flash_cfg = (const SystemConfig_t *) (XIP_BASE + CONFIG_FLASH_OFFSET);

    bool valid = false;
    // Check if the magic word is present
    if (flash_cfg->magic == CONFIG_MAGIC_WORD) {
        // If magic matches, check checksum
        if (calculate_checksum((SystemConfig_t*)flash_cfg) == flash_cfg->crc) {
            valid = true;
        } else {
            serial_log("!CFG", "flash=CRC_MISMATCH");
        }
    }

    if (valid) {
        memcpy(&g_sys_config, flash_cfg, sizeof(SystemConfig_t));
        if (g_sys_config.thresh_syringe_contact_v <= 0.0f) g_sys_config.thresh_syringe_contact_v = ADC_THRESH_SYRINGE_REACHED_V_DEFAULT;
        if (g_sys_config.thresh_occlusion_v <= 0.0f)       g_sys_config.thresh_occlusion_v       = ADC_THRESH_OCCLUSION_V_DEFAULT;
        if (g_sys_config.thresh_occ_release_v <= 0.0f)     g_sys_config.thresh_occ_release_v     = ADC_THRESH_OCC_RELEASE_V_DEFAULT;
        if (g_sys_config.auto_antibolus_enabled > 1)       g_sys_config.auto_antibolus_enabled   = ANTIBOLUS_AUTO_ENABLED_DEFAULT;
        if (g_sys_config.wifi_profile_count > MAX_WIFI_PROFILES) g_sys_config.wifi_profile_count = MAX_WIFI_PROFILES;

        // Contar perfiles validos reales en la estructura cargada
        int valid_profiles = 0;
        for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
            if (g_sys_config.wifi_profiles[i].valid) {
                valid_profiles++;
            }
        }

        // Si la Flash tenia configuracion previa pero sin perfiles validos, inicializar con los de fabrica
        if (valid_profiles == 0) {
            serial_log("CFG", "flash=FACTORY_DEFAULTS");
            g_sys_config.wifi_profile_count = 2;

            strncpy(g_sys_config.wifi_profiles[0].ssid, WIFI_SSID_HOME, MAX_SSID_LEN - 1);
            strncpy(g_sys_config.wifi_profiles[0].pass, WIFI_PASSWORD_HOME, MAX_PASS_LEN - 1);
            strncpy(g_sys_config.wifi_profiles[0].mqtt_ip, MQTT_BROKER_IP_HOME, MAX_IP_LEN - 1);
            g_sys_config.wifi_profiles[0].mqtt_port = MQTT_BROKER_PORT;
            g_sys_config.wifi_profiles[0].valid = 1;
            g_sys_config.wifi_profiles[0].last_rssi = -128;

            strncpy(g_sys_config.wifi_profiles[1].ssid, WIFI_SSID_MOBILE, MAX_SSID_LEN - 1);
            strncpy(g_sys_config.wifi_profiles[1].pass, WIFI_PASSWORD_MOBILE, MAX_PASS_LEN - 1);
            strncpy(g_sys_config.wifi_profiles[1].mqtt_ip, MQTT_BROKER_IP_MOBILE, MAX_IP_LEN - 1);
            g_sys_config.wifi_profiles[1].mqtt_port = MQTT_BROKER_PORT;
            g_sys_config.wifi_profiles[1].valid = 1;
            g_sys_config.wifi_profiles[1].last_rssi = -128;

            if (g_sys_config.wifi_ssid[0] == '\0') {
                strncpy(g_sys_config.wifi_ssid, g_sys_config.wifi_profiles[0].ssid, MAX_SSID_LEN - 1);
                strncpy(g_sys_config.wifi_pass, g_sys_config.wifi_profiles[0].pass, MAX_PASS_LEN - 1);
                strncpy(g_sys_config.mqtt_ip, g_sys_config.wifi_profiles[0].mqtt_ip, MAX_IP_LEN - 1);
                g_sys_config.mqtt_port = g_sys_config.wifi_profiles[0].mqtt_port;
            }

            config_manager_save(true);
        } else {
            g_sys_config.wifi_profile_count = valid_profiles;
        }

        serial_log("CFG", "flash=LOADED ssid='%s' broker=%s:%d antibolus=%s prof=%d/%d",
                   g_sys_config.wifi_ssid, g_sys_config.mqtt_ip, g_sys_config.mqtt_port,
                   g_sys_config.auto_antibolus_enabled ? "AUTO" : "MANUAL",
                   g_sys_config.wifi_profile_count, MAX_WIFI_PROFILES);
    } else {
        serial_log("CFG", "flash=DEFAULTS");
        memset(&g_sys_config, 0, sizeof(SystemConfig_t));
        g_sys_config.magic = CONFIG_MAGIC_WORD;
        g_sys_config.wifi_enabled = 1;
        g_sys_config.calibration_valid = 0;
        g_sys_config.calibrated_max_encoder_count = 0;
        g_sys_config.thresh_syringe_contact_v = ADC_THRESH_SYRINGE_REACHED_V_DEFAULT;
        g_sys_config.thresh_occlusion_v       = ADC_THRESH_OCCLUSION_V_DEFAULT;
        g_sys_config.thresh_occ_release_v     = ADC_THRESH_OCC_RELEASE_V_DEFAULT;
        g_sys_config.auto_antibolus_enabled   = ANTIBOLUS_AUTO_ENABLED_DEFAULT;

        char unique_id[17];
        pico_get_unique_board_id_string(unique_id, sizeof(unique_id));
        snprintf(g_sys_config.device_id, sizeof(g_sys_config.device_id), "bj-%.8s", unique_id);

        // Preconfigurar perfiles de fabrica
        g_sys_config.wifi_profile_count = 2;

        // Perfil 0: Red Hogar (Telecentro)
        strncpy(g_sys_config.wifi_profiles[0].ssid, WIFI_SSID_HOME, MAX_SSID_LEN - 1);
        strncpy(g_sys_config.wifi_profiles[0].pass, WIFI_PASSWORD_HOME, MAX_PASS_LEN - 1);
        strncpy(g_sys_config.wifi_profiles[0].mqtt_ip, MQTT_BROKER_IP_HOME, MAX_IP_LEN - 1);
        g_sys_config.wifi_profiles[0].mqtt_port = MQTT_BROKER_PORT;
        g_sys_config.wifi_profiles[0].valid = 1;
        g_sys_config.wifi_profiles[0].last_rssi = -128;

        // Perfil 1: Red Movil Hotspot (Kupa)
        strncpy(g_sys_config.wifi_profiles[1].ssid, WIFI_SSID_MOBILE, MAX_SSID_LEN - 1);
        strncpy(g_sys_config.wifi_profiles[1].pass, WIFI_PASSWORD_MOBILE, MAX_PASS_LEN - 1);
        strncpy(g_sys_config.wifi_profiles[1].mqtt_ip, MQTT_BROKER_IP_MOBILE, MAX_IP_LEN - 1);
        g_sys_config.wifi_profiles[1].mqtt_port = MQTT_BROKER_PORT;
        g_sys_config.wifi_profiles[1].valid = 1;
        g_sys_config.wifi_profiles[1].last_rssi = -128;

        // Asignar red activa inicial (Perfil 0)
        strncpy(g_sys_config.wifi_ssid, g_sys_config.wifi_profiles[0].ssid, MAX_SSID_LEN - 1);
        strncpy(g_sys_config.wifi_pass, g_sys_config.wifi_profiles[0].pass, MAX_PASS_LEN - 1);
        strncpy(g_sys_config.mqtt_ip, g_sys_config.wifi_profiles[0].mqtt_ip, MAX_IP_LEN - 1);
        g_sys_config.mqtt_port = g_sys_config.wifi_profiles[0].mqtt_port;

        // Guardar valores por defecto en Flash
        config_manager_save(true);
    }
}

bool config_manager_save(bool override_motor_check) {
    config_lock();

    // Check motor status first
    if (!override_motor_check && global_motor != NULL) {
        if (tmc2209_is_moving(global_motor)) {
            serial_log("!CFG", "ERR=save_aborted reason=motor_moving");
            config_unlock();
            return false;
        }
    }

    _Static_assert(sizeof(SystemConfig_t) <= CONFIG_FLASH_BUF_SIZE,
                   "SystemConfig_t exceeds CONFIG_FLASH_BUF_SIZE!");

    // Prepare buffer. MUST be a multiple of FLASH_PAGE_SIZE (256 bytes)
    uint8_t flash_buffer[CONFIG_FLASH_BUF_SIZE];
    memset(flash_buffer, 0, CONFIG_FLASH_BUF_SIZE);

    g_sys_config.magic = CONFIG_MAGIC_WORD;
    g_sys_config.crc = calculate_checksum(&g_sys_config);
    memcpy(flash_buffer, &g_sys_config, sizeof(SystemConfig_t));

    serial_log("CFG", "flash_write start bytes=%u", (unsigned)CONFIG_FLASH_BUF_SIZE);
    
    // Pause execution of the other core
    multicore_lockout_start_blocking();
    
    // Disable interrupts locally (Core 0)
    uint32_t ints = save_and_disable_interrupts();
    
    // Perform Flash operations safely
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, flash_buffer, CONFIG_FLASH_BUF_SIZE);

    // Re-enable interrupts
    restore_interrupts(ints);
    
    // Resume Core 1
    multicore_lockout_end_blocking();

    config_unlock();
    serial_log("CFG", "flash_write done");
    return true;
}

bool config_wifi_profile_add(const char *ssid, const char *pass, const char *mqtt_ip, uint16_t mqtt_port) {
    if (!ssid || ssid[0] == '\0') {
        serial_log("!CFG", "ERR=empty_ssid");
        return false;
    }

    config_lock();

    // Verificar si el SSID ya existe para actualizarlo
    int idx = -1;
    for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
        if (g_sys_config.wifi_profiles[i].valid &&
            strcmp(g_sys_config.wifi_profiles[i].ssid, ssid) == 0) {
            idx = i;
            break;
        }
    }

    // Si no existe, buscar el primer slot libre
    if (idx < 0) {
        for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
            if (!g_sys_config.wifi_profiles[i].valid) {
                idx = i;
                break;
            }
        }
    }

    if (idx < 0) {
        config_unlock();
        serial_log("!CFG", "ERR=profile_list_full max=%d", MAX_WIFI_PROFILES);
        return false;
    }

    WifiProfile_t *p = &g_sys_config.wifi_profiles[idx];
    if (!p->valid) {
        memset(p, 0, sizeof(WifiProfile_t));
        p->valid = 1;
        p->last_rssi = -128;
        if (g_sys_config.wifi_profile_count < MAX_WIFI_PROFILES) {
            g_sys_config.wifi_profile_count++;
        }
    }

    strncpy(p->ssid, ssid, MAX_SSID_LEN - 1);
    p->ssid[MAX_SSID_LEN - 1] = '\0';

    if (pass != NULL) {
        strncpy(p->pass, pass, MAX_PASS_LEN - 1);
        p->pass[MAX_PASS_LEN - 1] = '\0';
    } else {
        p->pass[0] = '\0';
    }

    if (mqtt_ip != NULL && mqtt_ip[0] != '\0') {
        strncpy(p->mqtt_ip, mqtt_ip, MAX_IP_LEN - 1);
        p->mqtt_ip[MAX_IP_LEN - 1] = '\0';
    } else if (p->mqtt_ip[0] == '\0') {
        strncpy(p->mqtt_ip, MQTT_BROKER_IP, MAX_IP_LEN - 1);
        p->mqtt_ip[MAX_IP_LEN - 1] = '\0';
    }

    if (mqtt_port > 0) {
        p->mqtt_port = mqtt_port;
    } else if (p->mqtt_port == 0) {
        p->mqtt_port = MQTT_BROKER_PORT;
    }

    config_unlock();

    serial_log("CFG", "prof_saved [%d/%d] ssid='%s' broker=%s:%d",
           idx + 1, g_sys_config.wifi_profile_count, p->ssid, p->mqtt_ip, p->mqtt_port);

    config_manager_save(false);
    return true;
}

bool config_wifi_profile_del(const char *ssid) {
    if (!ssid || ssid[0] == '\0') return false;

    config_lock();
    int idx = -1;
    for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
        if (g_sys_config.wifi_profiles[i].valid &&
            strcmp(g_sys_config.wifi_profiles[i].ssid, ssid) == 0) {
            idx = i;
            break;
        }
    }

    if (idx < 0) {
        config_unlock();
        serial_log("!CFG", "ERR=profile_not_found ssid='%s'", ssid);
        return false;
    }

    // Compactar perfiles restantes hacia la izquierda
    for (int i = idx; i < MAX_WIFI_PROFILES - 1; i++) {
        g_sys_config.wifi_profiles[i] = g_sys_config.wifi_profiles[i + 1];
    }
    memset(&g_sys_config.wifi_profiles[MAX_WIFI_PROFILES - 1], 0, sizeof(WifiProfile_t));
    if (g_sys_config.wifi_profile_count > 0) {
        g_sys_config.wifi_profile_count--;
    }

    config_unlock();
    serial_log("CFG", "prof_deleted ssid='%s' rem=%d", ssid, g_sys_config.wifi_profile_count);
    config_manager_save(false);
    return true;
}

int config_wifi_profile_get_all(WifiProfile_t *out, int max_entries) {
    if (!out || max_entries <= 0) return 0;
    config_lock();
    int count = 0;
    for (int i = 0; i < MAX_WIFI_PROFILES && count < max_entries; i++) {
        if (g_sys_config.wifi_profiles[i].valid) {
            out[count++] = g_sys_config.wifi_profiles[i];
        }
    }
    config_unlock();
    return count;
}

const WifiProfile_t* config_wifi_profile_find(const char *ssid) {
    if (!ssid || ssid[0] == '\0') return NULL;
    for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
        if (g_sys_config.wifi_profiles[i].valid &&
            strcmp(g_sys_config.wifi_profiles[i].ssid, ssid) == 0) {
            return &g_sys_config.wifi_profiles[i];
        }
    }
    return NULL;
}

void config_wifi_profile_set_active(const char *ssid, const char *pass, const char *mqtt_ip, uint16_t mqtt_port) {
    config_lock();
    if (ssid != NULL && ssid[0] != '\0') {
        strncpy(g_sys_config.wifi_ssid, ssid, MAX_SSID_LEN - 1);
        g_sys_config.wifi_ssid[MAX_SSID_LEN - 1] = '\0';
    }
    if (pass != NULL) {
        strncpy(g_sys_config.wifi_pass, pass, MAX_PASS_LEN - 1);
        g_sys_config.wifi_pass[MAX_PASS_LEN - 1] = '\0';
    } else {
        g_sys_config.wifi_pass[0] = '\0';
    }
    if (mqtt_ip != NULL && mqtt_ip[0] != '\0') {
        strncpy(g_sys_config.mqtt_ip, mqtt_ip, MAX_IP_LEN - 1);
        g_sys_config.mqtt_ip[MAX_IP_LEN - 1] = '\0';
    }
    if (mqtt_port > 0) {
        g_sys_config.mqtt_port = mqtt_port;
    }
    config_unlock();
}

void config_set_wifi(const char* ssid, const char* pass) {
    config_lock();
    strncpy(g_sys_config.wifi_ssid, ssid, MAX_SSID_LEN);
    g_sys_config.wifi_ssid[MAX_SSID_LEN - 1] = '\0';
    
    strncpy(g_sys_config.wifi_pass, pass, MAX_PASS_LEN);
    g_sys_config.wifi_pass[MAX_PASS_LEN - 1] = '\0';
    config_unlock();

    serial_log("CFG", "wifi_ram_upd");
}

void config_set_wifi_ssid(const char* ssid) {
    config_lock();
    strncpy(g_sys_config.wifi_ssid, ssid, MAX_SSID_LEN);
    g_sys_config.wifi_ssid[MAX_SSID_LEN - 1] = '\0';
    config_unlock();
    serial_log("CFG", "wifi_ssid='%s'", ssid);
}

void config_set_wifi_pass(const char* pass) {
    config_lock();
    strncpy(g_sys_config.wifi_pass, pass, MAX_PASS_LEN);
    g_sys_config.wifi_pass[MAX_PASS_LEN - 1] = '\0';
    config_unlock();
    serial_log("CFG", "wifi_pass=UPDATED");
}

void config_set_mqtt(const char* ip, uint16_t port) {
    config_lock();
    strncpy(g_sys_config.mqtt_ip, ip, MAX_IP_LEN);
    g_sys_config.mqtt_ip[MAX_IP_LEN - 1] = '\0';
    g_sys_config.mqtt_port = port;
    config_unlock();
    
    serial_log("CFG", "mqtt_ram_upd broker=%s:%d", g_sys_config.mqtt_ip, g_sys_config.mqtt_port);
}

void config_set_mqtt_ip(const char* ip) {
    config_lock();
    strncpy(g_sys_config.mqtt_ip, ip, MAX_IP_LEN);
    g_sys_config.mqtt_ip[MAX_IP_LEN - 1] = '\0';
    config_unlock();
    serial_log("CFG", "mqtt_ip=%s", ip);
}

void config_set_mqtt_port(uint16_t port) {
    config_lock();
    g_sys_config.mqtt_port = port;
    config_unlock();
    serial_log("CFG", "mqtt_port=%d", port);
}

void config_set_wifi_enabled(bool enabled) {
    config_lock();
    g_sys_config.wifi_enabled = enabled ? 1 : 0;
    config_unlock();
    serial_log("CFG", "wifi_ena=%d", enabled ? 1 : 0);
}

void config_set_adc_thresholds(float v_syringe, float v_occ, float v_release) {
    config_lock();
    if (v_syringe > 0.0f) g_sys_config.thresh_syringe_contact_v = v_syringe;
    if (v_occ > 0.0f)     g_sys_config.thresh_occlusion_v = v_occ;
    if (v_release > 0.0f) g_sys_config.thresh_occ_release_v = v_release;
    config_unlock();

    cmd_send_set_adc_thresholds(g_sys_config.thresh_syringe_contact_v,
                                g_sys_config.thresh_occlusion_v,
                                g_sys_config.thresh_occ_release_v);
    serial_log("CFG", "adc_thr cont=%.2fV occ=%.2fV rel=%.2fV",
           g_sys_config.thresh_syringe_contact_v,
           g_sys_config.thresh_occlusion_v,
           g_sys_config.thresh_occ_release_v);
}

void config_set_antibolus_mode(bool auto_enabled) {
    config_lock();
    g_sys_config.auto_antibolus_enabled = auto_enabled ? 1 : 0;
    config_unlock();

    cmd_send_set_antibolus_mode(auto_enabled);
    serial_log("CFG", "antibolus=%s", auto_enabled ? "AUTO" : "MANUAL");
}
