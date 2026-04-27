#include "config_manager.h"
#include "mqtt_client.h" // For default macros
#include <string.h>
#include <stdio.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
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
#define CONFIG_MAGIC_WORD 0xA1B2C3D4

SystemConfig_t g_sys_config;
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
            printf("CONFIG: Magic OK, but CRC mismatch!\n");
        }
    }

    if (valid) {
        memcpy(&g_sys_config, flash_cfg, sizeof(SystemConfig_t));
        printf("CONFIG: Loaded from Flash successfully.\n");
        printf("CONFIG: SSID='%s', MQTT='%s:%d'\n", g_sys_config.wifi_ssid, g_sys_config.mqtt_ip, g_sys_config.mqtt_port);
    } else {
        printf("CONFIG: No valid config in Flash. Using firmware defaults.\n");
        g_sys_config.magic = CONFIG_MAGIC_WORD;
        
        strncpy(g_sys_config.wifi_ssid, WIFI_SSID, MAX_SSID_LEN);
        g_sys_config.wifi_ssid[MAX_SSID_LEN - 1] = '\0';
        
        strncpy(g_sys_config.wifi_pass, WIFI_PASSWORD, MAX_PASS_LEN);
        g_sys_config.wifi_pass[MAX_PASS_LEN - 1] = '\0';
        
        strncpy(g_sys_config.mqtt_ip, MQTT_BROKER_IP, MAX_IP_LEN);
        g_sys_config.mqtt_ip[MAX_IP_LEN - 1] = '\0';
        
        g_sys_config.mqtt_port = MQTT_BROKER_PORT;
        g_sys_config.wifi_enabled = 1;
    }
}

bool config_manager_save(bool override_motor_check) {
    config_lock();

    // Check motor status first
    if (!override_motor_check && global_motor != NULL) {
        if (tmc2209_is_moving(global_motor)) {
            printf("CONFIG: Guardado abortado! El motor esta en movimiento.\n");
            config_unlock();
            return false;
        }
    }

    // Prepare buffer. MUST be a multiple of FLASH_PAGE_SIZE (256 bytes)
    uint8_t flash_buffer[FLASH_PAGE_SIZE];
    memset(flash_buffer, 0, FLASH_PAGE_SIZE);

    g_sys_config.magic = CONFIG_MAGIC_WORD;
    g_sys_config.crc = calculate_checksum(&g_sys_config);
    // Note: sizeof(SystemConfig_t) must be <= FLASH_PAGE_SIZE
    memcpy(flash_buffer, &g_sys_config, sizeof(SystemConfig_t));

    printf("CONFIG: Suspendiendo Core 1 y deshabilitando interrupciones para borrar/escribir sector Flash...\n");
    
    // Pause execution of the other core
    multicore_lockout_start_blocking();
    
    // Disable interrupts locally (Core 0)
    uint32_t ints = save_and_disable_interrupts();
    
    // Perform Flash operations safely
    flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CONFIG_FLASH_OFFSET, flash_buffer, FLASH_PAGE_SIZE);

    // Re-enable interrupts
    restore_interrupts(ints);
    
    // Resume Core 1
    multicore_lockout_end_blocking();

    config_unlock();
    printf("CONFIG: Flash write complete. Sistema reanudado.\n");
    return true;
}

void config_set_wifi(const char* ssid, const char* pass) {
    config_lock();
    strncpy(g_sys_config.wifi_ssid, ssid, MAX_SSID_LEN);
    g_sys_config.wifi_ssid[MAX_SSID_LEN - 1] = '\0';
    
    strncpy(g_sys_config.wifi_pass, pass, MAX_PASS_LEN);
    g_sys_config.wifi_pass[MAX_PASS_LEN - 1] = '\0';
    config_unlock();

    printf("CONFIG: WiFi en RAM actualizado (SSID y Pass)\n");
}

void config_set_wifi_ssid(const char* ssid) {
    config_lock();
    strncpy(g_sys_config.wifi_ssid, ssid, MAX_SSID_LEN);
    g_sys_config.wifi_ssid[MAX_SSID_LEN - 1] = '\0';
    config_unlock();
    printf("CONFIG: WiFi SSID actualizado a '%s'\n", ssid);
}

void config_set_wifi_pass(const char* pass) {
    config_lock();
    strncpy(g_sys_config.wifi_pass, pass, MAX_PASS_LEN);
    g_sys_config.wifi_pass[MAX_PASS_LEN - 1] = '\0';
    config_unlock();
    printf("CONFIG: WiFi Password actualizado.\n");
}

void config_set_mqtt(const char* ip, uint16_t port) {
    config_lock();
    strncpy(g_sys_config.mqtt_ip, ip, MAX_IP_LEN);
    g_sys_config.mqtt_ip[MAX_IP_LEN - 1] = '\0';
    g_sys_config.mqtt_port = port;
    config_unlock();
    
    printf("CONFIG: MQTT en RAM actualizado a '%s:%d'\n", g_sys_config.mqtt_ip, g_sys_config.mqtt_port);
}

void config_set_mqtt_ip(const char* ip) {
    config_lock();
    strncpy(g_sys_config.mqtt_ip, ip, MAX_IP_LEN);
    g_sys_config.mqtt_ip[MAX_IP_LEN - 1] = '\0';
    config_unlock();
    printf("CONFIG: MQTT IP actualizado a '%s'\n", ip);
}

void config_set_mqtt_port(uint16_t port) {
    config_lock();
    g_sys_config.mqtt_port = port;
    config_unlock();
    printf("CONFIG: MQTT Port actualizado a %d\n", port);
}

void config_set_wifi_enabled(bool enabled) {
    config_lock();
    g_sys_config.wifi_enabled = enabled ? 1 : 0;
    config_unlock();
    printf("CONFIG: WiFi Enabled status actualizado a %d\n", enabled ? 1 : 0);
}
