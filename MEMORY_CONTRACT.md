# Contrato de Estructura de Memoria — Bomba de Infusión a Jeringa IoT

Documento de referencia para el firmware (Pico 2W / RP2350).  
Define el layout de la memoria flash, los datos persistentes y las reglas de acceso.

---

## Mapa de memoria flash (RP2350 — 4 MB)

```
┌─────────────────────────────────────┬──────────┬────────────────────────────┐
│ Región                              │ Tamaño   │ Dirección (FLASH_BASE=0x10000000) │
├─────────────────────────────────────┼──────────┼────────────────────────────┤
│ Código + datos (firmware)           │ Variable │ 0x10000000  → dinámico      │
│ (gestionado por el linker)          │ ≤ 3.5 MB │                            │
├─────────────────────────────────────┼──────────┼────────────────────────────┤
│ [RESERVADO] LittleFS — Logger +     │ 256 KB   │ 0x103C0000 → 0x103FFFFF    │
│ Perfiles de jeringa (futuro)        │ (64 sect)│                            │
├─────────────────────────────────────┼──────────┼────────────────────────────┤
│ SystemConfig (raw flash)            │ 4 KB     │ 0x103FF000 → 0x103FFFFF    │
│ 1 sector de erase — ACTIVO HOY      │ 1 sector │ = PICO_FLASH_SIZE_BYTES    │
│                                     │          │   - FLASH_SECTOR_SIZE      │
└─────────────────────────────────────┴──────────┴────────────────────────────┘
```

### Constante de referencia en código

```c
// En config_manager.c
#define CONFIG_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
// Para Pico 2W (4 MB): 0x400000 - 0x1000 = 0x3FF000
// Dirección física: XIP_BASE + CONFIG_FLASH_OFFSET = 0x10000000 + 0x3FF000 = 0x103FF000
```

---

## Estructura de configuración — `SystemConfig_t`

Definida en `src/config_manager.h`. Se reproduce aquí con semántica completa.

```c
#define CONFIG_MAGIC  0xA1B2C3D5U   // Cambia si el layout cambia (migración)

typedef struct __attribute__((packed)) {
    uint32_t magic;                          // [0x000] Validador de estructura
    char     wifi_ssid[MAX_SSID_LEN];        // [0x004] SSID — max 32 chars + '\0'
    char     wifi_pass[MAX_PASS_LEN];        // [0x024] Password — max 64 chars + '\0'
    char     mqtt_ip[MAX_IP_LEN];            // [0x064] IP del broker — max 16 chars + '\0'
    uint16_t mqtt_port;                      // [0x074] Puerto del broker (default: 1883)
    uint8_t  wifi_enabled;                   // [0x076] 0=WiFi OFF, 1=WiFi ON
    uint8_t  calibration_valid;              // [0x077] 0=no calibrado, 1=calibrado
    int32_t  calibrated_max_encoder_count;   // [0x078] Counts encoder de punta a punta
    char     device_id[17];                  // [0x07C] "bj-XXXXXXXX\0" (8 hex del UID RP2350)
    // ── Futuras extensiones (v2+) ──────────────────────────────────────────
    // uint8_t  touch_cal_valid;             // Calibración táctil (reservado)
    // int16_t  touch_cal[6];                // Coeficientes calibración XPT2046
    // uint8_t  occ_threshold_level;         // Nivel de oclusión 0-3 (reservado)
    // uint8_t  _pad[N];                     // Padding para alineación
    // ───────────────────────────────────────────────────────────────────────
    uint32_t crc;                            // [0x08D] CRC32 de los campos anteriores
} SystemConfig_t;
```

**Tamaño:** ~145 bytes en el layout actual (bien dentro del sector de 4 KB).  
**Padding:** El sector no usado queda en 0xFF (estado de flash erased). No se usa.

### Valores por defecto (cuando magic inválido o CRC falla)

| Campo | Default |
|---|---|
| `wifi_ssid` | `""` |
| `wifi_pass` | `""` |
| `mqtt_ip` | `"192.168.1.100"` |
| `mqtt_port` | `1883` |
| `wifi_enabled` | `0` (OFF) |
| `calibration_valid` | `0` |
| `calibrated_max_encoder_count` | `MAX_TRAVEL_ENCODER_COUNT` (system_config.h) |
| `device_id` | Generado en runtime desde `pico_unique_board_id()` |

---

## Algoritmo CRC

```c
// CRC32 estándar (polinomio 0xEDB88320 — IEEE 802.3)
// Se calcula sobre todos los campos EXCEPTO el campo crc en sí.
// Implementación: crc32_compute(const uint8_t *data, size_t len)

uint32_t crc_expected = crc32_compute(
    (const uint8_t *)&g_sys_config,
    offsetof(SystemConfig_t, crc)   // Bytes hasta crc, sin incluirlo
);
```

---

## Reglas de acceso a flash

### ¿Cuándo se puede escribir?

| Condición | ¿Permite escritura? | Razón |
|---|---|---|
| Motor detenido (`ST_READY_AT_HOME`, `ST_DISPENSE_COMPLETED`, etc.) | ✅ Sí | No hay restricción de tiempo real |
| Motor en movimiento (cualquier estado que no sea detenido) | ❌ No | Flash write bloquea XIP → el Core 1 baremetal quedaría sin código |
| Override explícito (`override_motor_check = true`) | ⚠️ Solo en init | Reservado para `config_manager_init()` al arranque |

```c
// API existente — respetar este contrato
bool config_manager_save(bool override_motor_check);
//   return false si: motor corriendo, CRC error, flash error
//   return true  si: guardado exitoso
```

### Secuencia de escritura a flash (RP2350 multicore)

El SDK de Pico requiere que flash write/erase ocurra con el Core 1 suspendido y con
interrupciones deshabilitadas en Core 0. `config_manager.c` ya implementa esto:

```
1. Verificar que motor no esté en movimiento (via global_motor / FSM state)
2. multicore_lockout_start_blocking()  ← suspende Core 1
3. uint32_t ints = save_and_disable_interrupts()
4. flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE)
5. flash_range_program(CONFIG_FLASH_OFFSET, buf, sizeof(buf))
6. restore_interrupts(ints)
7. multicore_lockout_end_blocking()    ← reanuda Core 1
```

**Tiempo de operación:** ~50 ms (erase 4KB + program 256 bytes aprox).  
**Impacto en FreeRTOS:** Todas las tareas de Core 0 se suspenden durante el paso 3–6.  
Los ticks de FreeRTOS se pierden — aceptable dado que ocurre en reposo.

---

## Calibración del encoder — campo especial

```c
// Setter: llamado desde Core 1 via flag volatile, guardado por Core 0
extern volatile bool g_calibration_dirty;

// Flujo:
// 1. Core 1 completa calibración → actualiza g_sys_config.calibrated_max_encoder_count
//                                 → g_sys_config.calibration_valid = 1
//                                 → g_calibration_dirty = true
// 2. task_logger (Core 0) detecta g_calibration_dirty
//    → llama config_manager_save(false)
//    → limpia g_calibration_dirty
```

---

## Calibración táctil — extensión planificada (v2)

Los coeficientes de calibración de la pantalla táctil (XPT2046) se almacenarán en
`SystemConfig_t` como extensión v2 cuando el magic se actualice a `0xA1B2C3D6`.

```c
// Coeficientes de transformación afín (3-punto o 4-punto):
// x_display = (A * x_raw + B * y_raw + C) / 4096
// y_display = (D * x_raw + E * y_raw + F) / 4096
int16_t touch_cal[6];     // A, B, C, D, E, F
uint8_t touch_cal_valid;  // 0=sin calibrar, 1=calibrado
```

**Regla de migración:** Si al leer flash el magic es `0xA1B2C3D5` (v1), los campos de
calibración táctil se inicializan con defaults hardcodeados hasta que el usuario realice
la calibración.

---

## LittleFS — Plan de integración futura

El sector de 256 KB reservado en `0x103C0000` está pensado para LittleFS.  
**No implementado en v1.** Cuando se implemente:

| Sistema de archivos | Contenido | Tamaño estimado |
|---|---|---|
| LittleFS | `/log/` — Registros de infusión | 200 KB |
| LittleFS | `/syr/` — Perfiles de jeringas personalizadas | 16 KB |
| LittleFS | `/cfg/` — Configuración extendida (si supera SystemConfig_t) | 4 KB |
| LittleFS | Metadatos y wear leveling LittleFS | Interno |

**Prerrequisito para habilitar LittleFS:** Asegurarse de que el linker script
(`memmap_default.ld` del SDK de Pico) o un script custom deje los 256 KB finales
libres del binario. Verificar con `objdump -h firmware.elf`.

---

## Syringe database — Plan futuro

La lista de jeringas (marca, diámetro interno, volumen máximo) se almacenará en
LittleFS como un archivo binario de registros `SyringeProfile_t` (ya definida en
`syringe_pump_api.h`).

En v1, la base de datos de jeringas está hardcodeada en flash como array `const`.

---

## Versiones

| Campo | Valor |
|---|---|
| SystemConfig magic | `0xA1B2C3D5` (v1) |
| Flash target | RP2350 — 4 MB (Pico 2W) |
| CRC | CRC32 IEEE 802.3 |
| LittleFS | No implementado (reservado) |
| Logger persistente | No implementado (reservado) |
| Esquema | v1 |
