#ifndef HONEYWELL_SPI_H
#define HONEYWELL_SPI_H

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>

// Sensor status codes
typedef enum {
  HONEYWELL_HSC_STATUS_NORMAL = 0, // Normal operation, valid data
  HONEYWELL_HSC_STATUS_STALE = 1,  // Data has already been read
  HONEYWELL_HSC_STATUS_FAULT = 2,  // Device in command mode or fault state
  HONEYWELL_HSC_STATUS_DIAG = 3    // Diagnostic fault, replace sensor
} honeywell_hsc_status_t;

// Sensor data structure
typedef struct {
  honeywell_hsc_status_t status;
  float pressure_psi;
  float temperature_c;
} honeywell_hsc_data_t;

// Context structure for the sensor instance
typedef struct {
  spi_inst_t *spi_port;
  uint cs_pin;
  float p_min; // Minimum pressure rating (-100 psi for HSCMRRD100PDSA3)
  float p_max; // Maximum pressure rating (100 psi for HSCMRRD100PDSA3)
} honeywell_hsc_t;

/**
 * @brief Initializes the Honeywell HSC SPI sensor context.
 *
 * @param sensor Pointer to sensor context structure.
 * @param spi_port Pointer to the hardware SPI instance (e.g., spi0).
 * @param cs_pin Chip select GPIO pin number.
 * @param p_min Minimum pressure range in PSI (e.g. -100).
 * @param p_max Maximum pressure range in PSI (e.g. 100).
 */
void honeywell_hsc_init(honeywell_hsc_t *sensor, spi_inst_t *spi_port,
                        uint cs_pin, float p_min, float p_max);

/**
 * @brief Reads data from the Honeywell sensor over SPI.
 *
 * @param sensor Pointer to configured sensor context structure.
 * @param data Pointer to data structure to store results.
 * @return true if communication succeeded and data was read.
 */
bool honeywell_hsc_read(honeywell_hsc_t *sensor, honeywell_hsc_data_t *data);

#endif // HONEYWELL_SPI_H
