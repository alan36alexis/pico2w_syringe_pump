#include "honeywell_spi.h"
#include <string.h>

// Calibration constants for 10% to 90% transfer function (14-bit resolution)
#define OUTPUT_MIN 1638.0f  // 10% of 2^14
#define OUTPUT_MAX 14745.0f // 90% of 2^14

void honeywell_hsc_init(honeywell_hsc_t *sensor, spi_inst_t *spi_port,
                        uint cs_pin, float p_min, float p_max) {
  if (!sensor)
    return;

  sensor->spi_port = spi_port;
  sensor->cs_pin = cs_pin;
  sensor->p_min = p_min;
  sensor->p_max = p_max;
}

bool honeywell_hsc_read(honeywell_hsc_t *sensor, honeywell_hsc_data_t *data) {
  if (!sensor || !data || !sensor->spi_port)
    return false;

  uint8_t buffer[4] = {0, 0, 0, 0};

  // Assert CS (active low)
  gpio_put(sensor->cs_pin, 0);
  sleep_us(10); // Short delay to allow sensor to output data

  // Read 4 bytes from the sensor
  int bytes_read = spi_read_blocking(sensor->spi_port, 0x00, buffer, 4);

  // De-assert CS
  gpio_put(sensor->cs_pin, 1);

  if (bytes_read != 4) {
    return false;
  }

  // Extract status (top 2 bits of the first byte)
  data->status = (honeywell_hsc_status_t)((buffer[0] >> 6) & 0x03);

  // Extract 14-bit bridge pressure (lower 6 bits of byte 0, all of byte 1)
  uint16_t bridge_data = ((buffer[0] & 0x3F) << 8) | buffer[1];

  // Transfer function: P = (Output - OutputMin) * (Pmax - Pmin) / (OutputMax -
  // OutputMin) + Pmin
  data->pressure_psi = ((float)bridge_data - OUTPUT_MIN) *
                           (sensor->p_max - sensor->p_min) /
                           (OUTPUT_MAX - OUTPUT_MIN) +
                       sensor->p_min;

  // Extract 11-bit temperature (all of byte 2, top 3 bits of byte 3)
  uint16_t temp_data = (buffer[2] << 3) | ((buffer[3] >> 5) & 0x07);

  // Transfer function: T = Output * 200 / 2047 - 50
  data->temperature_c = ((float)temp_data * 200.0f / 2047.0f) - 50.0f;

  return true;
}
