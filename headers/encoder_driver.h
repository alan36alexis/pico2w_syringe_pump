#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void encoder_driver_init(void);
void encoder_driver_gpio_irq(uint gpio, uint32_t events);
void encoder_set_rotate_handler(void (*handler)(int diff));
void encoder_set_button_handler(void (*handler)(bool pressed));

#ifdef __cplusplus
}
#endif

#endif // ENCODER_DRIVER_H
