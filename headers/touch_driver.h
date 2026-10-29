#ifndef TOUCH_DRIVER_H
#define TOUCH_DRIVER_H

#include <stdint.h>
#include "hardware/gpio.h"

void touch_driver_init(void);
void touch_driver_gpio_irq(uint gpio, uint32_t events);

#endif