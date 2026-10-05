#include "pico/stdlib.h"
#include "lvgl.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "display_driver.h"
#include "encoder_driver.h"
#include "modulo_tft.h"
#include "touch_driver.h"
#include "ui/ui.h"

static void modulo_tft_gpio_irq(uint gpio, uint32_t events) {
    touch_driver_gpio_irq(gpio, events);
    encoder_driver_gpio_irq(gpio, events);
}

void modulo_tft_init(void) {
    //stdio_init_all();
    lv_init();
    gpio_set_irq_callback(modulo_tft_gpio_irq);
    irq_set_enabled(IO_IRQ_BANK0, true);
    display_driver_init();
    touch_driver_init();
    //encoder_driver_init();
    ui_init();
}

void modulo_tft_run_once(void) {
    lv_tick_inc(5);
    lv_timer_handler();
    ui_tick();
}
