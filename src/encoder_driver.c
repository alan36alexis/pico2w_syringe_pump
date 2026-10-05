#include "encoder_driver.h"

#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "lvgl.h"

#define ENCODER_CLK_PIN 10
#define ENCODER_DT_PIN 11
#define ENCODER_SW_PIN 12

static volatile int16_t encoder_diff = 0;
static volatile bool encoder_pressed = false;
static void (*rotate_handler)(int diff) = NULL;
static void (*button_handler)(bool pressed) = NULL;

void encoder_driver_gpio_irq(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == ENCODER_CLK_PIN) {
        bool clk = gpio_get(ENCODER_CLK_PIN);
        bool dt = gpio_get(ENCODER_DT_PIN);
        if (clk) {
            int16_t delta = dt ? -1 : 1;
            encoder_diff += delta;
        }
    } else if (gpio == ENCODER_SW_PIN) {
        encoder_pressed = !gpio_get(ENCODER_SW_PIN);
    }
}

static void encoder_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    (void)drv;

    int16_t diff = __atomic_exchange_n(&encoder_diff, 0, __ATOMIC_RELAXED);
    bool pressed = __atomic_load_n(&encoder_pressed, __ATOMIC_RELAXED);

    data->enc_diff = diff;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

    if (diff != 0 && rotate_handler) {
        rotate_handler(diff);
    }
    if (button_handler) {
        button_handler(pressed);
    }
}

void encoder_driver_init(void) {
    gpio_init(ENCODER_CLK_PIN);
    gpio_set_dir(ENCODER_CLK_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_CLK_PIN);

    gpio_init(ENCODER_DT_PIN);
    gpio_set_dir(ENCODER_DT_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_DT_PIN);

    gpio_init(ENCODER_SW_PIN);
    gpio_set_dir(ENCODER_SW_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_SW_PIN);

    gpio_set_irq_enabled(ENCODER_CLK_PIN, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(ENCODER_SW_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read_cb;
    lv_indev_drv_register(&indev_drv);
}

void encoder_set_rotate_handler(void (*handler)(int diff)) {
    rotate_handler = handler;
}

void encoder_set_button_handler(void (*handler)(bool pressed)) {
    button_handler = handler;
}
