#include <stdio.h>
#include "touch_driver.h"
#include "lvgl.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define PIN_TCS 21
#define PIN_TIRQ 14
#define PIN_CS 17

spi_inst_t *TFT_SPI = spi0;

// Dimensiones de tu pantalla (ajusta si es necesario)
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320

// Valores de calibración RAW (ajusta estos números si el cursor no sigue tu dedo)
#define RAW_X_MIN 200
#define RAW_X_MAX 3800
#define RAW_Y_MIN 200
#define RAW_Y_MAX 3800

static volatile bool touch_pressed = false;

static lv_indev_drv_t indev_drv;

void touch_driver_gpio_irq(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == PIN_TIRQ) {
        // Si el pin está en bajo (0), hay presión
        touch_pressed = !gpio_get(PIN_TIRQ);
    }
}

static uint16_t touch_read_raw(uint8_t cmd) {
    uint8_t rx[2];
    
    // Bajamos la velocidad del SPI temporalmente para el Touch (XPT2046 no soporta 8MHz)
    gpio_put (PIN_CS, 1);
    
    spi_set_baudrate(TFT_SPI, 1500000); // 1.5 MHz es seguro
    gpio_put(PIN_TCS, 0); // Seleccionamos el chip táctil
    
    spi_write_blocking(TFT_SPI, &cmd, 1);
    spi_read_blocking(TFT_SPI, 0, rx, 2);
    
    gpio_put(PIN_TCS, 1); // Deseleccionamos

    // Restauramos la velocidad para el Display
    spi_set_baudrate(TFT_SPI, 20000000);

    // El XPT2046 devuelve 12 bits en los 2 bytes recibidos
    uint16_t res = (rx[0] << 8) | rx[1];
    return res >> 4; 
}

static bool touch_get_xy(uint16_t *x, uint16_t *y) {
    // 2. REVISAR LA BANDERA
    // Si la interrupción no detectó nada, salimos rápido
    if (!touch_pressed) return false;

    // Si detectó algo, hacemos la lectura SPI real
    uint16_t raw_x = touch_read_raw(0x90);
    uint16_t raw_y = touch_read_raw(0xD0);

    int32_t x_max = 2000;
    int32_t y_max = 2000;

    *x = (x_max - raw_x) * SCREEN_WIDTH / x_max;
    *y = (y_max - raw_y) * SCREEN_HEIGHT / y_max;
    *x = SCREEN_WIDTH - 1 - *x;
    *y = SCREEN_HEIGHT - 1 - *y;

    // Verificación de doble check: si el pin volvió a subir, reseteamos la bandera
    if (gpio_get(PIN_TIRQ)) touch_pressed = false;


    // 🔥 PRINTF DE TESTEO 🔥
    // Muestra: Valores Crudos -> Coordenadas calculadas en Píxeles
    printf("TOUCH -> Raw[%4d, %4d] | Pixel[%3d, %3d]\n", raw_x, raw_y, *x, *y);

    return true;
}

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    uint16_t x, y;

    if (touch_get_xy(&x, &y)) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void touch_driver_init(void) {
    // Inicializar CS del Touch
    gpio_init(PIN_TCS);
    gpio_set_dir(PIN_TCS, GPIO_OUT);
    gpio_put(PIN_TCS, 1); // Inicia desactivado

    // Inicializar TIRQ (Interrupción)
    gpio_init(PIN_TIRQ);
    gpio_set_dir(PIN_TIRQ, GPIO_IN);
    gpio_pull_up(PIN_TIRQ);

    gpio_set_irq_enabled(PIN_TIRQ, GPIO_IRQ_EDGE_FALL, true);

    // Registrar el dispositivo de entrada en LVGL
    
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    lv_indev_drv_register(&indev_drv);
}