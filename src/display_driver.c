#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "lvgl.h"
#include "display_driver.h"

// ---------------- CONFIG ----------------

extern spi_inst_t *TFT_SPI;

#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_MISO 16

#define PIN_CS   17
#define PIN_DC   15
#define PIN_RST  20
#define PIN_BL   22

#define TFT_WIDTH  480
#define TFT_HEIGHT 320

#define SPI_FREQ_DISP 20000000

// ---------------- LVGL ----------------

static lv_disp_t *disp;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;   // 🔥 CRÍTICO: STATIC

// buffer más grande (mejor estabilidad)
static lv_color_t buf[TFT_WIDTH * 40];

// ---------------- LOW LEVEL ----------------

static void tft_cmd(uint8_t cmd)
{
    gpio_put(PIN_DC, 0);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(TFT_SPI, &cmd, 1);
    gpio_put(PIN_CS, 1);
}

static void tft_data(const uint8_t *data, size_t len)
{
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);

    const size_t CHUNK = 4096;

    while (len > 0) {
        size_t to_send = (len > CHUNK) ? CHUNK : len;
        spi_write_blocking(TFT_SPI, data, to_send);
        data += to_send;
        len -= to_send;
    }

    gpio_put(PIN_CS, 1);
}

static void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];

    tft_cmd(0x2A);
    d[0] = x0 >> 8; d[1] = x0 & 0xFF;
    d[2] = x1 >> 8; d[3] = x1 & 0xFF;
    tft_data(d, 4);

    tft_cmd(0x2B);
    d[0] = y0 >> 8; d[1] = y0 & 0xFF;
    d[2] = y1 >> 8; d[3] = y1 & 0xFF;
    tft_data(d, 4);

    tft_cmd(0x2C);
}

// ---------------- LVGL CALLBACK ----------------

static void flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    uint32_t len = w * h;

    tft_set_window(area->x1, area->y1, area->x2, area->y2);

    // NO swap manual (ya usás LV_COLOR_16_SWAP)
    tft_data((uint8_t *)px_map, len * 2);

    lv_disp_flush_ready(disp_drv);
}

// ---------------- INIT ST7796 ----------------

static void st7796_init(void)
{
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(120);

    tft_cmd(0x01); // SW reset
    sleep_ms(150);

    // unlock
    tft_cmd(0xF0); uint8_t d = 0xC3; tft_data(&d, 1);
    tft_cmd(0xF0); d = 0x96; tft_data(&d, 1);

    // pixel format
    tft_cmd(0x3A); d = 0x55; tft_data(&d, 1);

    // Con MV activo, MX y MY invierten ambos ejes; conserva BGR.
    tft_cmd(0x36); d = 0xE8; tft_data(&d, 1);

    // sleep out
    tft_cmd(0x11);
    sleep_ms(120);

    // display on
    tft_cmd(0x29);
    sleep_ms(50);
}

// ---------------- PUBLIC INIT ----------------

void display_driver_init(void)
{
    spi_init(TFT_SPI, SPI_FREQ_DISP);  // 🔥 más rápido y estable
    spi_set_format(TFT_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_init(PIN_DC);  gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_init(PIN_BL);  gpio_set_dir(PIN_BL, GPIO_OUT);

    gpio_put(PIN_CS, 1);
    gpio_put(PIN_BL, 1);

    st7796_init();

    // LVGL init display
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, TFT_WIDTH * 40);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = TFT_WIDTH;
    disp_drv.ver_res = TFT_HEIGHT;
    disp_drv.flush_cb = flush_cb;
    disp_drv.draw_buf = &draw_buf;

    disp = lv_disp_drv_register(&disp_drv);
}