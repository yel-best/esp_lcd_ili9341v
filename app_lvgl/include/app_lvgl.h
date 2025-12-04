#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include <sdkconfig.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* LCD size */
#define LVGL_LCD_H_RES (240)
#define LVGL_LCD_V_RES (320)

/* LCD settings */
#define LVGL_LCD_SPI_NUM (SPI2_HOST)
#define LVGL_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define LVGL_LCD_CMD_BITS (8)
#define LVGL_LCD_PARAM_BITS (8)
#define LVGL_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_RGB)
#define LVGL_LCD_BITS_PER_PIXEL (16)
#define LVGL_LCD_DRAW_BUFF_DOUBLE (1)
#define LVGL_LCD_DRAW_BUFF_HEIGHT (80)
#define LVGL_LCD_BL_ON_LEVEL (1)

/* LCD pins */
#define LVGL_LCD_GPIO_SCLK (GPIO_NUM_12)
#define LVGL_LCD_GPIO_MOSI (GPIO_NUM_11)
#define LVGL_LCD_GPIO_MISO (GPIO_NUM_13)
#define LVGL_LCD_GPIO_RST (GPIO_NUM_4)
#define LVGL_LCD_GPIO_DC (GPIO_NUM_7)
#define LVGL_LCD_GPIO_CS (GPIO_NUM_10)
#define LVGL_LCD_GPIO_BL (GPIO_NUM_21)

/* Touch settings */
#define LVGL_TOUCH_I2C_NUM (0)
#define LVGL_TOUCH_I2C_CLK_HZ (400000)

#if CONFIG_ESP_TERMINAL_V1_SUPPORT
/* old ESP32S3 N8R2 LCD touch pins */
#define LVGL_TOUCH_I2C_SCL (GPIO_NUM_35)
#define LVGL_TOUCH_I2C_SDA (GPIO_NUM_36)
#define LVGL_TOUCH_GPIO_RST (GPIO_NUM_37)
#define LVGL_TOUCH_GPIO_INT (GPIO_NUM_39)

#else
/* ESP32S3 N16R8 LCD touch pins */
#define LVGL_TOUCH_I2C_SCL (GPIO_NUM_41)
#define LVGL_TOUCH_I2C_SDA (GPIO_NUM_40)
#define LVGL_TOUCH_GPIO_RST (GPIO_NUM_42)
#define LVGL_TOUCH_GPIO_INT (GPIO_NUM_39)
#endif

    void app_lvgl_start(void);

#ifdef __cplusplus
}
#endif
