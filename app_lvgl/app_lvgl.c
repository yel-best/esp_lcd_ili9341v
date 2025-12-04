#include <stdio.h>
#include <esp_event.h>

#include "esp_system.h"
#include "app_lvgl.h"
#include "esp_app_desc.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_commands.h"

#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_ft5x06.h"

static const char *TAG = "app_lvgl";

terminal_config_t *terminal_config = NULL;
terminal_output_t *terminal_output = NULL;

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* LVGL display and touch */
static lv_disp_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

/**
 * Uncomment these lines if use custom initialization commands.
 * The array should be declared as "static const" and positioned outside the function.
 */
static const ili9341_lcd_init_cmd_t lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, (uint8_t[]){0x00, 0xC1, 0X30}, 3, 0},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, (uint8_t[]){0x64, 0x03, 0X12, 0X81}, 4, 0},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, (uint8_t[]){0x85, 0x00, 0x78}, 3, 0},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, (uint8_t[]){0x39, 0x2C, 0x00, 0x34, 0x02}, 5, 0},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, (uint8_t[]){0x20}, 1, 0},
    /* Driver timing control, all=0 unit */
    {0xEA, (uint8_t[]){0x00, 0x00}, 2, 0},
    /* Power control 1, GVDD=4.75V */
    {0xC0, (uint8_t[]){0x13}, 1, 0},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, (uint8_t[]){0x13}, 1, 0},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, (uint8_t[]){0x1C, 0x35}, 2, 0},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, (uint8_t[]){0xC8}, 1, 0},

    /* Invert colors */
    // {LCD_CMD_INVON, (uint8_t[]){0x00}, 1, 0},
    {0x36, (uint8_t[]){0x08}, 1, 0},
    {0xB6, (uint8_t[]){0x08, 0x82}, 2, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0xF6, (uint8_t[]){0x01, 0x30}, 2, 0},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, (uint8_t[]){0x00, 0x1B}, 2, 0},
    /* Enable 3G, disabled */
    {0xF2, (uint8_t[]){0x00}, 1, 0},
    /* Gamma set, curve 1 */
    {0x26, (uint8_t[]){0x01}, 1, 0},
    /* Positive gamma correction */
    {0xE0, (uint8_t[]){0x0F, 0x35, 0x31, 0x0B, 0x0E, 0x06, 0x49, 0xA7, 0x33, 0x07, 0x0F, 0x03, 0x0C, 0x0A, 0x00}, 15, 0},
    /* Negative gamma correction */
    {0xE1, (uint8_t[]){0x00, 0x0A, 0x0F, 0x04, 0x11, 0x08, 0x36, 0x58, 0x4D, 0x07, 0x10, 0x0C, 0x32, 0x34, 0x0F}, 15, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, (uint8_t[]){0x07}, 1, 0},
    /* Display function control */
    {0xB6, (uint8_t[]){0x08, 0x82, 0x27}, 3, 0},
};

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LVGL_LCD_GPIO_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    
    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    // const spi_bus_config_t buscfg = {
    //     .sclk_io_num = LVGL_LCD_GPIO_SCLK,
    //     .mosi_io_num = LVGL_LCD_GPIO_MOSI,
    //     .miso_io_num = LVGL_LCD_GPIO_MISO,
    //     .quadwp_io_num = GPIO_NUM_NC,
    //     .quadhd_io_num = GPIO_NUM_NC,
    //     .max_transfer_sz = LVGL_LCD_H_RES * LVGL_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    // };
    const spi_bus_config_t buscfg = ILI9341_PANEL_BUS_SPI_CONFIG(
        LVGL_LCD_GPIO_SCLK,
        LVGL_LCD_GPIO_MOSI,
        LVGL_LCD_H_RES * LVGL_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t));

    ESP_RETURN_ON_ERROR(spi_bus_initialize(LVGL_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LVGL_LCD_GPIO_DC,
        .cs_gpio_num = LVGL_LCD_GPIO_CS,
        .pclk_hz = LVGL_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = LVGL_LCD_CMD_BITS,
        .lcd_param_bits = LVGL_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LVGL_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    ili9341_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(ili9341_lcd_init_cmd_t),
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LVGL_LCD_GPIO_RST,
        // .color_space = LVGL_LCD_COLOR_SPACE,
        .bits_per_pixel = LVGL_LCD_BITS_PER_PIXEL,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .vendor_config = &vendor_config,    // Uncomment this line if use custom initialization commands
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);

    esp_lcd_panel_swap_xy(lcd_panel, true);
    esp_lcd_panel_mirror(lcd_panel, false, false);
    esp_lcd_panel_invert_color(lcd_panel, true);

    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_LOGI(TAG, "Turn on LCD backlight");
    ESP_ERROR_CHECK(gpio_set_level(LVGL_LCD_GPIO_BL, LVGL_LCD_BL_ON_LEVEL));

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(LVGL_LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_touch_init(void)
{
#if CONFIG_ESP_TERMINAL_V1_SUPPORT
    /* Initilize I2C */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LVGL_TOUCH_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = LVGL_TOUCH_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LVGL_TOUCH_I2C_CLK_HZ};
#else

    /* Initilize I2C */
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LVGL_TOUCH_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = LVGL_TOUCH_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = LVGL_TOUCH_I2C_CLK_HZ};
#endif

    ESP_RETURN_ON_ERROR(i2c_param_config(LVGL_TOUCH_I2C_NUM, &i2c_conf), TAG, "I2C configuration failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(LVGL_TOUCH_I2C_NUM, i2c_conf.mode, 0, 0, 0), TAG, "I2C initialization failed");

    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LVGL_LCD_H_RES,
        .y_max = LVGL_LCD_V_RES,
        .rst_gpio_num = LVGL_TOUCH_GPIO_RST, // Shared with LCD reset
        .int_gpio_num = LVGL_TOUCH_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)LVGL_TOUCH_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &touch_handle);
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 6,         /* LVGL task priority */
        .task_stack = 16384,         /* LVGL task stack size */
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = LVGL_LCD_H_RES * LVGL_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = LVGL_LCD_DRAW_BUFF_DOUBLE,
        .hres = LVGL_LCD_V_RES,
        .vres = LVGL_LCD_H_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = lvgl_disp,
        .handle = touch_handle,
    };
    lvgl_touch_indev = lvgl_port_add_touch(&touch_cfg);

    return ESP_OK;
}


void app_lvgl_start(void)
{

    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

    /* Touch initialization */
    ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Task lock */
    lvgl_port_lock(0);

    /* Show LVGL objects */
    app_lvgl_xxxxx();

     /* Task unlock */
    lvgl_port_unlock();

    // lvgl_task();
}