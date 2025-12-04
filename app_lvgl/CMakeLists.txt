set(priv_req nvs_flash esp_http_client json_generator json_parser esp_app_format esp_wifi)
set(requires esp_lvgl_port driver esp_lcd esp_lcd_touch esp_lcd_touch_ft5x06 esp_lcd_ili9341 app_common json app_buzzer 
        app_led_lamp)
set(srcs
        "app_lvgl.c" 
        
        "app_lvgl_utils.c"
        "app_lvgl_button.c"
       
       )


idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES ${requires}
                    PRIV_REQUIRES ${priv_req})
