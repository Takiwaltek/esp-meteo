idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_i2c esp_driver_i2s esp_lcd esp_lcd_ili9341 esp_event esp_netif esp_wifi esp_http_client esp-tls esp_rom mbedtls nvs_flash bt lvgl espressif__esp_lvgl_adapter espressif__led_strip espressif__json_parser)
