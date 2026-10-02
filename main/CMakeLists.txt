idf_component_register(SRCS "main.c" "weather.c" "sat.c" "png.c" "ui.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_http_client esp_http_server
                                json esp_lcd driver esp_rom esp_timer esp-tls mbedtls)
