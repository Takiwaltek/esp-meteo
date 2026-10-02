#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_lvgl_port.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "config.h"
#include "ui.h"
#include "sat.h"
#include "weather.h"

static const char *TAG = "meteo";
static EventGroupHandle_t wifi_eg;
#define WIFI_OK BIT0
static led_strip_handle_t led;
static volatile bool sat_mode;
static volatile bool sat_request;
static weather_t wx;
static bool wx_valid;

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) { xEventGroupClearBits(wifi_eg, WIFI_OK); esp_wifi_connect(); }
    else if (base == IP_EVENT) xEventGroupSetBits(wifi_eg, WIFI_OK);
}

static void wifi_start(void)
{
    wifi_eg = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&ic);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL);
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
}

static void display_init(void)
{
    gpio_config_t bl = { .pin_bit_mask = 1ULL << PIN_LCD_BL, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&bl);
    gpio_set_level(PIN_LCD_BL, 1);

    spi_bus_config_t bus = { .mosi_io_num = PIN_LCD_MOSI, .miso_io_num = -1, .sclk_io_num = PIN_LCD_SCLK,
                             .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = LCD_W * 40 * 2 };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t ioc = { .dc_gpio_num = PIN_LCD_DC, .cs_gpio_num = PIN_LCD_CS, .pclk_hz = 40 * 1000 * 1000,
                                          .lcd_cmd_bits = 8, .lcd_param_bits = 8, .spi_mode = 0, .trans_queue_depth = 10 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &ioc, &io));
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_dev_config_t pc = { .reset_gpio_num = PIN_LCD_RST, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR, .bits_per_pixel = 16 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io, &pc, &panel));
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_disp_on_off(panel, true);

    lvgl_port_cfg_t lc = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lc));
    lvgl_port_display_cfg_t dc = {
        .io_handle = io, .panel_handle = panel, .buffer_size = LCD_W * 40, .double_buffer = true,
        .hres = LCD_W, .vres = LCD_H, .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = { .swap_xy = false, .mirror_x = true, .mirror_y = false },
        .flags = { .buff_dma = true, .swap_bytes = true },
    };
    lvgl_port_add_disp(&dc);
}

static void led_set(int code, int is_day)
{
    uint8_t r = 0, g = 0, b = 0;
    if (code == 0) { r = 40; g = 30; }                 /* soleil : jaune */
    else if (code <= 2) { r = 20; g = 20; b = 5; }     /* peu nuageux */
    else if (code == 3 || code == 45 || code == 48) { r = g = b = 10; } /* couvert : blanc */
    else if (code >= 95) { r = 40; b = 40; }           /* orage : violet */
    else if (code >= 71 && code <= 77) { g = 20; b = 40; } /* neige : cyan */
    else { b = 40; }                                   /* pluie : bleu */
    if (!is_day) { r /= 3; g /= 3; b /= 3; }
    led_strip_set_pixel(led, 0, r, g, b);
    led_strip_refresh(led);
}

static void sat_task(void *arg)
{
    ui_sat_start();
    sat_load_all(ui_sat_progress);
    if (sat_mode) ui_sat_show();
    vTaskDelete(NULL);
}

static void button_task(void *arg)
{
    gpio_config_t bc = { .pin_bit_mask = 1ULL << PIN_BOOT_BTN, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&bc);
    for (;;) {
        if (gpio_get_level(PIN_BOOT_BTN) == 0) {
            int ms = 0;
            while (gpio_get_level(PIN_BOOT_BTN) == 0 && ms < LONG_PRESS_MS) { vTaskDelay(pdMS_TO_TICKS(20)); ms += 20; }
            if (ms >= LONG_PRESS_MS) {
                if (sat_mode) { sat_mode = false; ui_sat_stop(); }
                while (gpio_get_level(PIN_BOOT_BTN) == 0) vTaskDelay(pdMS_TO_TICKS(20));
            } else if (!sat_mode) {
                sat_mode = true;
                xTaskCreate(sat_task, "sat", 8192, NULL, 4, NULL);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }

    led_strip_config_t lcfg = { .strip_gpio_num = PIN_LED, .max_leds = 1 };
    led_strip_rmt_config_t rcfg = { .resolution_hz = 10 * 1000 * 1000 };
    led_strip_new_rmt_device(&lcfg, &rcfg, &led);
    led_strip_clear(led);

    display_init();
    ui_init();
    ui_show_status("Connexion Wi-Fi...");
    setenv("TZ", TZ_PARIS, 1);
    tzset();
    wifi_start();
    xEventGroupWaitBits(wifi_eg, WIFI_OK, pdFALSE, pdTRUE, portMAX_DELAY);

    esp_sntp_config_t sc = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&sc);
    xTaskCreate(button_task, "btn", 4096, NULL, 5, NULL);

    int64_t last_fetch = -1000000000LL;
    for (;;) {
        ui_update_clock();
        int64_t now = esp_timer_get_time() / 1000000;
        if (!sat_mode && (now - last_fetch >= 900 || (!wx_valid && now - last_fetch >= 30))) {
            last_fetch = now;
            if (weather_fetch(&wx)) {
                wx_valid = true;
                ui_show_weather(&wx);
                led_set(wx.code, wx.is_day);
            } else {
                ESP_LOGW(TAG, "échec météo");
                if (!wx_valid) ui_show_status("Météo indisponible");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
