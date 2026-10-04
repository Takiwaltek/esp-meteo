#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_event.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_lv_adapter.h"
#include "esp_wifi.h"
#include "miniz.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "lvgl.h"
#include "json_parser.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "es3n28p";

#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_RESET_PIN GPIO_NUM_48
#define LCD_BL_PIN GPIO_NUM_45
#define BOOT_BUTTON_PIN GPIO_NUM_0
#define STATUS_LED_PIN GPIO_NUM_42

#define BLE_DEVICE_LIMIT 8
#define BLE_DEVICE_NAME_SIZE 24
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT BIT1
#define WIFI_MAX_RETRIES 10
#define SATELLITE_REFRESH_MS (30 * 60 * 1000)

typedef enum {
    SCAN_VIEW_WIFI,
    SCAN_VIEW_BLUETOOTH,
    SCAN_VIEW_MICROPHONE,
    SCAN_VIEW_WEATHER,
} scan_view_t;

typedef struct {
    uint8_t address[6];
    char name[BLE_DEVICE_NAME_SIZE];
    int8_t rssi;
} ble_device_t;

#define FORECAST_DAYS 7

typedef struct {
    char data[8192];
    size_t length;
    bool overflow;
} weather_response_t;

typedef struct {
    uint8_t *data;
    size_t length;
    size_t capacity;
    bool overflow;
    bool allocation_failed;
} satellite_response_t;

static lv_obj_t *s_weather_screen;
static lv_obj_t *s_satellite_screen;
static lv_obj_t *s_forecast_screen;
static lv_obj_t *s_fc_day[FORECAST_DAYS];
static lv_obj_t *s_fc_cond[FORECAST_DAYS];
static lv_obj_t *s_fc_temp[FORECAST_DAYS];
static lv_obj_t *s_fc_rain[FORECAST_DAYS];
static lv_obj_t *s_fc_bar[FORECAST_DAYS];
static lv_obj_t *s_satellite_image;
static lv_obj_t *s_satellite_status;
static lv_obj_t *s_clock_label;
static lv_obj_t *s_status_label;
static lv_obj_t *s_wifi_signal_label;
static lv_obj_t *s_status_dot;
static lv_obj_t *s_networks_label;
static lv_obj_t *s_audio_bar;
static lv_obj_t *s_weather_label;
static lv_obj_t *s_weather_icon;
static lv_obj_t *s_rain_banner;
static lv_obj_t *s_rain_label;
static lv_obj_t *s_date_label;
static lv_obj_t *s_weather_condition_label;
static char s_weather_text[320] = "En attente des conditions...";
static led_strip_handle_t s_status_led;
static uint32_t s_last_led_color = UINT32_MAX;
static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retries;
static volatile scan_view_t s_scan_view = SCAN_VIEW_WEATHER;
static volatile bool s_wifi_connected;
static volatile bool s_satellite_refresh_requested;
static volatile bool s_satellite_view;
static volatile bool s_ble_ready;
static volatile bool s_ble_init_failed;
static uint8_t s_ble_own_address_type;
static bool s_ble_scan_active;
static ble_device_t s_ble_devices[BLE_DEVICE_LIMIT];
static size_t s_ble_device_count;
static i2s_chan_handle_t s_mic_rx_channel;
static volatile bool s_mic_ready;
static volatile bool s_mic_failed;

static void set_status_led(uint8_t red, uint8_t green, uint8_t blue)
{
    if (s_status_led == NULL) {
        return;
    }

    uint32_t color = ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
    if (color == s_last_led_color) {
        return;
    }

    if (led_strip_set_pixel(s_status_led, 0, red, green, blue) == ESP_OK &&
        led_strip_refresh(s_status_led) == ESP_OK) {
        s_last_led_color = color;
    }
}

static bool wifi_credentials_configured(void)
{
    return CONFIG_APP_WIFI_SSID[0] != '\0';
}

static esp_err_t weather_http_event_handler(esp_http_client_event_t *event)
{
    weather_response_t *body = event->user_data;
    if (event->event_id == HTTP_EVENT_ON_DATA) {
        if (event->data_len < sizeof(body->data) - body->length) {
            memcpy(body->data + body->length, event->data, event->data_len);
            body->length += event->data_len;
            body->data[body->length] = '\0';
        } else {
            body->overflow = true;
        }
    }
    return ESP_OK;
}

#define SATELLITE_IMAGE_WIDTH 240
#define SATELLITE_IMAGE_HEIGHT 320
#define SATELLITE_DECODE_WIDTH 240
#define SATELLITE_DECODE_HEIGHT 320
#define SATELLITE_MAX_RESPONSE_BYTES (128 * 1024)

static esp_err_t satellite_http_event_handler(esp_http_client_event_t *event)
{
    satellite_response_t *response = event->user_data;
    if (event->event_id != HTTP_EVENT_ON_DATA || event->data_len <= 0) {
        return ESP_OK;
    }

    size_t incoming_size = (size_t)event->data_len;
    if (incoming_size > SATELLITE_MAX_RESPONSE_BYTES - response->length) {
        response->overflow = true;
        return ESP_OK;
    }

    size_t required_size = response->length + incoming_size;
    if (required_size > response->capacity) {
        size_t new_capacity = response->capacity == 0 ? 16384 : response->capacity;
        while (new_capacity < required_size) {
            new_capacity *= 2;
        }
        if (new_capacity > SATELLITE_MAX_RESPONSE_BYTES) {
            new_capacity = SATELLITE_MAX_RESPONSE_BYTES;
        }
        uint8_t *new_data = heap_caps_realloc(response->data, new_capacity,
                                              MALLOC_CAP_8BIT);
        if (new_data == NULL) {
            response->allocation_failed = true;
            return ESP_ERR_NO_MEM;
        }
        response->data = new_data;
        response->capacity = new_capacity;
    }

    memcpy(response->data + response->length, event->data, incoming_size);
    response->length += incoming_size;
    return ESP_OK;
}

static void set_satellite_message(const char *message)
{
    if (s_satellite_status == NULL || esp_lv_adapter_lock(-1) != ESP_OK) {
        return;
    }
    lv_label_set_text(s_satellite_status, message);
    esp_lv_adapter_unlock();
}

static uint8_t paeth_predictor(uint8_t a, uint8_t b, uint8_t c)
{
    int p = (int)a + b - c;
    int pa = abs(p - a);
    int pb = abs(p - b);
    int pc = abs(p - c);
    if (pa <= pb && pa <= pc) {
        return a;
    }
    return pb <= pc ? b : c;
}

static uint32_t read_be32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
}

// Decode un PNG 8 bits a palette de 240x240 et le reduit a 120x120 en RGB565.
static bool decode_png8_to_rgb565(const uint8_t *png, size_t length, uint16_t *output)
{
    static const uint8_t signature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (length < sizeof(signature) || memcmp(png, signature, sizeof(signature)) != 0) {
        return false;
    }

    const size_t stride = SATELLITE_IMAGE_WIDTH + 1;
    const size_t raw_size = stride * SATELLITE_IMAGE_HEIGHT;
    uint8_t *raw = heap_caps_malloc(raw_size, MALLOC_CAP_8BIT);
    tinfl_decompressor *inflater = heap_caps_malloc(sizeof(*inflater), MALLOC_CAP_8BIT);
    uint8_t palette[256 * 3] = {0};
    bool header_ok = false;
    bool inflate_done = false;
    bool failed = raw == NULL || inflater == NULL;
    size_t raw_used = 0;
    size_t position = sizeof(signature);

    if (!failed) {
        tinfl_init(inflater);
    }
    while (!failed && position + 12 <= length) {
        uint32_t chunk_size = read_be32(png + position);
        const uint8_t *type = png + position + 4;
        const uint8_t *data = png + position + 8;
        if (chunk_size > length - position - 12) {
            failed = true;
            break;
        }
        if (memcmp(type, "IHDR", 4) == 0) {
            header_ok = chunk_size == 13 &&
                        read_be32(data) == SATELLITE_IMAGE_WIDTH &&
                        read_be32(data + 4) == SATELLITE_IMAGE_HEIGHT &&
                        data[8] == 8 && data[9] == 3 && data[12] == 0;
            failed = !header_ok;
        } else if (memcmp(type, "PLTE", 4) == 0) {
            size_t palette_bytes = chunk_size < sizeof(palette) ? chunk_size : sizeof(palette);
            memcpy(palette, data, palette_bytes);
        } else if (memcmp(type, "IDAT", 4) == 0 && header_ok && !inflate_done) {
            size_t input_size = chunk_size;
            size_t output_size = raw_size - raw_used;
            tinfl_status status = tinfl_decompress(
                inflater, data, &input_size, raw, raw + raw_used, &output_size,
                TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_HAS_MORE_INPUT |
                    TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
            raw_used += output_size;
            if (status < TINFL_STATUS_DONE) {
                failed = true;
            }
            inflate_done = status == TINFL_STATUS_DONE;
        } else if (memcmp(type, "IEND", 4) == 0) {
            break;
        }
        position += 12 + (size_t)chunk_size;
    }

    bool success = !failed && inflate_done && raw_used == raw_size;
    if (success) {
        for (int y = 0; y < SATELLITE_IMAGE_HEIGHT; ++y) {
            uint8_t *row = raw + (size_t)y * stride;
            uint8_t filter = row[0];
            uint8_t *pixels = row + 1;
            const uint8_t *previous = y > 0 ? pixels - stride : NULL;
            if (filter > 4) {
                success = false;
                break;
            }
            for (int x = 0; x < SATELLITE_IMAGE_WIDTH; ++x) {
                uint8_t a = x > 0 ? pixels[x - 1] : 0;
                uint8_t b = previous != NULL ? previous[x] : 0;
                uint8_t c = (x > 0 && previous != NULL) ? previous[x - 1] : 0;
                switch (filter) {
                case 1: pixels[x] = (uint8_t)(pixels[x] + a); break;
                case 2: pixels[x] = (uint8_t)(pixels[x] + b); break;
                case 3: pixels[x] = (uint8_t)(pixels[x] + ((a + b) >> 1)); break;
                case 4: pixels[x] = (uint8_t)(pixels[x] + paeth_predictor(a, b, c)); break;
                default: break;
                }
            }
        }
    }
    if (success) {
        for (int y = 0; y < SATELLITE_DECODE_HEIGHT; ++y) {
            const uint8_t *pixels = raw +             (size_t)y * stride + 1;
            for (int x = 0; x < SATELLITE_DECODE_WIDTH; ++x) {
                            const uint8_t *color = &palette[pixels[x] * 3];
                output[y * SATELLITE_DECODE_WIDTH + x] =
                    (uint16_t)(((color[0] & 0xF8) << 8) | ((color[1] & 0xFC) << 3) | (color[2] >> 3));
            }
        }
    }

    heap_caps_free(inflater);
    heap_caps_free(raw);
    return success;
}

#define SAT_FRAME_COUNT 8
#define SAT_FRAME_STEP_S (30 * 60)
#define MONTPELLIER_LATITUDE_DEG 43.6119
#define MONTPELLIER_LONGITUDE_DEG 3.8772
#define SAT_BASE_URL_FORMAT \
    "https://view.eumetsat.int/geoserver/ows?service=WMS&version=1.3.0" \
    "&request=GetMap&layers=%s,backgrounds:ne_10m_coastline," \
    "backgrounds:ne_boundary_lines_land&styles=raster,,&crs=CRS:84" \
    "&bbox=-12,35,28,61&width=240&height=320&format=image/png8" \
    "&time=%04d-%02d-%02dT%02d:%02d:00.000Z"

static uint8_t *s_sat_frames[SAT_FRAME_COUNT];
static lv_image_dsc_t s_sat_dsc[SAT_FRAME_COUNT];
static time_t s_sat_times[SAT_FRAME_COUNT];
static int s_sat_count;
static int s_sat_index;
static int s_sat_hold;
static lv_timer_t *s_sat_timer;
static bool s_satellite_infrared;
static bool s_satellite_mode_known;
static bool s_satellite_mode_daylight;

static void satellite_show_frame(int index)
{
    char text[64];
    struct tm local_time;
    localtime_r(&s_sat_times[index], &local_time);
    snprintf(text, sizeof(text), "Meteosat %s  %02d:%02d  (%d/%d)",
             s_satellite_infrared ? "IR108" : "RGB",
             local_time.tm_hour, local_time.tm_min, index + 1, s_sat_count);
    lv_image_set_src(s_satellite_image, &s_sat_dsc[index]);
    lv_label_set_text(s_satellite_status, text);
}

static void satellite_animation_cb(lv_timer_t *timer)
{
    (void)timer;
    if (s_sat_count < 2 || lv_screen_active() != s_satellite_screen) {
        return;
    }
    if (s_sat_index == s_sat_count - 1 && s_sat_hold < 4) {
        ++s_sat_hold;
        return;
    }
    s_sat_hold = 0;
    s_sat_index = (s_sat_index + 1) % s_sat_count;
    satellite_show_frame(s_sat_index);
}

static bool is_daylight_in_montpellier(time_t now)
{
    struct tm utc;
    gmtime_r(&now, &utc);
    const double pi = 3.14159265358979323846;
    const double gamma = (2.0 * pi / 365.0) * utc.tm_yday;
    const double cos_gamma = cos(gamma);
    const double sin_gamma = sin(gamma);
    const double cos_2gamma = cos(2.0 * gamma);
    const double sin_2gamma = sin(2.0 * gamma);
    const double cos_3gamma = cos(3.0 * gamma);
    const double sin_3gamma = sin(3.0 * gamma);
    const double equation_of_time = 229.18 *
        (0.000075 + 0.001868 * cos_gamma - 0.032077 * sin_gamma
         - 0.014615 * cos_2gamma - 0.040849 * sin_2gamma);
    const double solar_declination =
        0.006918 - 0.399912 * cos_gamma + 0.070257 * sin_gamma
        - 0.006758 * cos_2gamma + 0.000907 * sin_2gamma
        - 0.002697 * cos_3gamma + 0.00148 * sin_3gamma;
    const double latitude = MONTPELLIER_LATITUDE_DEG * pi / 180.0;
    const double zenith = 90.833 * pi / 180.0;
    const double cos_hour_angle =
        cos(zenith) / (cos(latitude) * cos(solar_declination))
        - tan(latitude) * tan(solar_declination);
    if (cos_hour_angle <= -1.0 || cos_hour_angle >= 1.0) {
        ESP_LOGW(TAG, "Sunrise/sunset calculation is out of range");
        return false;
    }

    const double hour_angle_deg = acos(cos_hour_angle) * 180.0 / pi;
    const double solar_noon_minutes = 720.0 - 4.0 * MONTPELLIER_LONGITUDE_DEG
                                      - equation_of_time;
    const double sunrise_minutes = solar_noon_minutes - 4.0 * hour_angle_deg;
    const double sunset_minutes = solar_noon_minutes + 4.0 * hour_angle_deg;
    const time_t utc_midnight = now - utc.tm_hour * 3600 - utc.tm_min * 60 - utc.tm_sec;
    const time_t sunrise = utc_midnight + (time_t)(sunrise_minutes * 60.0);
    const time_t sunset = utc_midnight + (time_t)(sunset_minutes * 60.0);
    return now >= sunrise && now < sunset;
}

static uint8_t *fetch_satellite_frame(time_t frame_time, const char *layer)
{
    struct tm utc;
    gmtime_r(&frame_time, &utc);
    char url[512];
    int url_length = snprintf(url, sizeof(url), SAT_BASE_URL_FORMAT, layer,
                              utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
                              utc.tm_hour, utc.tm_min);
    if (url_length < 0 || (size_t)url_length >= sizeof(url)) {
        ESP_LOGE(TAG, "Satellite URL does not fit in the request buffer");
        return NULL;
    }

    satellite_response_t response = {0};
    const esp_http_client_config_t config = {
        .url = url,
        .event_handler = satellite_http_event_handler,
        .user_data = &response,
        .timeout_ms = 20000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = client != NULL ? esp_http_client_perform(client) : ESP_ERR_NO_MEM;
    int status_code = client != NULL ? esp_http_client_get_status_code(client) : 0;
    if (client != NULL) {
        esp_http_client_cleanup(client);
    }
    if (err != ESP_OK || status_code != 200 || response.data == NULL ||
        response.overflow || response.allocation_failed) {
        ESP_LOGW(TAG, "Satellite download failed: %s, HTTP %d, bytes %u",
                 esp_err_to_name(err), status_code, (unsigned)response.length);
        heap_caps_free(response.data);
        return NULL;
    }

    uint8_t *pixels = heap_caps_calloc(
        SATELLITE_DECODE_WIDTH * SATELLITE_DECODE_HEIGHT, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    bool decoded = pixels != NULL &&
                   decode_png8_to_rgb565(response.data, response.length, (uint16_t *)pixels);
    heap_caps_free(response.data);
    if (!decoded) {
        ESP_LOGW(TAG, "Satellite PNG decode failed");
        heap_caps_free(pixels);
        return NULL;
    }
    return pixels;
}

static void fetch_montpellier_satellite_image(void)
{
    if (!s_wifi_connected) {
        set_satellite_message("Wi-Fi requis pour charger l'image");
        return;
    }
    time_t now;
    time(&now);
    if (now < 1700000000) {
        set_satellite_message("Heure non synchronisee");
        return;
    }
    const bool daylight = is_daylight_in_montpellier(now);
    const char *layer = daylight ? "msg_fes:rgb_natural" : "msg_fes:ir108";

    uint8_t *frames[SAT_FRAME_COUNT] = {0};
    time_t times[SAT_FRAME_COUNT] = {0};
    int count = 0;
    time_t base = ((now - 10 * 60) / 900) * 900;
    for (int i = 0; i < SAT_FRAME_COUNT; ++i) {
        char message[48];
        snprintf(message, sizeof(message), "Chargement image %d/%d...", i + 1, SAT_FRAME_COUNT);
        set_satellite_message(message);
        time_t frame_time = base - (time_t)i * SAT_FRAME_STEP_S;
        uint8_t *pixels = fetch_satellite_frame(frame_time, layer);
        if (pixels == NULL && i == 0) {
            base -= 900;
            frame_time = base;
            pixels = fetch_satellite_frame(frame_time, layer);
        }
        if (pixels != NULL) {
            frames[count] = pixels;
            times[count] = frame_time;
            ++count;
        }
    }

    if (count == 0) {
        set_satellite_message("Image satellite indisponible");
        return;
    }

    uint8_t *old_frames[SAT_FRAME_COUNT];
    memcpy(old_frames, s_sat_frames, sizeof(old_frames));
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        for (int i = 0; i < count; ++i) {
            heap_caps_free(frames[i]);
        }
        set_satellite_message("Affichage satellite indisponible");
        return;
    }
    memset(s_sat_frames, 0, sizeof(s_sat_frames));
    s_satellite_infrared = !daylight;
    s_satellite_mode_daylight = daylight;
    s_satellite_mode_known = true;
    for (int i = 0; i < count; ++i) {
        int slot = count - 1 - i;
        s_sat_frames[slot] = frames[i];
        s_sat_times[slot] = times[i];
        s_sat_dsc[slot] = (lv_image_dsc_t){
            .header = {
                .magic = LV_IMAGE_HEADER_MAGIC,
                .cf = LV_COLOR_FORMAT_RGB565,
                .w = SATELLITE_DECODE_WIDTH,
                .h = SATELLITE_DECODE_HEIGHT,
                .stride = SATELLITE_DECODE_WIDTH * sizeof(uint16_t),
            },
            .data_size = SATELLITE_DECODE_WIDTH * SATELLITE_DECODE_HEIGHT * sizeof(uint16_t),
            .data = frames[i],
        };
    }
    s_sat_count = count;
    s_sat_index = 0;
    s_sat_hold = 0;
    lv_image_set_scale(s_satellite_image, 256);
    satellite_show_frame(0);
    if (s_sat_timer == NULL) {
        s_sat_timer = lv_timer_create(satellite_animation_cb, 400, NULL);
    }
    esp_lv_adapter_unlock();
    for (int i = 0; i < SAT_FRAME_COUNT; ++i) {
        heap_caps_free(old_frames[i]);
    }
}

static bool json_get_first_array_double(jparse_ctx_t *json_context, const char *name,
                                        double *value)
{
    int element_count = 0;
    if (json_obj_get_array(json_context, name, &element_count) != OS_SUCCESS) {
        return false;
    }
    bool valid = element_count > 0 &&
                 json_arr_get_double(json_context, 0, value) == OS_SUCCESS;
    int leave_result = json_obj_leave_array(json_context);
    return valid && leave_result == OS_SUCCESS;
}

static void set_weather_led(int weather_code, double temperature)
{
    bool raining = (weather_code >= 51 && weather_code <= 67) ||
                   (weather_code >= 80 && weather_code <= 82) ||
                   (weather_code >= 95 && weather_code <= 99);
    if (raining) {
        set_status_led(48, 0, 0);
    } else if (temperature <= 10.0) {
        set_status_led(0, 0, 48);
    } else {
        set_status_led(0, 48, 0);
    }
}

static lv_obj_t *icon_shape(lv_obj_t *parent, int x, int y, int w, int h, int radius,
                            uint32_t color, lv_opa_t opa)
{
    lv_obj_t *shape = lv_obj_create(parent);
    lv_obj_set_size(shape, w, h);
    lv_obj_set_pos(shape, x, y);
    lv_obj_set_style_radius(shape, radius, LV_PART_MAIN);
    lv_obj_set_style_bg_color(shape, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(shape, opa, LV_PART_MAIN);
    lv_obj_set_style_border_width(shape, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(shape, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(shape, LV_DIR_NONE);
    return shape;
}

static void icon_cloud(lv_obj_t *parent, int x, int y, uint32_t color)
{
    icon_shape(parent, x, y + 12, 40, 16, 8, color, LV_OPA_COVER);
    icon_shape(parent, x + 5, y + 4, 18, 18, LV_RADIUS_CIRCLE, color, LV_OPA_COVER);
    icon_shape(parent, x + 15, y, 22, 22, LV_RADIUS_CIRCLE, color, LV_OPA_COVER);
}

static void icon_sun(lv_obj_t *parent, int x, int y, int diameter)
{
    icon_shape(parent, x - 5, y - 5, diameter + 10, diameter + 10, LV_RADIUS_CIRCLE,
               0xFFD54A, LV_OPA_30);
    icon_shape(parent, x, y, diameter, diameter, LV_RADIUS_CIRCLE, 0xFFD54A, LV_OPA_COVER);
}

static void icon_moon(lv_obj_t *parent, int x, int y, int diameter)
{
    icon_shape(parent, x, y, diameter, diameter, LV_RADIUS_CIRCLE, 0xF2E6B1, LV_OPA_COVER);
    icon_shape(parent, x + diameter / 3, y - diameter / 6, diameter, diameter,
               LV_RADIUS_CIRCLE, 0x14546A, LV_OPA_COVER);
}

static void bg_anim_x_cb(void *obj, int32_t value)
{
    lv_obj_set_x(obj, value);
}

static void bg_anim_y_cb(void *obj, int32_t value)
{
    lv_obj_set_y(obj, value);
}

static void create_background_animation(lv_obj_t *parent)
{
    static const struct { int16_t y, w; uint16_t ms; } clouds[] = {
        {70, 70, 26000}, {150, 90, 34000}, {235, 60, 22000},
    };
    for (size_t i = 0; i < sizeof(clouds) / sizeof(clouds[0]); ++i) {
        lv_obj_t *cloud = icon_shape(parent, -clouds[i].w, clouds[i].y, clouds[i].w,
                                     clouds[i].w / 3, clouds[i].w / 6, 0xFFFFFF, LV_OPA_10);
        lv_obj_remove_flag(cloud, LV_OBJ_FLAG_CLICKABLE);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, cloud);
        lv_anim_set_exec_cb(&a, bg_anim_x_cb);
        lv_anim_set_values(&a, -clouds[i].w, LCD_H_RES);
        lv_anim_set_duration(&a, clouds[i].ms);
        lv_anim_set_delay(&a, (uint32_t)i * 4000);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&a);
    }
    for (int i = 0; i < 6; ++i) {
        lv_obj_t *dot = icon_shape(parent, 20 + i * 38, 0, 4, 4, LV_RADIUS_CIRCLE,
                                   0x9CE3E0, LV_OPA_30);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, dot);
        lv_anim_set_exec_cb(&a, bg_anim_y_cb);
        lv_anim_set_values(&a, LCD_V_RES, -6);
        lv_anim_set_duration(&a, 9000 + i * 1700);
        lv_anim_set_delay(&a, (uint32_t)i * 1500);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&a);
    }
}

static void set_weather_icon(int code, bool is_day)
{
    if (s_weather_icon == NULL) {
        return;
    }
    lv_obj_clean(s_weather_icon);
    lv_obj_t *icon = s_weather_icon;

    if (code == 0 || code == 1) {
        if (is_day) {
            icon_sun(icon, 14, 14, 28);
        } else {
            icon_moon(icon, 14, 12, 28);
        }
    } else if (code == 2) {
        if (is_day) {
            icon_sun(icon, 4, 4, 20);
        } else {
            icon_moon(icon, 4, 4, 20);
        }
        icon_cloud(icon, 14, 20, 0xE8EEF2);
    } else if (code == 3) {
        icon_cloud(icon, 8, 14, 0xC9D3DA);
    } else if (code == 45 || code == 48) {
        icon_cloud(icon, 8, 6, 0xC9D3DA);
        icon_shape(icon, 8, 36, 40, 4, 2, 0xAAB7BF, LV_OPA_COVER);
        icon_shape(icon, 14, 44, 34, 4, 2, 0xAAB7BF, LV_OPA_COVER);
    } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
        icon_cloud(icon, 8, 4, 0xC9D3DA);
        icon_shape(icon, 14, 36, 6, 6, LV_RADIUS_CIRCLE, 0xFFFFFF, LV_OPA_COVER);
        icon_shape(icon, 26, 42, 6, 6, LV_RADIUS_CIRCLE, 0xFFFFFF, LV_OPA_COVER);
        icon_shape(icon, 38, 36, 6, 6, LV_RADIUS_CIRCLE, 0xFFFFFF, LV_OPA_COVER);
    } else if (code >= 95) {
        icon_cloud(icon, 8, 2, 0x8795A1);
        lv_obj_t *bolt = lv_label_create(icon);
        lv_label_set_text(bolt, LV_SYMBOL_CHARGE);
        lv_obj_set_pos(bolt, 20, 32);
        lv_obj_set_style_text_color(bolt, lv_color_hex(0xFFD54A), LV_PART_MAIN);
        lv_obj_set_style_text_font(bolt, &lv_font_montserrat_14, LV_PART_MAIN);
    } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
        icon_cloud(icon, 8, 4, 0xAEBBC4);
        icon_shape(icon, 14, 34, 4, 11, 2, 0x6EC6FF, LV_OPA_COVER);
        icon_shape(icon, 26, 38, 4, 11, 2, 0x6EC6FF, LV_OPA_COVER);
        icon_shape(icon, 38, 34, 4, 11, 2, 0x6EC6FF, LV_OPA_COVER);
    } else {
        icon_cloud(icon, 8, 14, 0xC9D3DA);
    }
}

static void set_rain_banner(int level, const char *text)
{
    if (s_rain_banner == NULL) {
        return;
    }
    const uint32_t colors[] = {0x1F6F54, 0xD9822B, 0xC0392B};
    lv_obj_set_style_bg_color(s_rain_banner, lv_color_hex(colors[level]), LV_PART_MAIN);
    lv_label_set_text(s_rain_label, text);
}

static const char *short_condition(int code)
{
    if (code == 0) return "Soleil";
    if (code == 1) return "Peu nuageux";
    if (code == 2) return "Nuageux";
    if (code == 3) return "Couvert";
    if (code == 45 || code == 48) return "Brouillard";
    if ((code >= 51 && code <= 57)) return "Bruine";
    if ((code >= 61 && code <= 67) || (code >= 80 && code <= 82)) return "Pluie";
    if ((code >= 71 && code <= 77) || code == 85 || code == 86) return "Neige";
    if (code >= 95) return "Orage";
    return "Variable";
}

static uint32_t condition_color(int code)
{
    if (code == 0 || code == 1) return 0xFFD54A;
    if (code == 2 || code == 3 || code == 45 || code == 48) return 0xC9D3DA;
    if (code >= 95) return 0xFF8A6B;
    if (code >= 71 && code <= 86 && code != 80 && code != 81 && code != 82) return 0xFFFFFF;
    return 0x6EC6FF;
}

static void set_forecast_row(int i, const char *date, int code, double tmin, double tmax, int rain)
{
    static const char *const names[] = {"Dim", "Lun", "Mar", "Mer", "Jeu", "Ven", "Sam"};
    int y = 0, m = 0, d = 0;
    char buf[32];
    if (sscanf(date, "%d-%d-%d", &y, &m, &d) == 3) {
        struct tm t = {.tm_year = y - 1900, .tm_mon = m - 1, .tm_mday = d, .tm_hour = 12};
        mktime(&t);
        snprintf(buf, sizeof(buf), "%s %02d", i == 0 ? "Auj." : names[t.tm_wday], d);
    } else {
        snprintf(buf, sizeof(buf), "J+%d", i);
    }
    lv_label_set_text(s_fc_day[i], buf);
    lv_label_set_text(s_fc_cond[i], short_condition(code));
    lv_obj_set_style_text_color(s_fc_cond[i], lv_color_hex(condition_color(code)), LV_PART_MAIN);
    snprintf(buf, sizeof(buf), "%.0f° / %.0f°", tmin, tmax);
    lv_label_set_text(s_fc_temp[i], buf);
    snprintf(buf, sizeof(buf), "%d%%", rain);
    lv_label_set_text(s_fc_rain[i], buf);
    uint32_t color = rain >= 60 ? 0xFF6B5B : (rain >= 30 ? 0xFFB347 : 0x7FE0A8);
    lv_obj_set_style_text_color(s_fc_rain[i], lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_fc_bar[i], lv_color_hex(color), LV_PART_INDICATOR);
    lv_bar_set_value(s_fc_bar[i], rain, LV_ANIM_OFF);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        ++s_wifi_retries;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_wifi_retries = 0;
        s_wifi_connected = true;
        if (s_wifi_event_group != NULL) {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

static void clock_task(void *arg)
{
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    char previous_clock_text[64] = "";

    while (true) {
        time_t now;
        time(&now);
        char clock_text[32];
        char date_text[32] = "";
        if (now < 1700000000) {
            snprintf(clock_text, sizeof(clock_text), "--:--");
        } else {
            struct tm local_time;
            localtime_r(&now, &local_time);
            strftime(clock_text, sizeof(clock_text), "%H:%M", &local_time);
            static const char *const days[] = {"dim.", "lun.", "mar.", "mer.", "jeu.", "ven.", "sam."};
            static const char *const months[] = {"janv.", "fevr.", "mars", "avr.", "mai", "juin",
                                                 "juil.", "aout", "sept.", "oct.", "nov.", "dec."};
            snprintf(date_text, sizeof(date_text), "%s %d %s", days[local_time.tm_wday],
                     local_time.tm_mday, months[local_time.tm_mon]);
        }

        char wifi_text[96] = "";
        if (s_wifi_connected) {
            wifi_ap_record_t ap = {0};
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                const char *quality = ap.rssi >= -60 ? "Fort" : (ap.rssi >= -70 ? "Moyen" : "Faible");
                const char *color = ap.rssi >= -60 ? "7FE0A8" : (ap.rssi >= -70 ? "FFD27A" : "FF6B5B");
                snprintf(wifi_text, sizeof(wifi_text), "#%s %s %d dBm#\n%.32s", color, quality,
                         ap.rssi, (const char *)ap.ssid);
            }
        }
        if (s_wifi_signal_label != NULL && esp_lv_adapter_lock(-1) == ESP_OK) {
            lv_label_set_text(s_wifi_signal_label, wifi_text);
            esp_lv_adapter_unlock();
        }

        char combined[64];
        snprintf(combined, sizeof(combined), "%s|%s", clock_text, date_text);
        if (strcmp(combined, previous_clock_text) != 0 &&
            esp_lv_adapter_lock(-1) == ESP_OK) {
            lv_label_set_text(s_clock_label, clock_text);
            lv_label_set_text(s_date_label, date_text);
            esp_lv_adapter_unlock();
            snprintf(previous_clock_text, sizeof(previous_clock_text), "%s", combined);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t es8311_write_register(i2c_master_dev_handle_t codec, uint8_t reg, uint8_t value)
{
    const uint8_t command[2] = {reg, value};
    return i2c_master_transmit(codec, command, sizeof(command), 100);
}

static void set_mic_message(const char *status, const char *message)
{
    if (s_scan_view != SCAN_VIEW_MICROPHONE || esp_lv_adapter_lock(-1) != ESP_OK) {
        return;
    }
    lv_label_set_text(s_status_label, status);
    lv_label_set_text(s_networks_label, message);
    lv_obj_set_style_bg_color(s_status_dot, lv_color_hex(0x48C7D1), LV_PART_MAIN);
    esp_lv_adapter_unlock();
}

static esp_err_t initialize_microphone(void)
{
    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_16,
        .scl_io_num = GPIO_NUM_15,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_err_t err = i2c_new_master_bus(&i2c_config, &i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    const i2c_device_config_t codec_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x18,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t codec = NULL;
    err = i2c_master_bus_add_device(i2c_bus, &codec_config, &codec);
    if (err != ESP_OK) {
        return err;
    }

    const i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    err = i2s_new_channel(&channel_config, NULL, &s_mic_rx_channel);
    if (err != ESP_OK) {
        return err;
    }

    i2s_std_config_t audio_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_4,
            .bclk = GPIO_NUM_5,
            .ws = GPIO_NUM_7,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_6,
            .invert_flags = {0},
        },
    };
    audio_config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    err = i2s_channel_init_std_mode(s_mic_rx_channel, &audio_config);
    if (err != ESP_OK) {
        return err;
    }
    err = i2s_channel_enable(s_mic_rx_channel);
    if (err != ESP_OK) {
        return err;
    }

    static const uint8_t codec_registers[][2] = {
        {0x00, 0x1F}, {0x00, 0x00}, {0x01, 0x3F}, {0x02, 0x08},
        {0x03, 0x10}, {0x04, 0x10}, {0x05, 0x00}, {0x06, 0x03},
        {0x07, 0x00}, {0x08, 0xFF}, {0x09, 0x0C}, {0x0A, 0x0C},
        {0x0B, 0x00}, {0x0C, 0x00}, {0x0D, 0x01}, {0x0E, 0x02},
        {0x12, 0x00}, {0x13, 0x10}, {0x14, 0x1A}, {0x15, 0x00},
        {0x16, 0x24}, {0x17, 0xC8}, {0x1B, 0x0A}, {0x1C, 0x6A},
        {0x31, 0x60}, {0x37, 0x08}, {0x00, 0x80},
    };
    for (size_t i = 0; i < sizeof(codec_registers) / sizeof(codec_registers[0]); ++i) {
        err = es8311_write_register(codec, codec_registers[i][0], codec_registers[i][1]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static void microphone_task(void *arg)
{
    esp_err_t err = initialize_microphone();
    if (err != ESP_OK) {
        s_mic_failed = true;
        ESP_LOGE(TAG, "Initialisation micro ES8311 echouee: %s", esp_err_to_name(err));
        set_mic_message("Erreur microphone", "Verifier codec ES8311 / I2C");
        vTaskDelete(NULL);
    }

    s_mic_ready = true;
    set_mic_message("Micro actif", "Parle pour voir le niveau");
    int16_t samples[256];
    while (true) {
        if (s_scan_view != SCAN_VIEW_MICROPHONE) {
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }

        size_t bytes_read = 0;
        err = i2s_channel_read(s_mic_rx_channel, samples, sizeof(samples),
                               &bytes_read, 100);
        if (err != ESP_OK || bytes_read < sizeof(int16_t)) {
            continue;
        }

        size_t sample_count = bytes_read / sizeof(samples[0]);
        int64_t sample_sum = 0;
        for (size_t i = 0; i < sample_count; ++i) {
            sample_sum += samples[i];
        }
        int32_t mean = (int32_t)(sample_sum / (int64_t)sample_count);
        uint32_t peak = 0;
        for (size_t i = 0; i < sample_count; ++i) {
            int32_t centered = (int32_t)samples[i] - mean;
            uint32_t magnitude = (uint32_t)(centered < 0 ? -centered : centered);
            if (magnitude > peak) {
                peak = magnitude;
            }
        }
        uint32_t level = (peak * 100U) / 500U;
        if (level > 100U) {
            level = 100U;
        }

        if (esp_lv_adapter_lock(-1) == ESP_OK) {
            lv_label_set_text_fmt(s_networks_label, "Niveau sonore : %u%%", (unsigned)level);
            lv_bar_set_value(s_audio_bar, (int32_t)level, LV_ANIM_OFF);
            esp_lv_adapter_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static void set_scan_message(scan_view_t view, const char *status, const char *networks)
{
    (void)view;
    if (esp_lv_adapter_lock(-1) != ESP_OK) {
        return;
    }
    if (s_status_label != NULL) {
        lv_label_set_text(s_status_label, status);
    }
    if (s_networks_label != NULL) {
        lv_label_set_text(s_networks_label, networks);
    }
    lv_color_t dot_color = lv_color_hex(0x48C7D1);
    if (strstr(status, "Erreur") != NULL || strstr(status, "impossible") != NULL ||
        strstr(status, "Echec") != NULL) {
        dot_color = lv_color_hex(0xF07167);
        set_status_led(48, 0, 0);
    } else if (strstr(status, "connecte") != NULL || strstr(status, "fini") != NULL) {
        dot_color = lv_color_hex(0x8BD49C);
        set_status_led(0, 40, 0);
    } else if (strstr(status, "Bluetooth") != NULL || strstr(status, "BLE") != NULL) {
        dot_color = lv_color_hex(0x48C7D1);
        set_status_led(0, 0, 40);
    } else if (strstr(status, "Connexion") != NULL) {
        dot_color = lv_color_hex(0xF7D154);
        set_status_led(36, 14, 0);
    } else if (strstr(status, "Recherche") != NULL) {
        dot_color = lv_color_hex(0xF7D154);
        set_status_led(36, 14, 0);
    }
    if (s_status_dot != NULL) {
        lv_obj_set_style_bg_color(s_status_dot, dot_color, LV_PART_MAIN);
    }
    esp_lv_adapter_unlock();
}

static void format_ble_devices(char *buffer, size_t buffer_size)
{
    size_t used = 0;
    buffer[0] = '\0';

    for (size_t i = 0; i < s_ble_device_count && used < buffer_size; ++i) {
        const ble_device_t *device = &s_ble_devices[i];
        int written = snprintf(buffer + used, buffer_size - used,
                               "%u. %.18s  %d dBm\n", (unsigned)(i + 1),
                               device->name[0] != '\0' ? device->name : "Appareil BLE",
                               device->rssi);
        if (written < 0 || (size_t)written >= buffer_size - used) {
            break;
        }
        used += (size_t)written;
    }

    if (used == 0) {
        snprintf(buffer, buffer_size, "Aucun appareil trouve");
    }
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC) {
        const struct ble_hs_adv_fields *fields = NULL;
        struct ble_hs_adv_fields parsed_fields;
        if (ble_hs_adv_parse_fields(&parsed_fields, event->disc.data,
                                    event->disc.length_data) == 0) {
            fields = &parsed_fields;
        }

        size_t index = 0;
        while (index < s_ble_device_count &&
               memcmp(s_ble_devices[index].address, event->disc.addr.val, 6) != 0) {
            ++index;
        }

        if (index == s_ble_device_count && s_ble_device_count < BLE_DEVICE_LIMIT) {
            ++s_ble_device_count;
            memcpy(s_ble_devices[index].address, event->disc.addr.val, 6);
            snprintf(s_ble_devices[index].name, BLE_DEVICE_NAME_SIZE,
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     event->disc.addr.val[5], event->disc.addr.val[4],
                     event->disc.addr.val[3], event->disc.addr.val[2],
                     event->disc.addr.val[1], event->disc.addr.val[0]);
        }

        if (index < s_ble_device_count && fields != NULL &&
            fields->name != NULL && fields->name_len > 0) {
            size_t name_size = fields->name_len < BLE_DEVICE_NAME_SIZE - 1
                                   ? fields->name_len : BLE_DEVICE_NAME_SIZE - 1;
            memcpy(s_ble_devices[index].name, fields->name, name_size);
            s_ble_devices[index].name[name_size] = '\0';
        }

        if (index < s_ble_device_count) {
            s_ble_devices[index].rssi = event->disc.rssi;
        }

        char results[512];
        format_ble_devices(results, sizeof(results));
        char status[48];
        snprintf(status, sizeof(status), "%u appareil(s) BLE", (unsigned)s_ble_device_count);
        set_scan_message(SCAN_VIEW_BLUETOOTH, status, results);
        return 0;
    }

    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        s_ble_scan_active = false;
        char results[512];
        format_ble_devices(results, sizeof(results));
        char status[48];
        snprintf(status, sizeof(status), "Scan fini : %u appareil(s)",
                 (unsigned)s_ble_device_count);
        set_scan_message(SCAN_VIEW_BLUETOOTH, status, results);
    }
    return 0;
}

static void start_ble_scan(void)
{
    if (s_ble_init_failed) {
        set_scan_message(SCAN_VIEW_BLUETOOTH, "Erreur d'initialisation BLE",
                         "Redemarre la carte");
        return;
    }
    if (!s_ble_ready || s_ble_scan_active) {
        set_scan_message(SCAN_VIEW_BLUETOOTH, "Bluetooth en demarrage...",
                         "Patiente un instant");
        return;
    }

    memset(s_ble_devices, 0, sizeof(s_ble_devices));
    s_ble_device_count = 0;
    s_ble_scan_active = true;
    set_scan_message(SCAN_VIEW_BLUETOOTH, "Recherche Bluetooth BLE...",
                     "Recherche pendant 10 secondes");

    const struct ble_gap_disc_params scan_parameters = {
        .passive = 0,
        .itvl = 0x50,
        .window = 0x30,
        .filter_duplicates = 0,
    };
    int err = ble_gap_disc(s_ble_own_address_type, 10000, &scan_parameters,
                           ble_gap_event_cb, NULL);
    if (err != 0) {
        s_ble_scan_active = false;
        char error_message[48];
        snprintf(error_message, sizeof(error_message), "Erreur BLE: %d", err);
        set_scan_message(SCAN_VIEW_BLUETOOTH, error_message, "Scan impossible");
    }
}

static void ble_on_sync(void)
{
    int err = ble_hs_util_ensure_addr(0);
    if (err == 0) {
        err = ble_hs_id_infer_auto(0, &s_ble_own_address_type);
    }
    if (err != 0) {
        ESP_LOGE(TAG, "Impossible de determiner l'adresse BLE: %d", err);
        return;
    }
    s_ble_ready = true;
    if (s_scan_view == SCAN_VIEW_BLUETOOTH) {
        start_ble_scan();
    }
}

static void ble_host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void bluetooth_init_task(void *arg)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        char error_message[48];
        snprintf(error_message, sizeof(error_message), "Erreur init BLE: %s",
                 esp_err_to_name(err));
        s_ble_init_failed = true;
        set_scan_message(SCAN_VIEW_BLUETOOTH, error_message, "Bluetooth indisponible");
        vTaskDelete(NULL);
    }

    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);
    vTaskDelete(NULL);
}

static void boot_button_task(void *arg)
{
    bool button_was_pressed = false;
    TickType_t button_pressed_at = 0;

    while (true) {
        bool button_is_pressed = gpio_get_level(BOOT_BUTTON_PIN) == 0;
        if (button_is_pressed && !button_was_pressed) {
            vTaskDelay(pdMS_TO_TICKS(40));
            if (gpio_get_level(BOOT_BUTTON_PIN) == 0) {
                button_pressed_at = xTaskGetTickCount();
                button_was_pressed = true;
            }
        } else if (!button_is_pressed && button_was_pressed) {
            TickType_t press_duration = xTaskGetTickCount() - button_pressed_at;
            if (press_duration >= pdMS_TO_TICKS(800)) {
                s_satellite_view = false;
                if (esp_lv_adapter_lock(-1) == ESP_OK) {
                    lv_screen_load(s_weather_screen);
                    esp_lv_adapter_unlock();
                }
            } else if (!s_satellite_view && esp_lv_adapter_lock(-1) == ESP_OK) {
                bool on_forecast = lv_screen_active() == s_forecast_screen;
                if (on_forecast) {
                    s_satellite_view = true;
                    lv_screen_load(s_satellite_screen);
                } else if (lv_screen_active() == s_weather_screen) {
                    lv_screen_load(s_forecast_screen);
                }
                esp_lv_adapter_unlock();
                if (on_forecast) {
                    s_satellite_refresh_requested = true;
                    set_satellite_message(s_wifi_connected
                                              ? "Chargement de la derniere image..."
                                              : "En attente du Wi-Fi...");
                }
            } else if (s_satellite_view) {
                if (esp_lv_adapter_lock(-1) == ESP_OK) {
                    lv_screen_load(s_weather_screen);
                    esp_lv_adapter_unlock();
                }
                s_satellite_view = false;
            }            button_was_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void fetch_montpellier_weather(void)
{
    if (s_weather_label == NULL) {
        return;
    }

    if (!wifi_credentials_configured() || !s_wifi_connected) {
        if (esp_lv_adapter_lock(-1) == ESP_OK) {
            lv_label_set_text(s_weather_label, "--°C");
            lv_label_set_text(s_weather_condition_label, "Wi-Fi requis");
            lv_label_set_text(s_networks_label, "Verifie la connexion");
            esp_lv_adapter_unlock();
        }
        return;
    }

    static weather_response_t response;
    memset(&response, 0, sizeof(response));
    const esp_http_client_config_t config = {
        .url = "https://api.open-meteo.com/v1/forecast?latitude=43.6108&longitude=3.8767&current=temperature_2m,relative_humidity_2m,apparent_temperature,weather_code,wind_speed_10m,uv_index,is_day,precipitation&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max,uv_index_max&forecast_days=7&hourly=precipitation,precipitation_probability&forecast_hours=24&timezone=Europe%2FParis",
        .event_handler = weather_http_event_handler,
        .user_data = &response,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = client != NULL ? esp_http_client_perform(client) : ESP_ERR_NO_MEM;
    int status_code = client != NULL ? esp_http_client_get_status_code(client) : 0;
    if (client != NULL) {
        esp_http_client_cleanup(client);
    }

    char temperature_text[24] = "--°C";
    char condition_text[48] = "Meteo indisponible";
    char details_text[320] = "Verifie la connexion Wi-Fi";
    char rain_text[64] = "Pluie: prevision indisponible";
    int rain_level = 0;
    int icon_code = -1;
    bool icon_is_day = true;
    if (err != ESP_OK || status_code != 200 || response.overflow) {
        ESP_LOGW(TAG, "Recuperation meteo echouee: %s, HTTP %d%s",
                 esp_err_to_name(err), status_code, response.overflow ? ", reponse trop longue" : "");
        rain_level = 1;
        snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: INCONNU - meteo indisponible");
    } else {
        jparse_ctx_t json_context = {0};
        double temperature = 0.0;
        double apparent = 0.0;
        double humidity = 0.0;
        double current_precipitation = 0.0;
        double wind_speed = 0.0;
        double uv_index = 0.0;
        double temperature_max = 0.0;
        double temperature_min = 0.0;
        double precipitation_probability = 0.0;
        double uv_index_max = 0.0;
        int code = -1;
        int is_day = 1;
        double rain_slots[24] = {0};
        int rain_slot_count = 0;
        int rain_probability_count = 0;
        double rain_probability_slots[24] = {0};
        char fc_date[FORECAST_DAYS][12] = {{0}};
        int fc_code[FORECAST_DAYS] = {0};
        double fc_tmin[FORECAST_DAYS] = {0};
        double fc_tmax[FORECAST_DAYS] = {0};
        int fc_rain[FORECAST_DAYS] = {0};
        int fc_count = 0;
        bool values_valid = false;
        if (json_parse_start(&json_context, response.data, (int)response.length) == OS_SUCCESS) {
            if (json_obj_get_object(&json_context, "current") == OS_SUCCESS) {
                values_valid =
                    json_obj_get_double(&json_context, "temperature_2m", &temperature) == OS_SUCCESS &&
                    json_obj_get_double(&json_context, "apparent_temperature", &apparent) == OS_SUCCESS &&
                    json_obj_get_double(&json_context, "relative_humidity_2m", &humidity) == OS_SUCCESS &&
                    json_obj_get_double(&json_context, "precipitation", &current_precipitation) == OS_SUCCESS &&
                    json_obj_get_double(&json_context, "wind_speed_10m", &wind_speed) == OS_SUCCESS &&
                    json_obj_get_double(&json_context, "uv_index", &uv_index) == OS_SUCCESS &&
                    json_obj_get_int(&json_context, "weather_code", &code) == OS_SUCCESS;
                json_obj_get_int(&json_context, "is_day", &is_day);
                json_obj_leave_object(&json_context);
            }
            if (values_valid && json_obj_get_object(&json_context, "daily") == OS_SUCCESS) {
                values_valid =
                    json_get_first_array_double(&json_context, "temperature_2m_min", &temperature_min) &&
                    json_get_first_array_double(&json_context, "temperature_2m_max", &temperature_max) &&
                    json_get_first_array_double(&json_context, "precipitation_probability_max",
                                                &precipitation_probability) &&
                    json_get_first_array_double(&json_context, "uv_index_max", &uv_index_max);
                int n = 0;
                if (json_obj_get_array(&json_context, "time", &n) == OS_SUCCESS) {
                    for (int i = 0; i < n && i < FORECAST_DAYS; ++i) {
                        json_arr_get_string(&json_context, i, fc_date[i], sizeof(fc_date[i]));
                    }
                    fc_count = n < FORECAST_DAYS ? n : FORECAST_DAYS;
                    json_obj_leave_array(&json_context);
                }
                if (json_obj_get_array(&json_context, "weather_code", &n) == OS_SUCCESS) {
                    for (int i = 0; i < n && i < FORECAST_DAYS; ++i) {
                        json_arr_get_int(&json_context, i, &fc_code[i]);
                    }
                    json_obj_leave_array(&json_context);
                }
                if (json_obj_get_array(&json_context, "temperature_2m_min", &n) == OS_SUCCESS) {
                    for (int i = 0; i < n && i < FORECAST_DAYS; ++i) {
                        json_arr_get_double(&json_context, i, &fc_tmin[i]);
                    }
                    json_obj_leave_array(&json_context);
                }
                if (json_obj_get_array(&json_context, "temperature_2m_max", &n) == OS_SUCCESS) {
                    for (int i = 0; i < n && i < FORECAST_DAYS; ++i) {
                        json_arr_get_double(&json_context, i, &fc_tmax[i]);
                    }
                    json_obj_leave_array(&json_context);
                }
                if (json_obj_get_array(&json_context, "precipitation_probability_max", &n) == OS_SUCCESS) {
                    for (int i = 0; i < n && i < FORECAST_DAYS; ++i) {
                        double p = 0;
                        json_arr_get_double(&json_context, i, &p);
                        fc_rain[i] = (int)(p + 0.5);
                    }
                    json_obj_leave_array(&json_context);
                }
                json_obj_leave_object(&json_context);
            } else {
                values_valid = false;
            }
            int slot_total = 0;
            if (values_valid && json_obj_get_object(&json_context, "hourly") == OS_SUCCESS) {
                if (json_obj_get_array(&json_context, "precipitation", &slot_total) == OS_SUCCESS) {
                    for (int i = 0; i < slot_total && i < 24; ++i) {
                        double amount = 0.0;
                        json_arr_get_double(&json_context, i, &amount);
                        rain_slots[rain_slot_count++] = amount;
                    }
                    json_obj_leave_array(&json_context);
                }
                if (json_obj_get_array(&json_context, "precipitation_probability",
                                       &slot_total) == OS_SUCCESS) {
                    for (int i = 0; i < slot_total && i < 24; ++i) {
                        double probability = 0.0;
                        if (json_arr_get_double(&json_context, i, &probability) == OS_SUCCESS) {
                            rain_probability_slots[rain_probability_count++] = probability;
                        }
                    }
                    json_obj_leave_array(&json_context);
                }
                json_obj_leave_object(&json_context);
            }
            json_parse_end(&json_context);
        }
        if (values_valid) {
            const char *condition = "Temps variable";
            switch (code) {
            case 0: condition = "Soleil"; break;
            case 1: condition = "Peu nuageux"; break;
            case 2: condition = "Nuageux"; break;
            case 3: condition = "Couvert"; break;
            case 45: case 48: condition = "Brouillard"; break;
            case 51: case 53: case 55: condition = "Bruine"; break;
            case 56: case 57: case 66: case 67: condition = "Pluie verglacante"; break;
            case 61: case 63: case 65: case 80: case 81: case 82: condition = "Pluie"; break;
            case 71: case 73: case 75: case 77: case 85: case 86: condition = "Neige"; break;
            case 95: case 96: case 99: condition = "Orage"; break;
            default: break;
            }
            snprintf(temperature_text, sizeof(temperature_text), "%.0f°C", temperature);
            snprintf(condition_text, sizeof(condition_text), "%s", condition);
            snprintf(details_text, sizeof(details_text),
                     "#9FB8C0 Ressenti# #FFD27A %.0f°#   #9FB8C0 Humidite# #7FD6FF %.0f%%#\n#9FB8C0 Mini# #7FB5FF %.0f°#   #9FB8C0 Maxi# #FF8A6B %.0f°#\n#9FB8C0 Pluie# #5FA8FF %.0f%%#   #9FB8C0 Vent# #A6F0C6 %.0f km/h#\n#9FB8C0 UV# #FFE45E %.1f# #9FB8C0 (max# #FFB347 %.1f#)#9FB8C0 )#",
                     apparent, humidity, temperature_min, temperature_max,
                     precipitation_probability, wind_speed, uv_index, uv_index_max);
            set_weather_led(code, temperature);
            icon_code = code;
            icon_is_day = is_day != 0;

            double rain_total = 0.0;
            int first_rain = -1;
            for (int i = 0; i < rain_slot_count; ++i) {
                rain_total += rain_slots[i];
                if (first_rain < 0 && rain_slots[i] >= 0.1) {
                    first_rain = i;
                }
            }
            int max_rain_probability = 0;
            int max_rain_probability_slot = 0;
            for (int i = 0; i < rain_probability_count; ++i) {
                int probability = (int)(rain_probability_slots[i] + 0.5);
                if (probability > max_rain_probability) {
                    max_rain_probability = probability;
                    max_rain_probability_slot = i;
                }
            }
            if (current_precipitation >= 0.1) {
                rain_level = 2;
                snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: OUI - en cours");
            } else if (first_rain > 0) {
                rain_level = 2;
                snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: OUI - vers +%dh (%.1f mm)",
                         first_rain, rain_total);
            } else if (max_rain_probability >= 50) {
                rain_level = 2;
                snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: RISQUE %d%% vers +%dh",
                         max_rain_probability, max_rain_probability_slot);
            } else if (max_rain_probability >= 30) {
                rain_level = 1;
                snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: RISQUE FAIBLE %d%%",
                         max_rain_probability);
            } else {
                rain_level = 0;
                snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: NON - pas de risque");
            }
            ESP_LOGI(TAG, "Rain 24h: current %.1f mm, total %.1f mm, first wet hour %d, max probability %d%%",
                     current_precipitation, rain_total, first_rain, max_rain_probability);

            if (fc_count > 0 && esp_lv_adapter_lock(-1) == ESP_OK) {
                for (int i = 0; i < fc_count; ++i) {
                    set_forecast_row(i, fc_date[i], fc_code[i], fc_tmin[i], fc_tmax[i], fc_rain[i]);
                }
                esp_lv_adapter_unlock();
            }
        } else {
            snprintf(condition_text, sizeof(condition_text), "Donnees meteo invalides");
            snprintf(details_text, sizeof(details_text), "Nouvel essai dans 10 min");
            rain_level = 1;
            snprintf(rain_text, sizeof(rain_text), "PLUIE 24H: INCONNU - meteo indisponible");
            ESP_LOGW(TAG, "La reponse meteo ne contient pas les donnees attendues");
        }
    }

    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        snprintf(s_weather_text, sizeof(s_weather_text), "%s", details_text);
        lv_label_set_text(s_weather_label, temperature_text);
        lv_label_set_text(s_weather_condition_label, condition_text);
        lv_label_set_text(s_networks_label, details_text);
        set_rain_banner(rain_level, rain_text);
        if (icon_code >= 0) {
            set_weather_icon(icon_code, icon_is_day);
        }
        if (strstr(condition_text, "indisponible") != NULL ||
            strstr(condition_text, "invalides") != NULL) {
            lv_label_set_text(s_status_label, "Meteo indisponible");
        }
        esp_lv_adapter_unlock();
    }
}

static void wifi_scan_task(void *arg)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        set_scan_message(SCAN_VIEW_WIFI, "Erreur reseau", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        set_scan_message(SCAN_VIEW_WIFI, "Erreur reseau", esp_err_to_name(err));
        vTaskDelete(NULL);
    }
    if (esp_netif_create_default_wifi_sta() == NULL) {
        set_scan_message(SCAN_VIEW_WIFI, "Erreur reseau", "Creation Wi-Fi impossible");
        vTaskDelete(NULL);
    }

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_config);
    if (err == ESP_OK) {
        err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    }
    if (err == ESP_OK) {
        err = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (err == ESP_OK && wifi_credentials_configured()) {
        wifi_config_t wifi_config = {0};
        snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid),
                 "%s", CONFIG_APP_WIFI_SSID);
        snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password),
                 "%s", CONFIG_APP_WIFI_PASSWORD);
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            set_scan_message(SCAN_VIEW_WIFI, "Erreur Wi-Fi", "Memoire insuffisante");
            vTaskDelete(NULL);
        }
        err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                         wifi_event_handler, NULL);
        if (err == ESP_OK) {
            err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wifi_event_handler, NULL);
        }
        if (err == ESP_OK) {
            err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        }
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err != ESP_OK) {
        set_scan_message(SCAN_VIEW_WIFI, "Erreur Wi-Fi", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    if (wifi_credentials_configured()) {
        set_scan_message(SCAN_VIEW_WIFI, "Connexion au Wi-Fi...", "Synchronisation de l'heure");
        EventBits_t connection_bits = xEventGroupWaitBits(
            s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));
        if ((connection_bits & WIFI_CONNECTED_BIT) != 0) {
            const esp_sntp_config_t sntp_config =
                ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
            err = esp_netif_sntp_init(&sntp_config);
            if (err == ESP_OK) {
                set_scan_message(SCAN_VIEW_WIFI, "Wi-Fi connecte", "Heure en synchronisation");
            } else {
                set_scan_message(SCAN_VIEW_WIFI, "Wi-Fi connecte", "Erreur de synchronisation NTP");
            }
            fetch_montpellier_weather();
        } else {
            set_scan_message(SCAN_VIEW_WIFI, "Connexion Wi-Fi impossible",
                             "Verifie le SSID et le mot de passe");
        }
    }

    TickType_t last_weather_update = xTaskGetTickCount();
    TickType_t last_satellite_update = 0;
    while (true) {
        time_t current_time;
        time(&current_time);
        bool daylight = current_time >= 1700000000 &&
                        is_daylight_in_montpellier(current_time);
        TickType_t now_ticks = xTaskGetTickCount();
        bool satellite_mode_changed =
            s_satellite_mode_known && daylight != s_satellite_mode_daylight;
        bool satellite_refresh_due =
            s_satellite_view && s_satellite_mode_known &&
            (now_ticks - last_satellite_update) >= pdMS_TO_TICKS(SATELLITE_REFRESH_MS);
        if (s_satellite_refresh_requested ||
            (s_satellite_view && s_wifi_connected &&
             (satellite_mode_changed || satellite_refresh_due))) {
            s_satellite_refresh_requested = false;
            fetch_montpellier_satellite_image();
            last_satellite_update = xTaskGetTickCount();
        }
        if (wifi_credentials_configured() &&
            (xTaskGetTickCount() - last_weather_update) >= pdMS_TO_TICKS(5 * 60 * 1000)) {
            fetch_montpellier_weather();
            last_weather_update = xTaskGetTickCount();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initialisation de l'ecran ES3N28P");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    const spi_bus_config_t bus_config = {
        .sclk_io_num = 12,
        .mosi_io_num = 11,
        .miso_io_num = 13,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_H_RES * 50 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));

    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = 10,
        .dc_gpio_num = 46,
        .spi_mode = 0,
        .pclk_hz = 20 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t panel_io = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &panel_io));

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RESET_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(gpio_reset_pin(LCD_BL_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(LCD_BL_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(LCD_BL_PIN, 1));

    const led_strip_config_t status_led_config = {
        .strip_gpio_num = STATUS_LED_PIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {.invert_out = false},
    };
    const led_strip_rmt_config_t status_led_rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {.with_dma = false},
    };
    esp_err_t led_err = led_strip_new_rmt_device(&status_led_config,
                                                 &status_led_rmt_config,
                                                 &s_status_led);
    if (led_err != ESP_OK) {
        ESP_LOGW(TAG, "LED RGB indisponible: %s", esp_err_to_name(led_err));
    }

    ESP_ERROR_CHECK(gpio_config(&(gpio_config_t){
        .pin_bit_mask = 1ULL << BOOT_BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    }));

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    esp_lv_adapter_config_t adapter_config = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_lv_adapter_init(&adapter_config));

    const esp_lv_adapter_display_config_t display_config =
        ESP_LV_ADAPTER_DISPLAY_SPI_WITHOUT_PSRAM_DEFAULT_CONFIG(
            panel, panel_io, LCD_H_RES, LCD_V_RES, ESP_LV_ADAPTER_ROTATE_0);
    lv_display_t *display = esp_lv_adapter_register_display(&display_config);
    assert(display != NULL);

    ESP_ERROR_CHECK(esp_lv_adapter_start());
    ESP_ERROR_CHECK(esp_lv_adapter_lock(-1));

    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x09212B), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    create_background_animation(screen);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(screen, LV_DIR_NONE);

    s_date_label = lv_label_create(screen);
    lv_label_set_text(s_date_label, "");
    lv_obj_align(s_date_label, LV_ALIGN_TOP_LEFT, 16, 16);
    lv_obj_set_style_text_color(s_date_label, lv_color_hex(0x77D8D2), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_date_label, &lv_font_montserrat_14, LV_PART_MAIN);

    s_clock_label = lv_label_create(screen);
    lv_label_set_text(s_clock_label, "--:--");
    lv_obj_align(s_clock_label, LV_ALIGN_TOP_RIGHT, -16, 16);
    lv_obj_set_style_text_color(s_clock_label, lv_color_hex(0xF4F0E8), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_clock_label, &lv_font_montserrat_14, LV_PART_MAIN);

    s_status_dot = lv_obj_create(screen);
    lv_obj_set_size(s_status_dot, 8, 8);
    lv_obj_align(s_status_dot, LV_ALIGN_TOP_LEFT, 17, 55);
    lv_obj_set_style_radius(s_status_dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_status_dot, lv_color_hex(0xF7D154), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_status_dot, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_status_dot, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(s_status_dot, LV_DIR_NONE);

    s_status_label = lv_label_create(screen);
    lv_label_set_text(s_status_label, "Connexion au Wi-Fi...");
    lv_obj_align(s_status_label, LV_ALIGN_TOP_LEFT, 32, 50);
    lv_obj_set_width(s_status_label, 120);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xA9C1C8), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_10, LV_PART_MAIN);

    s_wifi_signal_label = lv_label_create(screen);
    lv_label_set_text(s_wifi_signal_label, "");
    lv_label_set_recolor(s_wifi_signal_label, true);
    lv_obj_set_width(s_wifi_signal_label, 96);
    lv_label_set_long_mode(s_wifi_signal_label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_align(s_wifi_signal_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_wifi_signal_label, lv_color_hex(0x8FA6AD), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_wifi_signal_label, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(s_wifi_signal_label, LV_ALIGN_TOP_RIGHT, -12, 44);

    lv_obj_t *results_panel = lv_obj_create(screen);
    lv_obj_set_size(results_panel, LCD_H_RES - 24, 206);
    lv_obj_align(results_panel, LV_ALIGN_TOP_LEFT, 12, 78);
    lv_obj_set_style_bg_opa(results_panel, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(results_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(results_panel, 20, LV_PART_MAIN);
    lv_obj_set_style_pad_all(results_panel, 8, LV_PART_MAIN);
    lv_obj_set_scroll_dir(results_panel, LV_DIR_NONE);

    lv_obj_t *now_label = lv_label_create(results_panel);
    lv_label_set_text(now_label, "ACTUELLEMENT");
    lv_obj_align(now_label, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_color(now_label, lv_color_hex(0x94D6D5), LV_PART_MAIN);
    lv_obj_set_style_text_font(now_label, &lv_font_montserrat_10, LV_PART_MAIN);

    s_weather_icon = lv_obj_create(results_panel);
    lv_obj_set_size(s_weather_icon, 56, 56);
    lv_obj_align(s_weather_icon, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_set_style_bg_opa(s_weather_icon, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_weather_icon, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_weather_icon, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(s_weather_icon, LV_DIR_NONE);

    s_weather_label = lv_label_create(results_panel);
    lv_label_set_text(s_weather_label, "--°C");
    lv_obj_align(s_weather_label, LV_ALIGN_TOP_LEFT, 0, 16);
    lv_obj_set_style_text_color(s_weather_label, lv_color_hex(0xFFF3D1), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_weather_label, &lv_font_montserrat_28, LV_PART_MAIN);

    s_weather_condition_label = lv_label_create(results_panel);
    lv_label_set_text(s_weather_condition_label, "En attente...");
    lv_obj_set_width(s_weather_condition_label, LCD_H_RES - 48);
    lv_obj_align(s_weather_condition_label, LV_ALIGN_TOP_LEFT, 0, 56);
    lv_obj_set_style_text_color(s_weather_condition_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_weather_condition_label, &lv_font_montserrat_14, LV_PART_MAIN);

    s_networks_label = lv_label_create(results_panel);
    lv_label_set_text(s_networks_label, "Ressenti --°C\nHumidite --%");
    lv_obj_set_width(s_networks_label, LCD_H_RES - 48);
    lv_obj_align(s_networks_label, LV_ALIGN_TOP_LEFT, 0, 86);
    lv_obj_set_style_text_color(s_networks_label, lv_color_hex(0xC3E4E6), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_networks_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_label_set_recolor(s_networks_label, true);
    lv_obj_set_style_text_line_space(s_networks_label, 8, LV_PART_MAIN);

    s_rain_banner = lv_obj_create(screen);
    lv_obj_set_size(s_rain_banner, LCD_H_RES - 24, 28);
    lv_obj_align(s_rain_banner, LV_ALIGN_TOP_LEFT, 12, 290);
    lv_obj_set_style_bg_color(s_rain_banner, lv_color_hex(0x1F6F54), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_rain_banner, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_rain_banner, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_rain_banner, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_rain_banner, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(s_rain_banner, LV_DIR_NONE);
    s_rain_label = lv_label_create(s_rain_banner);
    lv_label_set_text(s_rain_label, "Prevision de pluie...");
    lv_obj_center(s_rain_label);
    lv_obj_set_style_text_color(s_rain_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_rain_label, &lv_font_montserrat_10, LV_PART_MAIN);

    esp_lv_adapter_unlock();

    ESP_ERROR_CHECK(esp_lv_adapter_lock(-1));
    s_weather_screen = screen;
    s_satellite_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_satellite_screen, lv_color_hex(0x071820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_satellite_screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_scroll_dir(s_satellite_screen, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(s_satellite_screen, LV_SCROLLBAR_MODE_OFF);

    s_satellite_image = lv_image_create(s_satellite_screen);
    lv_obj_set_size(s_satellite_image, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT);
    lv_obj_align(s_satellite_image, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *satellite_title = lv_label_create(s_satellite_screen);
    lv_label_set_text(satellite_title, "SATELLITE METEOSAT");
    lv_obj_align(satellite_title, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_color(satellite_title, lv_color_hex(0x9CE3E0), LV_PART_MAIN);
    lv_obj_set_style_text_font(satellite_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(satellite_title, lv_color_hex(0x071820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(satellite_title, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_pad_all(satellite_title, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(satellite_title, 4, LV_PART_MAIN);

    s_satellite_status = lv_label_create(s_satellite_screen);
    lv_label_set_text(s_satellite_status, "Appui court BOOT pour charger");
    lv_obj_set_width(s_satellite_status, LCD_H_RES - 12);
    lv_obj_set_style_text_align(s_satellite_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(s_satellite_status, lv_color_hex(0xD5E0DF), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_satellite_status, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(s_satellite_status, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_bg_color(s_satellite_status, lv_color_hex(0x071820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_satellite_status, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_satellite_status, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(s_satellite_status, 4, LV_PART_MAIN);

    lv_obj_t *satellite_hint = lv_label_create(s_satellite_screen);
    lv_label_set_text(satellite_hint, "BOOT: suivant  |  appui long: accueil");
    lv_obj_set_width(satellite_hint, LCD_H_RES - 8);
    lv_obj_set_style_text_align(satellite_hint, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(satellite_hint, lv_color_hex(0x789096), LV_PART_MAIN);
    lv_obj_set_style_text_font(satellite_hint, &lv_font_montserrat_10, LV_PART_MAIN);
    lv_obj_align(satellite_hint, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_set_style_bg_color(satellite_hint, lv_color_hex(0x071820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(satellite_hint, LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_pad_all(satellite_hint, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(satellite_hint, 4, LV_PART_MAIN);
    esp_lv_adapter_unlock();

    ESP_ERROR_CHECK(esp_lv_adapter_lock(-1));
    s_forecast_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_forecast_screen, lv_color_hex(0x0B1620), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_forecast_screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_forecast_screen, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(s_forecast_screen, LV_DIR_NONE);
    create_background_animation(s_forecast_screen);
    lv_obj_set_scrollbar_mode(s_forecast_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_t *fc_title = lv_label_create(s_forecast_screen);
    lv_label_set_text(fc_title, "PREVISIONS 7 JOURS");
    lv_obj_set_style_text_color(fc_title, lv_color_hex(0x9CE3E0), LV_PART_MAIN);
    lv_obj_set_style_text_font(fc_title, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(fc_title, LV_ALIGN_TOP_MID, 0, 6);
    for (int i = 0; i < FORECAST_DAYS; ++i) {
        lv_obj_t *row = lv_obj_create(s_forecast_screen);
        lv_obj_set_size(row, LCD_H_RES - 12, 38);
        lv_obj_set_pos(row, 6, 28 + i * 41);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
        lv_obj_set_style_border_color(row, lv_color_hex(0x2F5568), LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
        lv_obj_set_style_radius(row, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
        lv_obj_set_scroll_dir(row, LV_DIR_NONE);

        s_fc_day[i] = lv_label_create(row);
        lv_label_set_text(s_fc_day[i], "--");
        lv_obj_set_pos(s_fc_day[i], 8, 4);
        lv_obj_set_style_text_color(s_fc_day[i], lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        lv_obj_set_style_text_font(s_fc_day[i], &lv_font_montserrat_14, LV_PART_MAIN);

        s_fc_cond[i] = lv_label_create(row);
        lv_label_set_text(s_fc_cond[i], "");
        lv_obj_set_pos(s_fc_cond[i], 8, 23);
        lv_obj_set_style_text_font(s_fc_cond[i], &lv_font_montserrat_10, LV_PART_MAIN);

        s_fc_temp[i] = lv_label_create(row);
        lv_label_set_text(s_fc_temp[i], "-- / --");
        lv_obj_set_pos(s_fc_temp[i], 78, 10);
        lv_obj_set_style_text_color(s_fc_temp[i], lv_color_hex(0xFFD27A), LV_PART_MAIN);
        lv_obj_set_style_text_font(s_fc_temp[i], &lv_font_montserrat_14, LV_PART_MAIN);

        s_fc_rain[i] = lv_label_create(row);
        lv_label_set_text(s_fc_rain[i], "--%");
        lv_obj_set_pos(s_fc_rain[i], 168, 4);
        lv_obj_set_style_text_font(s_fc_rain[i], &lv_font_montserrat_14, LV_PART_MAIN);

        s_fc_bar[i] = lv_bar_create(row);
        lv_obj_set_size(s_fc_bar[i], 56, 6);
        lv_obj_set_pos(s_fc_bar[i], 160, 25);
        lv_bar_set_range(s_fc_bar[i], 0, 100);
        lv_obj_set_style_bg_color(s_fc_bar[i], lv_color_hex(0x2A4256), LV_PART_MAIN);
        lv_obj_set_style_pad_all(s_fc_bar[i], 0, LV_PART_MAIN);
    }
    esp_lv_adapter_unlock();
    BaseType_t task_created = xTaskCreate(wifi_scan_task, "wifi_scan", 16384, NULL, 5, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    task_created = xTaskCreate(clock_task, "clock", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    task_created = xTaskCreate(boot_button_task, "boot_button", 3072, NULL, 5, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
