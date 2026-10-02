#include "sat.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "config.h"
#include "png.h"

static const char *TAG = "sat";
#define MAX_PNG (400 * 1024)
static lv_image_dsc_t frames[SAT_FRAMES];
static uint16_t *pix[SAT_FRAMES];
static int count;

int sat_count(void) { return count; }
const lv_image_dsc_t *sat_frame(int i) { return &frames[i]; }

static int download(const char *url, uint8_t *buf, int max)
{
    esp_http_client_config_t cfg = { .url = url, .crt_bundle_attach = esp_crt_bundle_attach, .timeout_ms = 20000 };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c || esp_http_client_open(c, 0) != ESP_OK) { if (c) esp_http_client_cleanup(c); return -1; }
    esp_http_client_fetch_headers(c);
    int n = 0, r;
    while (n < max && (r = esp_http_client_read(c, (char *)buf + n, max - n)) > 0) n += r;
    int status = esp_http_client_get_status_code(c);
    esp_http_client_close(c); esp_http_client_cleanup(c);
    return status == 200 ? n : -1;
}

int sat_load_all(void (*progress)(int, int))
{
    uint8_t *buf = heap_caps_malloc(MAX_PNG, MALLOC_CAP_SPIRAM);
    if (!buf) return 0;
    /* images de la plus ancienne à la plus récente; Meteosat publie avec ~30 min de retard */
    time_t now = time(NULL);
    time_t last = (now / (SAT_STEP_MIN * 60)) * (SAT_STEP_MIN * 60) - SAT_STEP_MIN * 60;
    count = 0;
    for (int i = 0; i < SAT_FRAMES; i++) {
        if (progress) progress(i, SAT_FRAMES);
        time_t t = last - (time_t)(SAT_FRAMES - 1 - i) * SAT_STEP_MIN * 60;
        struct tm tm; gmtime_r(&t, &tm);
        char ts[32], url[400];
        strftime(ts, sizeof ts, "%Y-%m-%dT%H:%M:00.000Z", &tm);
        snprintf(url, sizeof url, SAT_URL, LCD_W, LCD_H, ts);
        int n = download(url, buf, MAX_PNG);
        if (n <= 0) { ESP_LOGW(TAG, "image %d indisponible", i); continue; }
        if (!pix[count]) pix[count] = heap_caps_malloc(LCD_W * LCD_H * 2, MALLOC_CAP_SPIRAM);
        int w, h;
        if (!pix[count] || !png_decode_rgb565(buf, n, pix[count], LCD_W, LCD_H, &w, &h)) continue;
        frames[count] = (lv_image_dsc_t){
            .header = { .magic = LV_IMAGE_HEADER_MAGIC, .cf = LV_COLOR_FORMAT_RGB565, .w = w, .h = h, .stride = w * 2 },
            .data_size = w * h * 2, .data = (const uint8_t *)pix[count] };
        count++;
    }
    heap_caps_free(buf);
    if (progress) progress(SAT_FRAMES, SAT_FRAMES);
    return count;
}
