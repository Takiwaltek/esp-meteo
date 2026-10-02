#include "weather.h"
#include <string.h>
#include <stdlib.h>
#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "weather";

const char *weather_text(int c)
{
    if (c == 0) return "Ciel dégagé";
    if (c <= 2) return "Peu nuageux";
    if (c == 3) return "Couvert";
    if (c == 45 || c == 48) return "Brouillard";
    if (c >= 51 && c <= 57) return "Bruine";
    if (c >= 61 && c <= 67) return "Pluie";
    if (c >= 71 && c <= 77) return "Neige";
    if (c >= 80 && c <= 82) return "Averses";
    if (c == 85 || c == 86) return "Averses de neige";
    if (c >= 95) return "Orage";
    return "Inconnu";
}

static double num_at(cJSON *arr, int i)
{
    cJSON *it = cJSON_GetArrayItem(arr, i);
    return cJSON_IsNumber(it) ? it->valuedouble : 0;
}

bool weather_fetch(weather_t *w)
{
    esp_http_client_config_t cfg = { .url = METEO_URL, .crt_bundle_attach = esp_crt_bundle_attach, .timeout_ms = 10000 };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c || esp_http_client_open(c, 0) != ESP_OK) { if (c) esp_http_client_cleanup(c); return false; }
    esp_http_client_fetch_headers(c);
    char *buf = malloc(8192);
    int n = 0, r;
    while (buf && n < 8191 && (r = esp_http_client_read(c, buf + n, 8191 - n)) > 0) n += r;
    esp_http_client_close(c); esp_http_client_cleanup(c);
    if (!buf) return false;
    buf[n] = 0;
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) { ESP_LOGE(TAG, "JSON invalide"); return false; }

    cJSON *cur = cJSON_GetObjectItem(root, "current"), *hr = cJSON_GetObjectItem(root, "hourly"), *dl = cJSON_GetObjectItem(root, "daily");
    bool ok = cur && hr && dl;
    if (ok) {
        memset(w, 0, sizeof *w);
        #define CUR(k) cJSON_GetObjectItem(cur, k)->valuedouble
        w->temp = CUR("temperature_2m"); w->feels = CUR("apparent_temperature");
        w->humidity = (int)CUR("relative_humidity_2m"); w->code = (int)CUR("weather_code");
        w->wind = CUR("wind_speed_10m"); w->is_day = (int)CUR("is_day");
        w->tmax = num_at(cJSON_GetObjectItem(dl, "temperature_2m_max"), 0);
        w->tmin = num_at(cJSON_GetObjectItem(dl, "temperature_2m_min"), 0);
        w->uv = num_at(cJSON_GetObjectItem(dl, "uv_index_max"), 0);
        w->rain_sum = num_at(cJSON_GetObjectItem(dl, "precipitation_sum"), 0);
        cJSON *pp = cJSON_GetObjectItem(hr, "precipitation_probability"), *pr = cJSON_GetObjectItem(hr, "precipitation");
        w->rain_hour_offset = -1;
        for (int i = 0; i < cJSON_GetArraySize(pr) && i < 12; i++) {
            int p = (int)num_at(pp, i);
            if (p > w->rain_prob_max) w->rain_prob_max = p;
            if (num_at(pr, i) >= 0.1 && p >= 40 && w->rain_hour_offset < 0) w->rain_hour_offset = i;
        }
        w->rain_soon = w->rain_hour_offset >= 0;
    }
    cJSON_Delete(root);
    return ok;
}
