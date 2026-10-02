#pragma once
/* Wi-Fi: renseigner via menuconfig-less defines ou -D à la compilation */
#define WIFI_SSID "VOTRE_SSID"
#define WIFI_PASS "VOTRE_MOT_DE_PASSE"

/* Écran ILI9341 (SPI2) */
#define PIN_LCD_MOSI 11
#define PIN_LCD_SCLK 12
#define PIN_LCD_CS   10
#define PIN_LCD_DC   9
#define PIN_LCD_RST  8
#define PIN_LCD_BL   7
#define LCD_W 240
#define LCD_H 320

#define PIN_BOOT_BTN 0
#define PIN_LED      48   /* LED RGB WS2812 de la carte */
#define LONG_PRESS_MS 800

/* Montpellier */
#define METEO_URL \
 "https://api.open-meteo.com/v1/forecast?latitude=43.61&longitude=3.88" \
 "&current=temperature_2m,apparent_temperature,relative_humidity_2m,weather_code,wind_speed_10m,is_day" \
 "&hourly=precipitation_probability,precipitation&forecast_hours=12" \
 "&daily=temperature_2m_max,temperature_2m_min,precipitation_sum,uv_index_max" \
 "&timezone=Europe%2FParis&forecast_days=1"
#define TZ_PARIS "CET-1CEST,M3.5.0,M10.5.0/3"

/* Meteosat infrarouge, Europe : WMS EUMETSAT */
#define SAT_FRAMES 8
#define SAT_STEP_MIN 30
#define SAT_URL \
 "https://view.eumetsat.int/geoserver/wms?service=WMS&version=1.1.1&request=GetMap" \
 "&layers=mtg_fd:ir105_hrfi&styles=&format=image/png&srs=EPSG:4326" \
 "&bbox=-12,34,18,64&width=%d&height=%d&time=%s"
