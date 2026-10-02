#pragma once
#include <stdbool.h>
typedef struct {
    float temp, feels, wind, tmin, tmax, uv, rain_sum;
    int humidity, code, is_day;
    bool rain_soon;       /* pluie dans les 12 prochaines heures */
    int rain_hour_offset; /* heures avant la première pluie */
    int rain_prob_max;
} weather_t;
bool weather_fetch(weather_t *w);
const char *weather_text(int code);
