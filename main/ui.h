#pragma once
#include "weather.h"
void ui_init(void);
void ui_update_clock(void);
void ui_show_weather(const weather_t *w);
void ui_show_status(const char *msg);
void ui_sat_start(void);   /* affiche et anime le satellite */
void ui_sat_progress(int done, int total);
void ui_sat_show(void);
void ui_sat_stop(void);    /* retour à la météo */
