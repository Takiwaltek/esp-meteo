#include "ui.h"
#include <stdio.h>
#include <time.h>
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "sat.h"
#include "config.h"

static lv_obj_t *scr_w, *scr_s;
static lv_obj_t *l_time, *l_date, *icon, *l_temp, *l_desc, *banner, *l_banner, *l_status;
static lv_obj_t *l_feel, *l_hum, *l_mm, *l_rain, *l_wind, *l_uv;
static lv_obj_t *sat_img, *sat_lbl, *sat_bar;
static lv_timer_t *sat_timer;
static int sat_idx;

static const char *jours[] = {"dimanche", "lundi", "mardi", "mercredi", "jeudi", "vendredi", "samedi"};
static const char *mois[] = {"janvier", "février", "mars", "avril", "mai", "juin", "juillet", "août", "septembre", "octobre", "novembre", "décembre"};

static lv_obj_t *label(lv_obj_t *p, const lv_font_t *f, lv_color_t c)
{
    lv_obj_t *l = lv_label_create(p);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    return l;
}

static lv_obj_t *shape(lv_obj_t *p, int x, int y, int w, int h, int r, lv_color_t c)
{
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_style_radius(o, r, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(o, c, 0);
    return o;
}

static void draw_icon(int code, int is_day)
{
    lv_obj_clean(icon);
    lv_color_t sun = lv_color_hex(0xFFC107), cloud = lv_color_hex(0xB0BEC5), dark = lv_color_hex(0x78909C);
    bool clouds = code >= 2, sunny = code <= 2 && is_day;
    bool rain = (code >= 51 && code <= 67) || (code >= 80 && code <= 82) || code >= 95;
    bool snow = (code >= 71 && code <= 77) || code == 85 || code == 86;
    bool fog = code == 45 || code == 48;
    if (code <= 2 && !is_day) shape(icon, 14, 6, 30, 30, 15, lv_color_hex(0xE0E0E0));
    if (sunny) {
        shape(icon, code == 0 ? 16 : 6, code == 0 ? 16 : 4, 32, 32, 16, sun);
        if (code == 0) for (int i = 0; i < 8; i++) {
            static const int8_t d[8][2] = {{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};
            shape(icon, 30 + d[i][0] * 28, 30 + d[i][1] * 28, 4, 4, 2, sun);
        }
    }
    if (clouds || fog) {
        lv_color_t c = code >= 95 || rain ? dark : cloud;
        shape(icon, 8, 30, 52, 22, 11, c);
        shape(icon, 18, 18, 26, 26, 13, c);
        shape(icon, 34, 24, 22, 22, 11, c);
    }
    if (fog) for (int i = 0; i < 3; i++) shape(icon, 8, 46 + i * 6, 52, 3, 1, lv_color_hex(0xCFD8DC));
    if (rain) for (int i = 0; i < 4; i++) shape(icon, 14 + i * 12, 54, 3, 9, 1, lv_color_hex(0x42A5F5));
    if (snow) for (int i = 0; i < 4; i++) shape(icon, 14 + i * 12, 56, 5, 5, 2, lv_color_white());
    if (code >= 95) shape(icon, 30, 46, 8, 16, 1, sun);
}

static lv_color_t temp_color(float t)
{
    return t < 5 ? lv_color_hex(0x64B5F6) : t < 15 ? lv_color_hex(0x4DD0E1) : t < 25 ? lv_color_hex(0xFFD54F) : lv_color_hex(0xFF7043);
}

void ui_init(void)
{
    lvgl_port_lock(0);
    scr_w = lv_obj_create(NULL);
    scr_s = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_w, lv_color_hex(0x10151C), 0);
    lv_obj_set_style_bg_color(scr_s, lv_color_black(), 0);
    lv_obj_remove_flag(scr_w, LV_OBJ_FLAG_SCROLLABLE);

    l_time = label(scr_w, &lv_font_montserrat_48, lv_color_white());
    lv_label_set_text(l_time, "--:--");
    lv_obj_align(l_time, LV_ALIGN_TOP_MID, 0, 6);
    l_date = label(scr_w, &lv_font_montserrat_14, lv_color_hex(0x90A4AE));
    lv_label_set_text(l_date, "");
    lv_obj_align(l_date, LV_ALIGN_TOP_MID, 0, 62);

    icon = lv_obj_create(scr_w);
    lv_obj_remove_style_all(icon);
    lv_obj_set_size(icon, 70, 70);
    lv_obj_align(icon, LV_ALIGN_TOP_LEFT, 14, 92);
    l_temp = label(scr_w, &lv_font_montserrat_48, lv_color_white());
    lv_label_set_text(l_temp, "--°");
    lv_obj_align(l_temp, LV_ALIGN_TOP_LEFT, 100, 96);
    l_desc = label(scr_w, &lv_font_montserrat_14, lv_color_hex(0xCFD8DC));
    lv_label_set_text(l_desc, "Montpellier");
    lv_obj_align(l_desc, LV_ALIGN_TOP_LEFT, 100, 150);

    lv_obj_t **lbl[6] = {&l_feel, &l_hum, &l_mm, &l_rain, &l_wind, &l_uv};
    lv_color_t col[6] = {lv_color_hex(0xFF8A65), lv_color_hex(0x4FC3F7), lv_color_hex(0xAED581), lv_color_hex(0x7986CB), lv_color_hex(0x80CBC4), lv_color_hex(0xBA68C8)};
    for (int i = 0; i < 6; i++) {
        *lbl[i] = label(scr_w, &lv_font_montserrat_14, col[i]);
        lv_label_set_text(*lbl[i], "");
        lv_obj_align(*lbl[i], LV_ALIGN_TOP_LEFT, 14, 178 + i * 20);
    }

    banner = shape(scr_w, 0, 290, LCD_W, 30, 0, lv_color_hex(0x1565C0));
    l_banner = label(banner, &lv_font_montserrat_14, lv_color_white());
    lv_obj_center(l_banner);
    lv_obj_add_flag(banner, LV_OBJ_FLAG_HIDDEN);
    l_status = label(scr_w, &lv_font_montserrat_14, lv_color_hex(0xFFAB91));
    lv_label_set_text(l_status, "");
    lv_obj_align(l_status, LV_ALIGN_BOTTOM_MID, 0, -2);

    sat_img = lv_image_create(scr_s);
    lv_obj_align(sat_img, LV_ALIGN_TOP_LEFT, 0, 0);
    sat_lbl = label(scr_s, &lv_font_montserrat_14, lv_color_white());
    lv_obj_set_style_bg_opa(sat_lbl, LV_OPA_50, 0);
    lv_obj_set_style_bg_color(sat_lbl, lv_color_black(), 0);
    lv_label_set_text(sat_lbl, "Meteosat IR");
    lv_obj_align(sat_lbl, LV_ALIGN_TOP_LEFT, 4, 4);
    sat_bar = lv_bar_create(scr_s);
    lv_obj_set_size(sat_bar, 160, 10);
    lv_obj_center(sat_bar);
    lv_bar_set_range(sat_bar, 0, SAT_FRAMES);
    lv_obj_add_flag(sat_bar, LV_OBJ_FLAG_HIDDEN);

    lv_screen_load(scr_w);
    lvgl_port_unlock();
}

void ui_update_clock(void)
{
    time_t now = time(NULL);
    struct tm tm; localtime_r(&now, &tm);
    lvgl_port_lock(0);
    if (tm.tm_year < 120) {
        lv_label_set_text(l_time, "--:--");
        lv_label_set_text(l_date, "synchronisation...");
    } else {
        lv_label_set_text_fmt(l_time, "%02d:%02d", tm.tm_hour, tm.tm_min);
        lv_label_set_text_fmt(l_date, "%s %d %s %d", jours[tm.tm_wday], tm.tm_mday, mois[tm.tm_mon], tm.tm_year + 1900);
    }
    lvgl_port_unlock();
}

void ui_show_status(const char *msg)
{
    lvgl_port_lock(0);
    lv_label_set_text(l_status, msg);
    lvgl_port_unlock();
}

void ui_show_weather(const weather_t *w)
{
    lvgl_port_lock(0);
    draw_icon(w->code, w->is_day);
    lv_label_set_text_fmt(l_temp, "%.0f°", w->temp);
    lv_obj_set_style_text_color(l_temp, temp_color(w->temp), 0);
    lv_label_set_text(l_desc, weather_text(w->code));
    lv_label_set_text_fmt(l_feel, "Ressenti : %.0f° (mini %.0f° / maxi %.0f°)", w->feels, w->tmin, w->tmax);
    lv_label_set_text_fmt(l_hum, "Humidité : %d %%", w->humidity);
    lv_label_set_text_fmt(l_mm, "Pluie du jour : %.1f mm", w->rain_sum);
    lv_label_set_text_fmt(l_rain, "Risque de pluie 12 h : %d %%", w->rain_prob_max);
    lv_label_set_text_fmt(l_wind, "Vent : %.0f km/h", w->wind);
    lv_label_set_text_fmt(l_uv, "UV max : %.0f", w->uv);
    lv_label_set_text(l_status, "");
    if (w->rain_soon) {
        if (w->rain_hour_offset == 0) lv_label_set_text(l_banner, "Pluie en cours ou imminente");
        else lv_label_set_text_fmt(l_banner, "Pluie prévue dans %d h", w->rain_hour_offset);
        lv_obj_remove_flag(banner, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(banner, LV_OBJ_FLAG_HIDDEN);
    }
    lvgl_port_unlock();
}

void ui_sat_start(void)
{
    lvgl_port_lock(0);
    lv_image_set_src(sat_img, NULL);
    lv_label_set_text(sat_lbl, "Chargement Meteosat IR...");
    lv_obj_remove_flag(sat_bar, LV_OBJ_FLAG_HIDDEN);
    lv_bar_set_value(sat_bar, 0, LV_ANIM_OFF);
    lv_screen_load(scr_s);
    lvgl_port_unlock();
}

void ui_sat_progress(int done, int total)
{
    lvgl_port_lock(0);
    lv_bar_set_value(sat_bar, done, LV_ANIM_OFF);
    lvgl_port_unlock();
}

static void sat_tick(lv_timer_t *t)
{
    int n = sat_count();
    if (!n) return;
    sat_idx = (sat_idx + 1) % n;
    lv_image_set_src(sat_img, sat_frame(sat_idx));
    lv_label_set_text_fmt(sat_lbl, "Meteosat IR  %d/%d  (-%d h %02d)", sat_idx + 1, n,
                          (n - 1 - sat_idx) * SAT_STEP_MIN / 60, (n - 1 - sat_idx) * SAT_STEP_MIN % 60);
}

void ui_sat_show(void)
{
    lvgl_port_lock(0);
    lv_obj_add_flag(sat_bar, LV_OBJ_FLAG_HIDDEN);
    if (!sat_count()) {
        lv_label_set_text(sat_lbl, "Images indisponibles");
    } else {
        sat_idx = -1;
        sat_tick(NULL);
        if (!sat_timer) sat_timer = lv_timer_create(sat_tick, 400, NULL);
        lv_timer_resume(sat_timer);
    }
    lvgl_port_unlock();
}

void ui_sat_stop(void)
{
    lvgl_port_lock(0);
    if (sat_timer) lv_timer_pause(sat_timer);
    lv_screen_load(scr_w);
    lvgl_port_unlock();
}
