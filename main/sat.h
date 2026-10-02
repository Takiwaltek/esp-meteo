#pragma once
#include <stdbool.h>
#include "lvgl.h"
/* Télécharge les SAT_FRAMES images (4 h) en PSRAM. Retourne le nombre d'images obtenues. */
int sat_load_all(void (*progress)(int done, int total));
int sat_count(void);
const lv_image_dsc_t *sat_frame(int i);
