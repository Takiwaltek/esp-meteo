#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
/* Décode un PNG 8 bits non entrelacé (gris, RVB, palette, RVBA) en RGB565 (w*h*2 octets).
 * Utilise inflate (tinfl) de la ROM de l'ESP32. out doit être alloué par l'appelant. */
bool png_decode_rgb565(const uint8_t *png, size_t len, uint16_t *out, int max_w, int max_h, int *w, int *h);
